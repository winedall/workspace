#include "robot_recorder/recorder_node.hpp"

#include "robot_recorder/joint_name_alias.hpp"
#include "robot_recorder/recording_paths.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/qos.hpp"
#include "robot_common/joint_config.hpp"

namespace robot_recorder
{

// ============================================================================
// 构造函数
// ============================================================================
RecorderNode::RecorderNode(const rclcpp::NodeOptions & options)
: Node("robot_recorder_node", options),
  buffer_capacity_(static_cast<size_t>(declare_parameter<int>("buffer_capacity", 2000))),
  flush_interval_ms_(declare_parameter<double>("flush_interval_ms", 100.0)),
  recording_mode_(declare_parameter<std::string>("recording_mode", "event_window")),
  pre_event_window_sec_(declare_parameter<double>("pre_event_window_sec", 180.0)),
  post_event_window_sec_(declare_parameter<double>("post_event_window_sec", 60.0)),
  event_cooldown_sec_(declare_parameter<double>("event_cooldown_sec", 2.0)),
  states_ring_(buffer_capacity_),
  commands_ring_(buffer_capacity_),
  desired_ring_(buffer_capacity_),
  accepted_ring_(buffer_capacity_),
  states_window_(pre_event_window_sec_),
  commands_window_(pre_event_window_sec_),
  desired_window_(pre_event_window_sec_),
  accepted_window_(pre_event_window_sec_),
  event_mgr_(event_cooldown_sec_)
{
  // 检查记录模式是否有效
  if (recording_mode_ != "continuous" && recording_mode_ != "event_window") {
    RCLCPP_ERROR(
      get_logger(), "recording_mode must be 'continuous' or 'event_window', got: %s",
      recording_mode_.c_str());
    throw std::runtime_error("invalid recorder recording_mode");
  }

  // 确保输出目录存在
  output_dir_ = declare_parameter<std::string>("output_dir", "recordings");
  if (!ensureDir(output_dir_)) {
    RCLCPP_ERROR(get_logger(), "Failed to create output directory: %s", output_dir_.c_str());
    throw std::runtime_error("cannot create recorder output directory");
  }
  joints_config_path_ = declare_parameter<std::string>("robot_joints_config", "");
  joint_names_ = robot_common::loadRobotJointsConfig(joints_config_path_).jointNames();

  // 默认 topic 名称
  states_topic_ = "control/hardware_states";
  commands_topic_ = "control/hardware_commands";
  desired_topic_ = "rl_manager/desired_commands";
  accepted_topic_ = "router/accepted_commands";
  imu_topic_ = "imu/nav_all";

  // 记录开关
  record_states_ = declare_parameter<bool>("record_states", true);
  record_commands_ = declare_parameter<bool>("record_commands", true);
  record_desired_ = declare_parameter<bool>("record_desired", true);
  record_accepted_ = declare_parameter<bool>("record_accepted", true);
  record_imu_ = declare_parameter<bool>("record_imu", true);

  if (recording_mode_ == "continuous") {
    initContinuousCsvFiles();
  }

  // 订阅三个命令 topic（每个 topic 独立缓冲，按 topic 写入不同 CSV 文件）
  if (record_desired_) {
    auto cb = [this](const robot_interfaces::msg::JointCommands::SharedPtr msg)
    { onJointCommands(msg, desired_topic_); };
    subs_.push_back(create_subscription<robot_interfaces::msg::JointCommands>(
      desired_topic_, rclcpp::SensorDataQoS(), std::move(cb)));
  }
  if (record_accepted_) {
    auto cb = [this](const robot_interfaces::msg::JointCommands::SharedPtr msg)
    { onJointCommands(msg, accepted_topic_); };
    subs_.push_back(create_subscription<robot_interfaces::msg::JointCommands>(
      accepted_topic_, rclcpp::SensorDataQoS(), std::move(cb)));
  }
  if (record_commands_) {
    auto cb = [this](const robot_interfaces::msg::JointCommands::SharedPtr msg)
    { onJointCommands(msg, commands_topic_); };
    subs_.push_back(create_subscription<robot_interfaces::msg::JointCommands>(
      commands_topic_, rclcpp::SensorDataQoS(), std::move(cb)));
  }

  // 订阅关节状态
  if (record_states_) {
    auto cb = std::bind(&RecorderNode::onJointStates, this, std::placeholders::_1);
    subs_.push_back(create_subscription<robot_interfaces::msg::JointStates>(
      states_topic_, rclcpp::SensorDataQoS(), std::move(cb)));
  }

  // 订阅 IMU 数据（低频率，仅缓存最新值用于状态 CSV 写入）
  if (record_imu_) {
    auto cb = std::bind(&RecorderNode::onNavAll, this, std::placeholders::_1);
    subs_.push_back(create_subscription<yesense_interface::msg::NavAll>(
      imu_topic_, rclcpp::SystemDefaultsQoS(), std::move(cb)));
  }

  // 订阅状态消息用于事件检测。event_window 模式依赖事件触发窗口录制，因此固定启用。
  auto driver_cb = std::bind(&RecorderNode::onDriverStatus, this, std::placeholders::_1);
  subs_.push_back(create_subscription<robot_interfaces::msg::DriverStatus>(
    "driver/status", rclcpp::SystemDefaultsQoS(), std::move(driver_cb)));

  auto cb = std::bind(&RecorderNode::onRobotState, this, std::placeholders::_1);
  subs_.push_back(create_subscription<robot_interfaces::msg::RobotState>(
    "state", rclcpp::SystemDefaultsQoS(), std::move(cb)));

  // 启动后台写入线程
  writer_thread_ = std::thread(&RecorderNode::writerLoop, this);

  RCLCPP_INFO(get_logger(), "Recorder started, output: %s", output_dir_.c_str());
}

RecorderNode::~RecorderNode()
{
  stopWriter();
}

// ============================================================================
// ROS 回调
// ============================================================================

// 关节命令回调
// 根据 topic 选择对应的独立缓冲区和 CSV 文件
void RecorderNode::onJointCommands(
  const robot_interfaces::msg::JointCommands::SharedPtr msg,
  const std::string & topic)
{
  const auto now_ms = now().nanoseconds() / 1000000;
  const auto msg_stamp_ms = toMillis(msg->header.stamp);

  // 根据 topic 选择对应的环形缓冲区
  RingBuffer<JointCommandsRecord> * cmd_ring = nullptr;
  TimeWindowBuffer<JointCommandsRecord> * cmd_window = nullptr;
  if (topic == desired_topic_) {
    cmd_ring = &desired_ring_;
    cmd_window = &desired_window_;
  } else if (topic == accepted_topic_) {
    cmd_ring = &accepted_ring_;
    cmd_window = &accepted_window_;
  } else if (topic == commands_topic_) {
    cmd_ring = &commands_ring_;
    cmd_window = &commands_window_;
  }

  // 常态：详细记录（每关节展开），写入 topic 对应的环形缓冲区
  auto detail_rec = makeJointCommandsRecord(*msg, joint_names_, now_ms, msg_stamp_ms);
  if (recording_mode_ == "event_window") {
    if (cmd_window) {
      cmd_window->push(detail_rec);
    }
  } else if (cmd_ring) {
    static_cast<void>(cmd_ring->tryPush(detail_rec));
  }

  // 缓存最新 sequence，供事件触发时定位
  latest_joint_commands_seq_ = msg->sequence;
}

// 关节状态回调
// 写入 control/hardware_states 详细记录（含最近一次 IMU 数据）
void RecorderNode::onJointStates(
  const robot_interfaces::msg::JointStates::SharedPtr msg)
{
  const auto now_ms = now().nanoseconds() / 1000000;
  const auto msg_stamp_ms = toMillis(msg->header.stamp);

  // 常态：详细记录（每关节 + IMU）
  auto detail_rec = makeJointStatesRecord(*msg, joint_names_, now_ms, msg_stamp_ms, latest_imu_);
  if (recording_mode_ == "event_window") {
    states_window_.push(detail_rec);
  } else {
    static_cast<void>(states_ring_.tryPush(detail_rec));
  }

  // 缓存最新 sequence，供事件触发时定位
  latest_joint_states_seq_ = msg->sequence;

  // 事件管理器回调
  event_mgr_.onJointStates(*msg, now_ms);
}

// IMU 数据回调
// 仅缓存最新值，不写入缓冲区；状态 CSV 在写入时附加缓存值
void RecorderNode::onNavAll(const yesense_interface::msg::NavAll::SharedPtr msg)
{
  latest_imu_.angular_vel_x = msg->gyro.x;
  latest_imu_.angular_vel_y = msg->gyro.y;
  latest_imu_.angular_vel_z = msg->gyro.z;
  latest_imu_.euler_roll = msg->euler.roll;
  latest_imu_.euler_pitch = msg->euler.pitch;
  latest_imu_.euler_yaw = msg->euler.yaw;
}

// 驱动状态回调
// 仅缓存最新值，不写入缓冲区；状态 CSV 在写入时附加缓存值
void RecorderNode::onDriverStatus(const robot_interfaces::msg::DriverStatus::SharedPtr msg)
{
  latest_driver_msg_ = msg->status_message;
}

// ============================================================================
// 工具函数
// ============================================================================

// 创建目录时不抛异常，调用方根据 false 决定是否终止启动或放弃本次事件窗口。
bool RecorderNode::ensureDir(const std::string & path) const
{
  std::error_code ec;
  std::filesystem::create_directories(path, ec);
  return !ec;
}

// ============================================================================
// Writer 线程：周期 drain 缓冲区，批量写入 CSV
// ============================================================================

// 周期 drain 缓冲区，批量写入 CSV 文件
void RecorderNode::writerLoop()
{
  const auto flush_duration = std::chrono::duration<double, std::milli>(flush_interval_ms_);

  // flushBuffers 从所有缓冲区取出记录并写入对应的 CSV 文件
  auto flushBuffers = [this]() {
    // 状态写入：states_ring_ → states_csv_
    auto drain_and_write_states = [this](
      RingBuffer<JointStatesRecord> & buf,
      auto & csv) -> size_t {
      auto batch = buf.drain();
      if (batch.empty() || !csv) return 0;
      try {
        return writeStates(batch, csv);
      } catch (const std::exception & e) {
        write_errors_.fetch_add(1);
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "CSV write error: %s", e.what());
      }
      return 0;
    };

    // 命令写入：各 topic 缓冲区 → 对应的 CSV
    auto drain_and_write_cmds = [this](
      RingBuffer<JointCommandsRecord> & buf,
      auto & csv) -> size_t {
      auto batch = buf.drain();
      if (batch.empty() || !csv) return 0;
      size_t n = 0;
      try {
        writeCommands(batch, csv);
        n = batch.size();
      } catch (const std::exception & e) {
        write_errors_.fetch_add(1);
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "CSV write error: %s", e.what());
      }
      return n;
    };

    size_t written = 0;
    if (recording_mode_ == "continuous") {
      written += drain_and_write_states(states_ring_, states_csv_);
      written += drain_and_write_cmds(commands_ring_, commands_csv_);
      written += drain_and_write_cmds(desired_ring_, desired_csv_);
      written += drain_and_write_cmds(accepted_ring_, accepted_csv_);
    } else if (event_active_) {
      drainWindowBuffersToCsv();
    }

    if (written > 0) {
      if (desired_csv_) desired_csv_->flush();
      if (accepted_csv_) accepted_csv_->flush();
      if (commands_csv_) commands_csv_->flush();
      if (states_csv_) states_csv_->flush();
      written_records_.fetch_add(written);
    }
  };

  // 主循环
  while (running_.load()) {
    std::this_thread::sleep_for(flush_duration);
    if (!running_.load()) {
      break;
    }

    flushBuffers();

    // 事件概要数据写入
    handleEvents();

    if (event_active_ && now().nanoseconds() / 1000000 >= event_end_ms_) {
      stopEventRecording();
    }

  }

  // 退出前最后 flush 一次
  flushBuffers();
  
  // 退出概要数据写入
  handleEvents();
  
  // 关闭所有 CSV 文件
  flushEvents();
}

// 停止 Writer 线程
void RecorderNode::stopWriter()
{
  running_.store(false);
  if (writer_thread_.joinable()) {
    writer_thread_.join();
  }
}

// ============================================================================
// CSV 写入函数
// ============================================================================
// 将一批详细状态记录写入指定的 CSV 文件
size_t RecorderNode::writeStates(
  const std::vector<JointStatesRecord> & batch,
  std::unique_ptr<CsvWriter> & csv)
{
  if (!csv) return 0;
  size_t n = 0;
  for (const auto & rec : batch) {
    const auto & f = rec.frame;
    std::vector<std::string> row;
    row.push_back(std::to_string(rec.recorder_time_ms));
    row.push_back(std::to_string(f.msg_stamp_ms));
    row.push_back(std::to_string(f.sequence));
    row.push_back(std::to_string(f.last_command_trace_id));
    row.push_back(std::to_string(f.last_command_sequence));
    appendStateValues(row, f.joints);
    row.push_back(std::to_string(f.imu.angular_vel_x));
    row.push_back(std::to_string(f.imu.angular_vel_y));
    row.push_back(std::to_string(f.imu.angular_vel_z));
    row.push_back(std::to_string(f.imu.euler_roll));
    row.push_back(std::to_string(f.imu.euler_pitch));
    row.push_back(std::to_string(f.imu.euler_yaw));
    csv->writeRow(row);
    ++n;
  }
  return n;
}

// 将一批详细命令记录写入指定的 CSV 文件
void RecorderNode::writeCommands(
  const std::vector<JointCommandsRecord> & batch,
  std::unique_ptr<CsvWriter> & csv)
{
  if (!csv) return;
  for (const auto & rec : batch) {
    const auto & f = rec.frame;
    std::vector<std::string> row;
    row.push_back(std::to_string(rec.recorder_time_ms));
    row.push_back(std::to_string(f.msg_stamp_ms));
    row.push_back(std::to_string(f.trace_id));
    row.push_back(std::to_string(f.sequence));
    row.push_back(std::to_string(f.parent_sequence));
    row.push_back(f.source);
    row.push_back(f.parent_source);
    row.push_back(f.stage);
    row.push_back(std::to_string(static_cast<int>(f.type)));
    appendCommandValues(row, f.joints);
    csv->writeRow(row);
  }
}

// ============================================================================
// 事件处理
// ============================================================================

// 系统状态回调（事件检测）
// 当 EventManager 检测到新事件触发时，记录故障元数据和当前 sequence
void RecorderNode::onRobotState(const robot_interfaces::msg::RobotState::SharedPtr msg)
{
  const auto now_ms = now().nanoseconds() / 1000000;
  event_mgr_.onRobotState(
    *msg, now_ms, latest_joint_states_seq_, latest_joint_commands_seq_, latest_driver_msg_);
}

// Writer 线程中处理事件数据写入
// 检测到未完成的事件会话，写入事件元数据
void RecorderNode::handleEvents()
{
  while (event_mgr_.hasPendingEvent()) {
    auto session = event_mgr_.takePendingEvent();
    if (recording_mode_ == "event_window") {
      if (!event_active_) {
        startEventRecording(session);
      } else {
        writeEvent(session);
      }
      continue;
    }

    if (!events_csv_) {
      try {
        initEventsCsvForDir(output_dir_);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          get_logger(), "Failed to create events.csv for event %d: %s",
          session.id, e.what());
      }
    }
    writeEvent(session);
  }
}

// 写入单条事件元数据。
void RecorderNode::writeEvent(const EventSession & session)
{
  if (!events_csv_) return;
  const auto now_ms = now().nanoseconds() / 1000000;
  events_csv_->writeRow({
    std::to_string(now_ms),
    std::to_string(session.id),
    std::to_string(session.trigger_time_ms),
    session.event_type,
    session.severity,
    std::to_string(session.trigger_state),
    session.trigger_state_name,
    session.trigger_fault_reason,
    session.trigger_fault_active ? "true" : "false",
    session.joint_name,
    std::to_string(static_cast<int>(session.joint_error)),
    std::to_string(static_cast<int>(session.motor_error)),
    session.error_message,
    std::to_string(session.joint_states_sequence),
    std::to_string(session.joint_commands_sequence),
  });
  events_csv_->flush();
}

// 开始事件窗口记录
void RecorderNode::startEventRecording(const EventSession & session)
{
  // 每个事件单独建目录，便于人工只查看某次故障前后的数据。
  const auto dir = eventRecordingDir(output_dir_, session.id);
  if (!ensureDir(dir)) {
    RCLCPP_ERROR(get_logger(), "Failed to create event recorder directory: %s", dir.c_str());
    return;
  }

  desired_csv_.reset();
  accepted_csv_.reset();
  commands_csv_.reset();
  states_csv_.reset();
  events_csv_.reset();

  // 事件窗口 CSV 使用 robot_joints_config 固定关节顺序，可在事件触发时直接创建。
  if (record_states_) {
    initStatesCsvForDir(dir, states_csv_);
  }
  if (record_commands_) {
    initCommandsCsvForDir(dir, "hardware", commands_csv_);
  }
  if (record_desired_) {
    initCommandsCsvForDir(dir, "desired", desired_csv_);
  }
  if (record_accepted_) {
    initCommandsCsvForDir(dir, "accepted", accepted_csv_);
  }

  initEventsCsvForDir(dir);
  writeEvent(session);

  event_active_ = true;
  event_end_ms_ = session.trigger_time_ms +
    static_cast<int64_t>(post_event_window_sec_ * 1000.0);

  // 先写出已缓存的故障前窗口；后续数据由 writerLoop 周期性 drain。
  drainWindowBuffersToCsv();
  RCLCPP_WARN(
    get_logger(), "Recorder event window started: %s (post %.1fs)",
    dir.c_str(), post_event_window_sec_);
}

// 关闭事件窗口记录
void RecorderNode::stopEventRecording()
{
  // 关闭前再 drain 一次，避免最后一个写周期内的数据丢失。
  drainWindowBuffersToCsv();
  flushEvents();
  desired_csv_.reset();
  accepted_csv_.reset();
  commands_csv_.reset();
  states_csv_.reset();
  events_csv_.reset();
  event_active_ = false;
  event_end_ms_ = 0;
}

// 从事件窗口缓存中 drain 数据并写入 CSV 文件
void RecorderNode::drainWindowBuffersToCsv()
{
  size_t written = 0;
  try {
    // TimeWindowBuffer::drain() 会清空缓存；事件窗口打开期间持续 drain 到 CSV。
    written += writeStates(states_window_.drain(), states_csv_);
    const auto commands = commands_window_.drain();
    writeCommands(commands, commands_csv_);
    written += commands.size();
    const auto desired = desired_window_.drain();
    writeCommands(desired, desired_csv_);
    written += desired.size();
    const auto accepted = accepted_window_.drain();
    writeCommands(accepted, accepted_csv_);
    written += accepted.size();
    if (states_csv_) states_csv_->flush();
    if (desired_csv_) desired_csv_->flush();
    if (accepted_csv_) accepted_csv_->flush();
    if (commands_csv_) commands_csv_->flush();
    if (written > 0) {
      written_records_.fetch_add(written);
    }
  } catch (const std::exception & e) {
    write_errors_.fetch_add(1);
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "CSV write error: %s", e.what());
  }
}

// 关闭事件 CSV 文件（释放资源）
void RecorderNode::flushEvents()
{
  if (events_csv_) events_csv_->flush();
}

// ============================================================================
// CSV 初始化
// ============================================================================
// 初始化常态记录 CSV 文件
void RecorderNode::initContinuousCsvFiles()
{
  if (record_states_) {
    initStatesCsvForDir(output_dir_, states_csv_);
  }
  if (record_commands_) {
    initCommandsCsvForDir(output_dir_, "hardware", commands_csv_);
  }
  if (record_desired_) {
    initCommandsCsvForDir(output_dir_, "desired", desired_csv_);
  }
  if (record_accepted_) {
    initCommandsCsvForDir(output_dir_, "accepted", accepted_csv_);
  }
}

// 初始化常态记录状态 CSV 文件
void RecorderNode::initStatesCsvForDir(
  const std::string & dir,
  std::unique_ptr<CsvWriter> & csv)
{
  if (csv) return;
  std::vector<std::string> headers;
  headers.push_back("recorder_time_ms");
  headers.push_back("msg_stamp_ms");
  headers.push_back("sequence");
  headers.push_back("last_command_trace_id");
  headers.push_back("last_command_sequence");
  // 所有关节按字段分组展开，组间加 --- 分隔。
  std::vector<std::string> aliased_names;
  aliased_names.reserve(joint_names_.size());
  for (const auto & n : joint_names_) {
    aliased_names.push_back(jointNameAlias(n));
  }
  auto value_headers = stateValueHeaders(aliased_names);
  headers.insert(headers.end(), value_headers.begin(), value_headers.end());
  // IMU 列
  headers.push_back("imu_angular_vel_x");
  headers.push_back("imu_angular_vel_y");
  headers.push_back("imu_angular_vel_z");
  headers.push_back("imu_euler_roll");
  headers.push_back("imu_euler_pitch");
  headers.push_back("imu_euler_yaw");
  csv = std::make_unique<CsvWriter>(recordingPath(dir, RecordingFile::kJointStates), headers);
}

// 初始化常态记录命令 CSV 文件
void RecorderNode::initCommandsCsvForDir(
  const std::string & dir,
  const std::string & suffix,
  std::unique_ptr<CsvWriter> & csv)
{
  if (csv) return;
  std::vector<std::string> headers;
  headers.push_back("recorder_time_ms");
  headers.push_back("msg_stamp_ms");
  headers.push_back("trace_id");
  headers.push_back("sequence");
  headers.push_back("parent_sequence");
  headers.push_back("source");
  headers.push_back("parent_source");
  headers.push_back("stage");
  headers.push_back("type");
  // 所有关节按字段分组：pos, vel, tor, kp, kd，组间加 --- 分隔
  std::vector<std::string> aliased_names;
  aliased_names.reserve(joint_names_.size());
  for (const auto & n : joint_names_) {
    aliased_names.push_back(jointNameAlias(n));
  }
  auto value_headers = commandValueHeaders(aliased_names);
  headers.insert(headers.end(), value_headers.begin(), value_headers.end());
  csv = std::make_unique<CsvWriter>(recordingPath(dir, commandRecordingFile(suffix)), headers);
}

// 初始化事件元数据 CSV 文件。
void RecorderNode::initEventsCsvForDir(const std::string & dir)
{
  if (events_csv_) return;
  events_csv_ = std::make_unique<CsvWriter>(
    recordingPath(dir, RecordingFile::kEvents),
    std::vector<std::string>{
      "recorder_time_ms", "event_id", "trigger_time_ms",
      "event_type", "severity",
      "trigger_state", "trigger_state_name", "trigger_fault_reason",
      "trigger_fault_active",
      "joint_name", "joint_error", "motor_error", "error_message",
      "joint_states_sequence", "joint_commands_sequence",
    });
}

}  // namespace robot_recorder
