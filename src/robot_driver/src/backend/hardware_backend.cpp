#include "robot_driver/backend/hardware_backend.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <ctime>
#include <cstdint>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <pthread.h>
#include <rclcpp/rclcpp.hpp>
#include <sched.h>

namespace robot_driver
{

namespace
{

static const auto kLogger = rclcpp::get_logger("hardware_backend");

#ifdef WORKER_RT
constexpr int kWorkerRtPriority = 70;
#endif

#ifdef WORKER_LOOP_DIAG
long long timespecDiffUs(const timespec & lhs, const timespec & rhs)
{
  const auto sec = static_cast<long long>(lhs.tv_sec) - static_cast<long long>(rhs.tv_sec);
  const auto nsec = static_cast<long long>(lhs.tv_nsec) - static_cast<long long>(rhs.tv_nsec);
  return sec * 1000000LL + nsec / 1000LL;
}
#endif

}  // namespace

// 加载通用配置，并记录 backend 名称。
//
// 构造函数只做纯数据加载，不打开真实硬件链路；具体链路初始化由派生 backend 控制。
HardwareBackend::HardwareBackend(const BackendConfig & config, std::string backend_name)
: backend_config_(config),
  joints_config_(robot_common::loadRobotJointsConfig(config.robot_joints_config)),
  hardware_config_(config.hardware_config),
  config_mode_(config.config_mode),
  backend_name_(std::move(backend_name))
{
}

// 析构时停止统一 worker，避免派生类释放通讯对象时后台线程仍在访问硬件。
HardwareBackend::~HardwareBackend()
{
  stopWorker();
}

// 返回 backend 名称。
std::string HardwareBackend::backendName() const
{
  return backend_name_;
}

// 初始化统一关节状态数组和 joint_name 索引。
//
// 所有关节顺序以 robot_joints.yaml 为准，硬件配置只补充 vendor 等底层映射信息。
void HardwareBackend::initializeJoints()
{
  joints_.clear();
  joint_index_.clear();
  joints_.reserve(joints_config_.joints.size());
  for (const auto & joint_config : joints_config_.joints) {
    JointRuntime runtime;
    runtime.name = joint_config.name;
    if (const auto * hardware_joint = hardware_config_.getJoint(joint_config.name)) {
      runtime.vendor = hardware_joint->vendor;
    }
    joint_index_.emplace(joint_config.name, joints_.size());
    joints_.push_back(runtime);
  }
}

// 设置统一 worker 周期。
//
// cycle_ms 来自硬件配置；非法值回退到 2ms，避免出现 0 周期忙等。
void HardwareBackend::setCycleTimeMs(double cycle_ms)
{
  if (cycle_ms <= 0.0) {
    cycle_ms = 2.0;
  }
  cycle_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double, std::milli>(cycle_ms));
}

// 查询逻辑关节在 runtime 数组中的下标。
//
// 未知关节名表示上层命令或配置错误，直接抛出异常，避免静默丢弃控制命令。
std::size_t HardwareBackend::requireJointIndex(const std::string & joint_name) const
{
  const auto iter = joint_index_.find(joint_name);
  if (iter == joint_index_.end()) {
    throw std::out_of_range("unknown joint command received by " + backend_name_ + " backend: " + joint_name);
  }
  return iter->second;
}

// 判断关节是否默认参与硬件使能流程。
//
// Tienkung 和 mix 的启动使能流程会用该信息跳过默认禁用的关节。
bool HardwareBackend::jointDefaultEnabled(const std::string & joint_name) const
{
  const auto * joint_config = joints_config_.getJoint(joint_name);
  return joint_config != nullptr && joint_config->enable_default;
}

// 锁存后端故障，并输出统一日志。
//
// 派生类可以覆盖该函数加入重连动作、关闭通讯配置等特殊处理。
void HardwareBackend::setFault(const std::string & message)
{
  FaultDetail detail;
  detail.source = "backend";
  detail.message = message;
  recordFault(detail);
}

void HardwareBackend::recordFault(const FaultDetail & detail)
{
  const auto formatted = formatFaultDetail(detail);
  RCLCPP_ERROR(kLogger, "%s", formatted.c_str());
  fault_active_ = true;
  setStatusMessage(formatted);
}

std::string HardwareBackend::formatFaultDetail(const FaultDetail & detail) const
{
  std::ostringstream stream;
  stream << backend_name_ << " fault";
  if (!detail.source.empty()) {
    stream << " source=" << detail.source;
  }
  if (!detail.transport.empty()) {
    stream << " transport=" << detail.transport;
  }
  if (!detail.message.empty()) {
    stream << " message=\"" << detail.message << "\"";
  }
  if (!detail.joint_name.empty()) {
    stream << " joint=" << detail.joint_name;
  }
  if (detail.motor_id >= 0) {
    stream << " motor_id=" << detail.motor_id;
  }
  if (detail.bus >= 0) {
    stream << " bus=" << detail.bus;
  }
  if (detail.slave_id >= 0) {
    stream << " slave_id=" << detail.slave_id;
  }
  if (detail.passage >= 0) {
    stream << " passage=" << detail.passage;
  }
  if (detail.expected_work_counter >= 0 || detail.actual_work_counter >= 0) {
    stream << " wkc=" << detail.actual_work_counter << "/" << detail.expected_work_counter;
  }
  if (detail.detected_slave_count >= 0 || detail.expected_slave_count >= 0) {
    stream << " slaves=" << detail.detected_slave_count << "/" << detail.expected_slave_count;
  }
  if (!detail.transport_error.empty()) {
    stream << " transport_error=\"" << detail.transport_error << "\"";
  }
  if (!detail.protocol_error.empty()) {
    stream << " protocol_error=\"" << detail.protocol_error << "\"";
  }
  if (!detail.gateway_error.empty()) {
    stream << " gateway_error=\"" << detail.gateway_error << "\"";
  }
  if (!detail.slave_summary.empty()) {
    stream << " slave_summary=\"" << detail.slave_summary << "\"";
  }
  return stream.str();
}

// 更新 DriverStatus 中的短状态文本。
//
// 该函数内部加锁，允许 worker 线程和 ROS 状态读取线程并发访问。
void HardwareBackend::setStatusMessage(const std::string & message)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  status_message_ = message;
}

// 处理上层下发的统一关节命令。
//
// 这里只做关节名校验、限位校验和命令缓存；协议编码统一放到周期线程中执行。
void HardwareBackend::handleCommand(
  const robot_interfaces::msg::JointCommands & command_msg,
  double receive_time_sec)
{
  (void)receive_time_sec;
  std::vector<robot_interfaces::msg::JointCommand> validated_commands;
  validated_commands.reserve(command_msg.commands.size());

  for (const auto & command : command_msg.commands) {
    const auto index = requireJointIndex(command.joint_name);
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      last_command_trace_id_ = command_msg.trace_id;
      last_command_sequence_ = command_msg.sequence;
      auto & joint = joints_[index];

      const auto * joint_config = joints_config_.getJoint(command.joint_name);
      if (joint_config != nullptr && (command.position < joint_config->min_pos || command.position > joint_config->max_pos)) {
        ++joint.invalid_cmd_count;
        if (joint.invalid_cmd_count >= 3) {
          joint.joint_error = robot_interfaces::msg::JointState::JOINT_ERR_INVALID_ANGLE_CMD;
          joint.error_message = "command position " + std::to_string(command.position) +
            " exceeds limits [" + std::to_string(joint_config->min_pos) +
            ", " + std::to_string(joint_config->max_pos) + "]";
        }
        continue;
      }
      joint.invalid_cmd_count = 0;
    }
    validated_commands.push_back(command);
  }

  std::lock_guard<std::mutex> lock(command_mutex_);
  cached_commands_ = std::move(validated_commands);
  has_received_command_ = true;
}

// backend 周期维护入口。
//
// 正常模式下，当底层通讯已经配置完成且未故障时，启动统一 worker 线程。
void HardwareBackend::update(double now_sec)
{
  (void)now_sec;
  if (config_mode_) {
    return;
  }
  if (!fault_active_.load() && configured() && !worker_running_.load()) {
    startWorker();
  }
}

// 生成统一关节状态消息。
//
// 厂家协议、通讯方式和底层单位换算都在派生类 readJointFeedback() 中完成，
// 这里仅把统一 runtime 状态转换为 ROS 消息。
robot_interfaces::msg::JointStates HardwareBackend::jointStates() const
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  robot_interfaces::msg::JointStates msg;
  msg.sequence = ++sequence_;
  msg.last_command_trace_id = last_command_trace_id_;
  msg.last_command_sequence = last_command_sequence_;
  msg.states.reserve(joints_.size());

  for (const auto & joint : joints_) {
    robot_interfaces::msg::JointState state;
    state.joint_name = joint.name;
    state.position = joint.position;
    state.velocity = joint.velocity;
    state.effort = joint.effort;
    state.enabled = joint.enabled;
    state.joint_error = joint.joint_error;
    state.motor_error = joint.motor_error;
    state.error_message = joint.error_message;
    state.motor_temp = joint.motor_temp;
    state.mos_temp = joint.mos_temp;
    msg.states.push_back(state);
  }
  return msg;
}

// 生成统一驱动状态消息。
//
// 故障状态只上报统一短状态；具体底层错误通过各 master/backend 的日志定位。
robot_interfaces::msg::DriverStatus HardwareBackend::driverStatus() const
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  robot_interfaces::msg::DriverStatus msg;
  msg.status_message = status_message_;
  msg.total_joints = static_cast<std::uint32_t>(joints_.size());
  for (const auto & joint : joints_) {
    if (joint.enabled) {
      ++msg.active_joints;
    }
  }

  if (fault_active_.load()) {
    msg.status = robot_interfaces::msg::DriverStatus::STATUS_FAULT;
    msg.fault_active = true;
    msg.online = false;
    msg.faulted_joints = msg.total_joints;
    return msg;
  }

  msg.status = robot_interfaces::msg::DriverStatus::STATUS_ONLINE;
  msg.fault_active = false;
  msg.online = true;
  return msg;
}

// 启动统一后台 worker。
//
// 使用 atomic exchange 避免 update() 被频繁调用时重复创建线程。
void HardwareBackend::startWorker()
{
  if (worker_running_.exchange(true)) {
    return;
  }
  worker_ = std::thread(&HardwareBackend::workerLoop, this);
#ifdef WORKER_RT
  sched_param param{};
  param.sched_priority = kWorkerRtPriority;
  const int rc = pthread_setschedparam(worker_.native_handle(), SCHED_FIFO, &param);
  if (rc != 0) {
    RCLCPP_WARN(kLogger,
      "%s worker SCHED_FIFO priority=%d setup failed: %s",
      backend_name_.c_str(), kWorkerRtPriority, std::strerror(rc));
  } else {
    RCLCPP_INFO(kLogger,
      "%s worker SCHED_FIFO priority=%d enabled",
      backend_name_.c_str(), kWorkerRtPriority);
  }
#endif
}

// 停止统一后台 worker，并等待线程退出。
void HardwareBackend::stopWorker()
{
  if (!worker_running_.exchange(false)) {
    return;
  }
  if (worker_.joinable()) {
    worker_.join();
  }
}

// 统一 worker 主循环。
//
// 使用 CLOCK_MONOTONIC + TIMER_ABSTIME 做绝对时间调度，减少周期漂移。
void HardwareBackend::workerLoop()
{
  struct timespec next_wake;
  clock_gettime(CLOCK_MONOTONIC, &next_wake);
#ifdef WORKER_LOOP_DIAG
  std::uint64_t diag_cycles = 0;
  std::uint64_t diag_run_sum_us = 0;
  std::uint64_t diag_run_max_us = 0;
  std::uint64_t diag_write_max_us = 0;
  std::uint64_t diag_exchange_max_us = 0;
  std::uint64_t diag_read_max_us = 0;
  std::uint64_t diag_sleep_sum_us = 0;
  std::uint64_t diag_sleep_min_us = std::numeric_limits<std::uint64_t>::max();
  std::uint64_t diag_deadline_missed = 0;
  std::uint64_t diag_max_late_us = 0;
  auto diag_next_report = std::chrono::steady_clock::now() + std::chrono::seconds(1);
#endif

  // 主循环，根据周期时间调度。
  while (worker_running_.load()) {
    struct timespec now_ts;
    clock_gettime(CLOCK_MONOTONIC, &now_ts);
    const double now_sec = static_cast<double>(now_ts.tv_sec) + static_cast<double>(now_ts.tv_nsec) / 1e9;

    // 记录周期开始时间，用于诊断。
#ifdef WORKER_LOOP_DIAG
    const auto cycle_start = std::chrono::steady_clock::now();
#endif

    // 运行周期循环，更新状态。
    if (!fault_active_.load() && configured()) {
      (void)runCycle(now_sec);
    }

    // 记录周期结束时间，用于诊断。
#ifdef WORKER_LOOP_DIAG
    const auto cycle_end = std::chrono::steady_clock::now();
    const auto run_us = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(cycle_end - cycle_start).count());
    ++diag_cycles;
    diag_run_sum_us += run_us;
    diag_run_max_us = std::max(diag_run_max_us, run_us);
    diag_write_max_us = std::max(diag_write_max_us, diag_last_write_us_);
    diag_exchange_max_us = std::max(diag_exchange_max_us, diag_last_exchange_us_);
    diag_read_max_us = std::max(diag_read_max_us, diag_last_read_us_);
#endif

    // 计算下一个唤醒时间。
    next_wake.tv_nsec += cycle_ns_.count();
    while (next_wake.tv_nsec >= 1000000000L) {
      next_wake.tv_nsec -= 1000000000L;
      next_wake.tv_sec += 1;
    }
    // 检查是否错过了周期。
    struct timespec current;
    clock_gettime(CLOCK_MONOTONIC, &current);
    const bool deadline_missed = current.tv_sec > next_wake.tv_sec || (current.tv_sec == next_wake.tv_sec && current.tv_nsec > next_wake.tv_nsec);

    // 记录是否错过了周期，以及延迟时间。
#ifdef WORKER_LOOP_DIAG
    if (deadline_missed) {
      const auto late_us = static_cast<std::uint64_t>(std::max<long long>(0, timespecDiffUs(current, next_wake)));
      ++diag_deadline_missed;
      diag_max_late_us = std::max(diag_max_late_us, late_us);
    } else {
      const auto sleep_budget_us =
        static_cast<std::uint64_t>(std::max<long long>(0, timespecDiffUs(next_wake, current)));
      diag_sleep_sum_us += sleep_budget_us;
      diag_sleep_min_us = std::min(diag_sleep_min_us, sleep_budget_us);
    }
#endif

    // 如果错过了周期，调整下一个唤醒时间。
    if (deadline_missed) {
      next_wake = current;
      next_wake.tv_nsec += cycle_ns_.count();
      while (next_wake.tv_nsec >= 1000000000L) {
        next_wake.tv_nsec -= 1000000000L;
        next_wake.tv_sec += 1;
      }
    }

    // 诊断周期，记录周期运行时间、睡眠时间、是否错过了周期、最大延迟时间等。
#ifdef WORKER_LOOP_DIAG
    if (cycle_end >= diag_next_report && diag_cycles > 0) {
      const auto sleep_samples = diag_cycles - diag_deadline_missed;
      const auto sleep_avg_us = sleep_samples > 0 ? diag_sleep_sum_us / sleep_samples : 0;
      const auto sleep_min_us =
        sleep_samples > 0 && diag_sleep_min_us != std::numeric_limits<std::uint64_t>::max() ? diag_sleep_min_us : 0;
      RCLCPP_INFO(kLogger,
        "%s worker diag: cycle=%ld us run_avg=%lu us run_max=%lu us write_max=%lu us exchange_max=%lu us read_max=%lu us sleep_avg=%lu us sleep_min=%lu us missed=%lu max_late=%lu us",
        backend_name_.c_str(),
        std::chrono::duration_cast<std::chrono::microseconds>(cycle_ns_).count(),
        static_cast<unsigned long>(diag_run_sum_us / diag_cycles),
        static_cast<unsigned long>(diag_run_max_us),
        static_cast<unsigned long>(diag_write_max_us),
        static_cast<unsigned long>(diag_exchange_max_us),
        static_cast<unsigned long>(diag_read_max_us),
        static_cast<unsigned long>(sleep_avg_us),
        static_cast<unsigned long>(sleep_min_us),
        static_cast<unsigned long>(diag_deadline_missed),
        static_cast<unsigned long>(diag_max_late_us));
      diag_cycles = 0;
      diag_run_sum_us = 0;
      diag_run_max_us = 0;
      diag_write_max_us = 0;
      diag_exchange_max_us = 0;
      diag_read_max_us = 0;
      diag_sleep_sum_us = 0;
      diag_sleep_min_us = std::numeric_limits<std::uint64_t>::max();
      diag_deadline_missed = 0;
      diag_max_late_us = 0;
      diag_next_report = cycle_end + std::chrono::seconds(1);
    }
#endif

    // 等待下一个周期开始时间。
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wake, nullptr);
  }
}

// 执行一次统一控制周期。
//
// 固定顺序为：必要启动流程 -> 写输出命令 -> 交换 process data -> 读取反馈。
// 具体通讯方式和协议差异由派生类钩子实现。
bool HardwareBackend::runCycle(double now_sec)
{
  (void)now_sec;
#ifdef WORKER_LOOP_DIAG
  diag_last_write_us_ = 0;
  diag_last_exchange_us_ = 0;
  diag_last_read_us_ = 0;
#endif
  
// 检查是否进入配置模式、故障状态或通讯未就绪。
  if (config_mode_ || fault_active_.load() || !transportReady()) {
    return false;
  }
  // 如果收到命令且未启动，执行启动流程（Encos 不需要额外使能握手，因此默认直接成功）。
  if (has_received_command_.load() && !startup_done_.load()) {
    if (!runStartup()) {
      return false;
    }
    startup_done_ = true;
  }

#ifdef WORKER_LOOP_DIAG
  auto section_start = std::chrono::steady_clock::now();
#endif

  // 写输出命令。
  if (!writeJointCommands()) {
    return false;
  }
  // 记录写输出命令时间。
#ifdef WORKER_LOOP_DIAG
  auto section_end = std::chrono::steady_clock::now();
  diag_last_write_us_ = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(section_end - section_start).count());
  section_start = section_end;
#endif

  // 交换 process data。
  if (!exchangeProcessData()) {
    setFault(backend_name_ + " process data exchange failed");
    return false;
  }
  // 记录交换 process data 时间。
#ifdef WORKER_LOOP_DIAG
  section_end = std::chrono::steady_clock::now();
  diag_last_exchange_us_ = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(section_end - section_start).count());
  section_start = section_end;
#endif

  // 读取反馈。
  readJointFeedback();
  // 记录读取反馈时间。
#ifdef WORKER_LOOP_DIAG
  section_end = std::chrono::steady_clock::now();
  diag_last_read_us_ = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(section_end - section_start).count());
#endif
  return true;
}

// 把当前缓存的所有运动命令写入底层输出区。
//
// 遍历缓存命令并调用派生类 encodeJointCommand()，由具体 backend 选择协议和目标信箱。
bool HardwareBackend::writeJointCommands()
{
  std::vector<robot_interfaces::msg::JointCommand> commands;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    commands = cached_commands_;
  }
  for (const auto & command : commands) {
    if (!encodeJointCommand(command)) {
      return false;
    }
  }
  return true;
}

// 默认启动流程。
//
// Encos 不需要额外使能握手，因此默认直接成功；Tienkung/Mix 覆盖该函数。
bool HardwareBackend::runStartup()
{
  return true;
}

}  // namespace robot_driver
