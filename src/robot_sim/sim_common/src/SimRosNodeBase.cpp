#include "sim_common/SimRosNodeBase.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <functional>
#include <stdexcept>

#include <rclcpp/qos.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace sim_common {

JointStatesLayout BuildJointStatesLayout(
    const std::vector<std::string>& simulated_joint_names,
    const std::vector<PlaceholderGroup>& placeholder_groups) {
  // 锚点必须是仿真关节之一（空锚点表示"插在最前"）。
  for (const auto& group : placeholder_groups) {
    if (group.before.empty()) {
      continue;
    }
    if (std::find(simulated_joint_names.begin(), simulated_joint_names.end(),
                  group.before) == simulated_joint_names.end()) {
      throw std::invalid_argument(
          "JointStates 布局锚点不是仿真关节: " + group.before);
    }
  }

  JointStatesLayout layout;
  layout.sim_to_msg_index.assign(simulated_joint_names.size(), -1);

  std::size_t next_index = 0;
  const auto emit_group = [&layout, &next_index](const PlaceholderGroup& group) {
    for (const auto& name : group.joint_names) {
      layout.placeholders.push_back(PlaceholderJointState{next_index++, name});
    }
  };

  // 1) 前置占位组（真机以头部关节开头时使用）
  for (const auto& group : placeholder_groups) {
    if (group.before.empty()) {
      emit_group(group);
    }
  }
  // 2) 仿真关节，并在锚点前插入其占位组
  for (std::size_t i = 0; i < simulated_joint_names.size(); ++i) {
    for (const auto& group : placeholder_groups) {
      if (group.before == simulated_joint_names[i]) {
        emit_group(group);
      }
    }
    layout.sim_to_msg_index[i] = static_cast<int>(next_index++);
  }
  return layout;
}

SimRosNodeBase::SimRosNodeBase(const SimRosNodeOptions& opts,
                               const rclcpp::NodeOptions& node_options)
    : rclcpp::Node(opts.node_name, node_options),
      sim_joint_commands_(opts.control_joint_names.size()),
      sim_joint_commands_fresh_(opts.control_joint_names.size()),
      joint_states_layout_(opts.joint_states_layout),
      joint_enabled_requires_enable_control_(opts.joint_enabled_requires_enable_control),
      topic_prefix_(opts.topic_prefix),
      camera_frame_id_(opts.camera_frame_id),
      sim_joint_names_(opts.control_joint_names),
      driver_status_message_(opts.driver_status_message) {
  // 话题前缀必须形如 "/robot"：带前导 '/' 且不带尾随 '/'，否则拼出来的话题名
  // 会出现双斜杠或相对名，节点起得来但话题树会静默错位。
  if (topic_prefix_.empty() || topic_prefix_.front() != '/' || topic_prefix_.back() == '/') {
    throw std::invalid_argument("SimRosNodeOptions::topic_prefix 非法: " + topic_prefix_);
  }
  LoadParam();
  InitCommandCache();
  InitRosInterfaces();
}

void SimRosNodeBase::LoadParam() {
  // enable_control 只控制是否把命令写入 MuJoCo d->ctrl。
  // 它不是硬件使能开关，不会访问 EtherCAT/CAN/串口等真实硬件。
  // 仿真环境默认允许控制输入；如需只观察模型，可在 launch/CLI 中覆盖为 false。
  this->declare_parameter<bool>("enable_control", true);
  enable_control_ = this->get_parameter("enable_control").as_bool();
  // 发布频率等可调常量：默认值仅作兜底，实际值由各仿真包的 config YAML 传入。
  this->declare_parameter<int64_t>("driver_status_period_ms", 200);
  driver_status_period_ms_ = this->get_parameter("driver_status_period_ms").as_int();
  this->declare_parameter<int64_t>("camera_period_us", 0);
  camera_period_us_ = this->get_parameter("camera_period_us").as_int();
  RCLCPP_INFO(this->get_logger(), "Enable control: %d", enable_control_);
  RCLCPP_INFO(this->get_logger(), "MuJoCo joint count: %d",
              static_cast<int>(sim_joint_names_.size()));
}

void SimRosNodeBase::InitCommandCache() {
  // 启动时所有仿真关节默认不使能，避免节点刚启动就向 MuJoCo 执行器写入非零目标。
  for (auto& command : sim_joint_commands_fresh_) {
    command = SimJointCommand{};
  }
  sim_joint_commands_ = sim_joint_commands_fresh_;
  for (auto& command : sim_joint_commands_) {
    command.position = 0.0;
    command.velocity = 0.0;
    command.effort = 0.0;
    command.kp = 0.0;
    command.kd = 0.0;
    command.enable = false;
  }

  // 对外发布的 JointStates 消息大小由布局决定：
  // 人形包为 31（29 仿真关节 + 2 占位），狗形包为 12。
  std::size_t total_states = 0;
  for (const int msg_index : joint_states_layout_.sim_to_msg_index) {
    if (msg_index >= 0) {
      total_states = std::max(total_states, static_cast<std::size_t>(msg_index) + 1);
    }
  }
  for (const auto& placeholder : joint_states_layout_.placeholders) {
    total_states = std::max(total_states, placeholder.msg_index + 1);
  }
  joint_states_msg_.states.resize(total_states);

  for (std::size_t i = 0; i < sim_joint_names_.size(); ++i) {
    const int msg_index = joint_states_layout_.sim_to_msg_index[i];
    if (msg_index < 0) {
      continue;
    }
    joint_states_msg_.states[static_cast<std::size_t>(msg_index)].joint_name =
        sim_joint_names_[i];
  }
  for (const auto& placeholder : joint_states_layout_.placeholders) {
    joint_states_msg_.states[placeholder.msg_index].joint_name = placeholder.joint_name;
  }

  // 仿真驱动默认在线且无故障。state_manager 依赖 DriverStatus 判断系统是否可进入 READY。
  driver_status_msg_.status = robot_interfaces::msg::DriverStatus::STATUS_ONLINE;
  driver_status_msg_.status_message = driver_status_message_;
  driver_status_msg_.fault_active = false;
  driver_status_msg_.online = true;
  driver_status_msg_.total_joints = static_cast<uint32_t>(total_states);
}

void SimRosNodeBase::InitRosInterfaces() {
  auto qos = rclcpp::SensorDataQoS();

  sub_hardware_commands_ = this->create_subscription<robot_interfaces::msg::JointCommands>(
      topic_prefix_ + "/control/hardware_commands", qos,
      std::bind(&SimRosNodeBase::HandleHardwareCommands, this, std::placeholders::_1));
  pub_joint_states_ = this->create_publisher<robot_interfaces::msg::JointStates>(
      topic_prefix_ + "/driver/joint_states", qos);
  pub_driver_status_ = this->create_publisher<robot_interfaces::msg::DriverStatus>(
      topic_prefix_ + "/driver/status", 10);
  driver_status_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(driver_status_period_ms_),
      std::bind(&SimRosNodeBase::PublishDriverStatus, this));

  // 发布和真实 yesense 驱动兼容的 NavAll，使 robot_rl_manager 在仿真和实机
  // 两种路径下可以复用同一套 IMU 输入处理逻辑。
  imu_pub_ = this->create_publisher<yesense_interface::msg::NavAll>(
      topic_prefix_ + "/imu/nav_all", 10);

  if (camera_period_us_ > 0) {
    // 创建 RealSense 风格的话题发布器
    color_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        topic_prefix_ + "/camera/color/image_raw", 10);
    depth_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        topic_prefix_ + "/camera/depth/image_rect_raw", 10);
    color_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        topic_prefix_ + "/camera/color/camera_info", 10);
    depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        topic_prefix_ + "/camera/depth/camera_info", 10);
  }
}

void SimRosNodeBase::HandleHardwareCommands(const robot_interfaces::msg::JointCommands& msg) {
  // subscription callback 只入队，不做复杂计算，尽量减少 ROS executor 中的阻塞时间。
  if (cmd_queue_.write_available()) {
    cmd_queue_.push(msg);
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "sim command queue is full, drop the command");
  }
}

bool SimRosNodeBase::TryTakeLatestCommand(robot_interfaces::msg::JointCommands& out) {
  // 排空队列并保留最后入队的一帧。即使 push 侧偶发丢帧，消费侧也始终
  // 拿到最新命令，不会执行比最新意图更旧的缓存目标。
  bool took_any = false;
  robot_interfaces::msg::JointCommands tmp;
  while (cmd_queue_.pop(tmp)) {
    out = std::move(tmp);
    took_any = true;
  }
  return took_any;
}

bool SimRosNodeBase::PopCommand(robot_interfaces::msg::JointCommands& out) {
  // 单帧 FIFO 出队。狗形仿真控制器逐帧消费全部命令，保持与原有行为一致。
  return cmd_queue_.pop(out);
}

void SimRosNodeBase::UpdateCommandCache(const robot_interfaces::msg::JointCommands& msg,
                                        double time) {
  last_command_trace_id_ = msg.trace_id;
  last_command_sequence_ = msg.sequence;

  // 一帧命令可能不是全身关节，也可能包含当前 MuJoCo 模型没有的关节。
  // 这里按 joint_name 更新"最新参考"缓存，未出现的关节保持上一次缓存值。
  for (const auto& command_msg : msg.commands) {
    const int sim_index = GetSimJointIndex(command_msg.joint_name);
    if (sim_index < 0) {
      // 未知/未建模关节在仿真驱动侧忽略。关节白名单和拒绝逻辑由
      // robot_router/robot_control 负责，不在仿真侧重复仲裁。
      continue;
    }
    auto& command = sim_joint_commands_fresh_[static_cast<size_t>(sim_index)];
    command.position = static_cast<double>(command_msg.position);
    command.velocity = static_cast<double>(command_msg.velocity);
    command.effort = static_cast<double>(command_msg.effort);
    command.kp = static_cast<double>(command_msg.kp);
    command.kd = static_cast<double>(command_msg.kd);
    command.enable = command_msg.enable;
  }

  // errors.command_delay_ms：把整份"最新参考"按仿真时间延后 Δ，再交给 ComputeControl。
  // 真机上驱动器收到的是 Δ 之前的目标位置/速度/前馈与增益，但它用**自己的编码器**闭合
  // PD——所以这条延迟在参考路径上（回路外），只让跟踪滞后，不像 torque_delay_ms 那样
  // 直接消耗相位裕度。这也是实测"发布指令 → 力矩体现"那 25~40 ms 的正确落点。
  if (error_model_ != nullptr) {
    error_model_->ApplyCommandDelay(time, sim_joint_commands_fresh_, sim_joint_commands_);
  } else {
    sim_joint_commands_ = sim_joint_commands_fresh_;
  }
}

int SimRosNodeBase::GetSimJointIndex(const std::string& joint_name) const {
  // 关节数量较小，线性查找足够简单可靠；后续如增加更多模型可改为 unordered_map。
  const auto it = std::find(sim_joint_names_.begin(), sim_joint_names_.end(), joint_name);
  if (it == sim_joint_names_.end()) {
    return -1;
  }
  return static_cast<int>(std::distance(sim_joint_names_.begin(), it));
}

void SimRosNodeBase::PublishJointStates() {
  const auto now = this->get_clock()->now();
  joint_states_msg_.header.stamp = now;
  // sequence 由仿真驱动维护，方便 command_router/state_manager 检查状态流是否持续更新。
  joint_states_msg_.sequence = ++joint_state_sequence_;
  joint_states_msg_.last_command_trace_id = last_command_trace_id_;
  joint_states_msg_.last_command_sequence = last_command_sequence_;
  pub_joint_states_->publish(joint_states_msg_);
}

void SimRosNodeBase::PublishDriverStatus() {
  // active_joints 统计当前命令缓存中 enable=true 的仿真关节数量。
  // 这不是硬件在线数量，只用于仿真诊断和状态观察。
  uint32_t active_joints = 0;
  for (const auto& command : sim_joint_commands_) {
    if (command.enable) {
      ++active_joints;
    }
  }

  const auto now = this->get_clock()->now();
  driver_status_msg_.header.stamp = now;
  driver_status_msg_.online = true;
  driver_status_msg_.fault_active = false;
  driver_status_msg_.active_joints = active_joints;
  driver_status_msg_.faulted_joints = 0;
  driver_status_msg_.faulted_joint_names.clear();
  pub_driver_status_->publish(driver_status_msg_);
}

void SimRosNodeBase::PublishImu() {
  // 纯发布动作：imu_data_ 的物理量字段由 SimNodeBase 在发布前注入。
  imu_pub_->publish(imu_data_);
}

namespace
{

// 垂直翻转图像行，把 MuJoCo 自底向上的渲染输出转为 ROS 图像自顶向下的行序。
template <typename T>
std::vector<T> FlipVertically(const std::vector<T>& src, int width, int height, int channels) {
  std::vector<T> flipped(src.size());
  const std::size_t row_size = static_cast<std::size_t>(width) * channels;
  for (int y = 0; y < height; ++y) {
    const std::size_t src_row = static_cast<std::size_t>(height - 1 - y) * row_size;
    std::memcpy(&flipped[static_cast<std::size_t>(y) * row_size], &src[src_row],
                row_size * sizeof(T));
  }
  return flipped;
}

}  // namespace

void SimRosNodeBase::PublishImages(const std::vector<unsigned char>& rgb,
                                   const std::vector<float>& depth, int width, int height) {
  // 把渲染层输出的原始像素数据序列化成 RealSense 风格的话题消息并发布。
  rclcpp::Time now = this->get_clock()->now();

  const auto rgb_flipped = FlipVertically(rgb, width, height, 3);
  const auto depth_flipped = FlipVertically(depth, width, height, 1);

  // ---- 发布 RGB 图像 ----
  auto rgb_msg = std::make_unique<sensor_msgs::msg::Image>();
  rgb_msg->header.stamp = now;
  rgb_msg->header.frame_id = camera_frame_id_;
  rgb_msg->width = width;
  rgb_msg->height = height;
  rgb_msg->encoding = sensor_msgs::image_encodings::RGB8;  // RealSense 默认 RGB8
  rgb_msg->step = width * 3;
  rgb_msg->data = rgb_flipped;  // 直接复制数据
  color_image_pub_->publish(std::move(rgb_msg));

  // ---- 发布深度图像 ----
  auto depth_msg = std::make_unique<sensor_msgs::msg::Image>();
  depth_msg->header.stamp = now;
  depth_msg->header.frame_id = camera_frame_id_;
  depth_msg->width = width;
  depth_msg->height = height;
  depth_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;  // 32位浮点数，单位米
  depth_msg->step = width * sizeof(float);
  // 将 float 数组转换为字节流
  depth_msg->data.resize(depth_flipped.size() * sizeof(float));
  std::memcpy(depth_msg->data.data(), depth_flipped.data(), depth_msg->data.size());
  depth_image_pub_->publish(std::move(depth_msg));

  // ---- 发布对应的 camera_info ----
  if (camera_info_ready_) {
    camera_info_msg_.header.stamp = now;
    color_info_pub_->publish(camera_info_msg_);
    depth_info_pub_->publish(camera_info_msg_);
  }
}

}  // namespace sim_common
