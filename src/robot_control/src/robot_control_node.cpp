#include "robot_control/robot_control_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>

#include "robot_common/ankle_transform.hpp"
#include "rclcpp/qos.hpp"

namespace robot_control
{
RobotControlNode::RobotControlNode(const rclcpp::NodeOptions & options)
: Node("robot_control_node", options)
{
  // Filter 参数负责关节配置、有限数检查和物理限幅。
  const auto robot_joints_config = declare_parameter<std::string>("robot_joints_config", "");

  const auto control_frequency_hz = declare_parameter<double>("control_frequency_hz", 200.0);
  enable_rejected_command_topic_ = declare_parameter<bool>("enable_rejected_command_topic", true);
  prepare_total_cycles_ = declare_parameter<int>("prepare_interpolation_cycles", 1000);
  enable_mujoco_sim_ = declare_parameter<bool>("enable_mujoco_sim", true);
  robot_type_ = declare_parameter<std::string>("robot_type", "humanoid");
  // robot_type 只表达形态：人形有踝关节转换，狗跳过。
  has_ankle_ = (robot_type_ == "humanoid");
  // 环境侧踝关节空间：真机人形固定为并联；仿真由 launch 的 ankle_env_parallel 传入。
  ankle_env_parallel_ = declare_parameter<bool>("ankle_env_parallel", has_ankle_ && !enable_mujoco_sim_);
  // 踝关节命令转换模式（等效增益 / 纯力矩前馈 / 混合前馈）由转换器自行读取参数。
  ankle_converter_.Configure(*this);

  // 初始化控制指令过滤器 filter。
  filter_ = std::make_unique<JointCommandFilter>(robot_joints_config);

  // 只订阅 command_router 已接受的关节命令，避免外部输入绕过统一路由。
  accepted_commands_sub_ = create_subscription<robot_interfaces::msg::JointCommands>(
    "router/accepted_commands",
    rclcpp::SensorDataQoS(),
    std::bind(&RobotControlNode::handleAcceptedCommands, this, std::placeholders::_1));

  // 订阅 driver 发布的原始关节状态，进行空间转换后使用。
  joint_states_sub_ = create_subscription<robot_interfaces::msg::JointStates>(
    "driver/joint_states",
    rclcpp::SensorDataQoS(),
    std::bind(&RobotControlNode::handleJointStates, this, std::placeholders::_1));

  // RL manager 状态：缓存 ankle_in_series 供状态路径 P2S 决策。
  rl_status_sub_ = create_subscription<robot_interfaces::msg::RlManagerStatus>(
    "rl_manager/status",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&RobotControlNode::handleRlManagerStatus, this, std::placeholders::_1));

  // 系统状态用于 FAULT 全局阻断。
  state_sub_ = create_subscription<robot_interfaces::msg::RobotState>(
    "state",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&RobotControlNode::handleSystemState, this, std::placeholders::_1));

  // 统一发布给 driver 的硬件命令（并联空间）。
  hardware_commands_pub_ = create_publisher<robot_interfaces::msg::JointCommands>(
    "control/hardware_commands", rclcpp::SensorDataQoS());

  // 发布经空间转换后的硬件状态（匹配模型期望空间），供上游使用。
  hardware_states_pub_ = create_publisher<robot_interfaces::msg::JointStates>(
    "control/hardware_states", rclcpp::SensorDataQoS());

  // 准备动作插值进度（0.0 ~ 1.0），供 state_manager 判断 DO_READY→READY 切换时机。
  preparation_progress_pub_ = create_publisher<std_msgs::msg::Float32>(
    "control/preparation_progress", rclcpp::SystemDefaultsQoS());

  // 被 control 层拒绝的命令单独发布，便于调试准入失败原因。
  rejected_commands_pub_ = create_publisher<robot_interfaces::msg::JointCommands>(
    "control/rejected_commands", rclcpp::SensorDataQoS());

  // 控制周期 timer；频率最低钳制到 1Hz，避免除零或无效周期。
  const auto period = std::chrono::duration<double>(1.0 / std::max(control_frequency_hz, 1.0));
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&RobotControlNode::handleControlTimer, this));
}

// 判断系统状态是否阻断控制命令
bool RobotControlNode::systemBlocksCommand(
    const robot_interfaces::msg::RobotState & state,
    const robot_interfaces::msg::JointCommands & commands)
{
  const bool blocked = state.state == robot_interfaces::msg::RobotState::STATE_FAULT || state.fault_active;
  if (!blocked) {
    return false;
  }
  return commands.type != robot_interfaces::msg::JointCommands::TYPE_DAMPING;
}

// 处理 RL manager 状态
void RobotControlNode::handleRlManagerStatus(
  const robot_interfaces::msg::RlManagerStatus::SharedPtr msg)
{
  // 缓存当前模型踝关节空间信息，供状态路径 P2S 决策。
  ankle_in_series_ = msg->ankle_in_series;
  have_rl_status_ = true;
}

// 处理已接受的控制命令
void RobotControlNode::handleAcceptedCommands(
  const robot_interfaces::msg::JointCommands::SharedPtr msg)
{
  // 命令帧前哨纠正：模型切换时命令帧先于 status 到达，立即更新状态路径缓存。
  last_cmd_ankle_in_series_ = msg->ankle_in_series;
  if (last_cmd_ankle_in_series_ != ankle_in_series_) {
    RCLCPP_INFO(
      get_logger(), "ankle_in_series fast-forward: %s -> %s (via command frame)",
      ankle_in_series_ ? "true" : "false", last_cmd_ankle_in_series_ ? "true" : "false");
    ankle_in_series_ = last_cmd_ankle_in_series_;
  }

  // FAULT 等系统阻断状态下，不再对新运动命令做限幅后下发，而是直接拒绝。
  if (systemBlocksCommand(latest_system_state_, *msg)) {
    publishRejected(*msg);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected joint commands: system state blocks motion");
    return;
  }

  // TYPE_PREPARE 命令：跳过普通 filter 路径，启动多周期插值。
  // 正在插值中收到新的 TYPE_PREPARE 说明用户切换了准备姿态，应重启插值。
  if (msg->type == robot_interfaces::msg::JointCommands::TYPE_PREPARE) {
    if (preparing_) {
      if (samePrepareTarget(*msg)) {
        return;
      }
      RCLCPP_INFO(get_logger(), "Restarting preparation interpolation with new target");
    }
    startPrepareInterpolation(*msg);
    return;
  }

  // 插值进行中时，丢弃所有非 PREPARE 帧，避免 TYPE_RUN 帧完整位姿覆盖插值中间帧。
  if (preparing_) {
    return;
  }

  // DO_READY 必须先由 TYPE_PREPARE 启动插值。若一次性 PREPARE 帧漏掉，
  // 后续 TYPE_RUN 不能作为普通命令直接下发，否则会瞬间跳到准备目标位。
  if (have_system_state_ &&
    latest_system_state_.state == robot_interfaces::msg::RobotState::STATE_DO_READY &&
    msg->type != robot_interfaces::msg::JointCommands::TYPE_PREPARE)
  {
    publishRejected(*msg);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected joint commands: waiting for TYPE_PREPARE in DO_READY");
    return;
  }

  // 第一阶段：物理边界、白名单、增益和 enable_default 过滤。
  const auto filter_result = filter_->filter(*msg);
  if (!filter_result.accepted) {
    publishRejected(*msg);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected joint commands: %s", filter_result.reason.c_str());
    return;
  }

  // 通过基础安全过滤后直接发布给 driver；命令平滑由上游模型/运控策略负责。
  publishHardwareCommands(filter_result.commands, "control_output");
}

// 处理关节反馈，缓存转换后的状态并转发给上游。
void RobotControlNode::handleJointStates(
  const robot_interfaces::msg::JointStates::SharedPtr msg)
{
  // 先缓存环境空间原始反馈（下面的转换会原地改写 msg）。
  ankle_converter_.CacheEnvStates(*msg);
  
  latest_raw_joint_states_ = *msg;
  have_raw_joint_states_ = true;

  auto converted = *msg;

  // 踝关节串并联转换（仅人形机器人，狗没有踝关节）。
  if (has_ankle_) {
    // 环境空间由 ankle_env_parallel_ 决定（true=并联，false=串联）。
    // 模型空间由 ankle_in_series_ 决定（true=串联，false=并联）。
    if (!ankle_env_parallel_ && !ankle_in_series_) {
      // 环境串联 → 模型并联：S2P
      robot_common::ankle_transform::convertToParallel(converted.states);
    } else if (ankle_env_parallel_ && ankle_in_series_) {
      // 环境并联 → 模型串联：P2S
      robot_common::ankle_transform::convertToSeries(converted.states);
    }
  }

  // 缓存转换后的状态（当前模型空间）。
  latest_joint_states_ = converted;
  have_joint_states_ = true;
  ankle_converter_.CacheModelStates(latest_joint_states_);

  // 发布转换后的状态（当前模型空间）。
  hardware_states_pub_->publish(converted);

  for (const auto & state : latest_joint_states_.states) {
    if (state.joint_error == robot_interfaces::msg::JointState::JOINT_ERR_MOTOR_WARN) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Joint motor warning: %s motor_error=%u message=%s",
        state.joint_name.c_str(),
        static_cast<unsigned>(state.motor_error),
        state.error_message.c_str());
    }
  }
}

// 处理系统状态。
void RobotControlNode::handleSystemState(
  const robot_interfaces::msg::RobotState::SharedPtr msg)
{
  // 退出 DO_READY 时取消准备插值，否则 preparing_ 会持续拦截后续所有非 PREPARE 帧。
  if (preparing_ && msg->state != robot_interfaces::msg::RobotState::STATE_DO_READY) {
    preparing_ = false;
  }
  // state_manager 是系统状态权威，control 只缓存并用于安全阻断判断。
  latest_system_state_ = *msg;
  have_system_state_ = true;
}

// 控制循环，处理准备动作插值。
void RobotControlNode::handleControlTimer()
{
  if (preparing_) {
    advancePrepareInterpolation();
    return;
  }
}

// 发布硬件命令（经空间转换后，适配目标环境）
void RobotControlNode::publishHardwareCommands(
  const robot_interfaces::msg::JointCommands & cmd,
  const std::string & stage)
{
  // 踝关节串并联转换（仅人形机器人，狗没有踝关节）。
  auto s2p_cmd = cmd;
  if (has_ankle_) {
    // 模型空间由 cmd.ankle_in_series 决定（true=串联，false=并联）。
    // 环境空间由 ankle_env_parallel_ 决定（true=并联，false=串联）。
    if (cmd.ankle_in_series && ankle_env_parallel_) {
      // 模型串联 → 环境并联：S2P
      // robot_common::ankle_transform::convertToParallel(s2p_cmd.commands);
	  // 模型串联 → 环境并联：S2P（等效增益 / 力矩前馈 / 混合前馈由转换器按参数分派）
      ankle_converter_.Convert(s2p_cmd.commands);
    } else if (!cmd.ankle_in_series && !ankle_env_parallel_) {
      // 模型并联 → 环境串联：P2S
      robot_common::ankle_transform::convertToSeries(s2p_cmd.commands);
    }
  }

  auto frame = makeHardwareCommandFrame(s2p_cmd, stage);
  frame.header.stamp = now();
  hardware_commands_pub_->publish(frame);
}

// 发布拒绝命令。
void RobotControlNode::publishRejected(
  const robot_interfaces::msg::JointCommands & commands)
{
  // rejected topic 可通过参数关闭，避免高频拒绝时产生过多诊断流量。
  if (enable_rejected_command_topic_) {
    rejected_commands_pub_->publish(commands);
  }
}

// 开始准备动作插值。
void RobotControlNode::startPrepareInterpolation(
  const robot_interfaces::msg::JointCommands & commands)
{
  // 保存目标帧，并记录当前关节反馈位置作为插值起点。
  prepare_target_ = commands;
  prepare_cycle_ = 0;

  auto start_states = have_raw_joint_states_ ? latest_raw_joint_states_ : latest_joint_states_;
  if (has_ankle_ && have_raw_joint_states_) {
    // PREPARE 插值起点必须与本次命令目标处于同一踝空间，避免启动早期缓存空间滞后。
    if (!ankle_env_parallel_ && !commands.ankle_in_series) {
      robot_common::ankle_transform::convertToParallel(start_states.states);
    } else if (ankle_env_parallel_ && commands.ankle_in_series) {
      robot_common::ankle_transform::convertToSeries(start_states.states);
    }
  }

  prepare_start_positions_.resize(commands.commands.size());
  for (std::size_t i = 0; i < commands.commands.size(); ++i) {
    const auto * state = findJointStateIn(start_states, commands.commands[i].joint_name);
    prepare_start_positions_[i] = (state != nullptr) ? state->position : 0.0f;
  }
  preparing_ = true;
  RCLCPP_INFO(get_logger(), "Started preparation interpolation: %zu joints, %d cycles", commands.commands.size(), prepare_total_cycles_);
}

// 进行准备动作插值。
void RobotControlNode::advancePrepareInterpolation()
{
  if (!preparing_ || !have_joint_states_) {
    return;
  }

  prepare_cycle_++;
  const double t = std::min(static_cast<double>(prepare_cycle_) / prepare_total_cycles_, 1.0);

  // 从固定的起始位置向目标做匀速线性插值。
  auto frame = prepare_target_;
  for (std::size_t i = 0; i < frame.commands.size(); ++i) {
    if (i < prepare_start_positions_.size()) {
      auto & cmd = frame.commands[i];
      const float start_pos = prepare_start_positions_[i];
      const float target_pos = cmd.position;
      cmd.position = start_pos + static_cast<float>(t) * (target_pos - start_pos);
    }
  }

  publishHardwareCommands(frame, "prepare_interp");

  // 发布准备进度。
  std_msgs::msg::Float32 progress_msg;
  progress_msg.data = static_cast<float>(t);
  preparation_progress_pub_->publish(progress_msg);

  if (t >= 1.0) {
    preparing_ = false;
    RCLCPP_INFO(get_logger(), "Preparation interpolation complete");
  }
}

// 查找指定关节的当前状态。
const robot_interfaces::msg::JointState * RobotControlNode::findJointState(
  const std::string & joint_name) const
{
  return findJointStateIn(latest_joint_states_, joint_name);
}

const robot_interfaces::msg::JointState * RobotControlNode::findJointStateIn(
  const robot_interfaces::msg::JointStates & states,
  const std::string & joint_name) const
{
  if (!have_joint_states_) {
    return nullptr;
  }

  for (const auto & s : states.states) {
    if (s.joint_name == joint_name) {
      return &s;
    }
  }
  return nullptr;
}

// 检查当前准备目标是否与指定命令相同。
bool RobotControlNode::samePrepareTarget(
  const robot_interfaces::msg::JointCommands & commands) const
{
  if (!preparing_ || commands.commands.size() != prepare_target_.commands.size()) {
    return false;
  }

  for (std::size_t i = 0; i < commands.commands.size(); ++i) {
    const auto & lhs = commands.commands[i];
    const auto & rhs = prepare_target_.commands[i];
    if (lhs.joint_name != rhs.joint_name ||
      lhs.position != rhs.position ||
      lhs.velocity != rhs.velocity ||
      lhs.effort != rhs.effort ||
      lhs.kp != rhs.kp ||
      lhs.kd != rhs.kd ||
      lhs.enable != rhs.enable)
    {
      return false;
    }
  }
  return true;
}

// 构造 hardware command 输出帧。
robot_interfaces::msg::JointCommands RobotControlNode::makeHardwareCommandFrame(
  const robot_interfaces::msg::JointCommands & input,
  const std::string & stage)
{
  auto output = input;
  output.parent_sequence = input.sequence;
  output.parent_source = input.source;
  output.sequence = ++hardware_command_sequence_;
  output.source = "robot_control";
  output.stage = stage;
  return output;
}

}  // namespace robot_control
