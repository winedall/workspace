#include "robot_router/robot_router_node.hpp"

#include <memory>
#include <unordered_set>

#include "rclcpp/qos.hpp"

namespace robot_router
{

RobotRouterNode::RobotRouterNode(const rclcpp::NodeOptions & options)
: Node("robot_router_node", options)
{
  // 所有可调阈值都通过 ROS 参数声明，launch 或命令行可以覆盖默认值。
  RouterConfig config;
  config.robot_joints_config = declare_parameter<std::string>("robot_joints_config", "");
  config.teleop_priority = declare_parameter<int>("teleop_priority", 0);
  config.app_priority_min = declare_parameter<int>("app_priority_min", 1);
  config.app_priority_max = declare_parameter<int>("app_priority_max", 49);
  config.api_priority_min = declare_parameter<int>("api_priority_min", 50);
  config.api_priority_max = declare_parameter<int>("api_priority_max", 99);
  config.unknown_priority = declare_parameter<int>("unknown_priority", 255);
  
  // 创建命令路由器实例，根据配置初始化。
  router_ = std::make_unique<CommandRouter>(config);

  // 高层意图入口。这里只做准入和路由，不在回调中直接生成 JointCommands。
  intent_sub_ = create_subscription<robot_interfaces::msg::RobotIntent>(
    "input/intent",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&RobotRouterNode::handleIntent, this, std::placeholders::_1));

  // RL 管理器输出的候选关节命令。先合并外部指令，再准入。
  rl_commands_sub_ = create_subscription<robot_interfaces::msg::JointCommands>(
    "rl_manager/desired_commands",
    rclcpp::SensorDataQoS(),
    std::bind(&RobotRouterNode::handleRlJointCommands, this, std::placeholders::_1));

  // 外部控制指令入口，缓存供合并使用。
  // 使用 SensorDataQoS（BEST_EFFORT + KeepLast(5)）以兼容各种外部来源（HTTP API、WebSocket 等）。
  external_commands_sub_ = create_subscription<robot_interfaces::msg::JointCommands>(
    "external/desired_commands",
    rclcpp::SensorDataQoS(),
    std::bind(&RobotRouterNode::handleExternalCommands, this, std::placeholders::_1));

  // 全量硬件状态，用于过滤外部控制关节的反馈。
  hardware_states_sub_ = create_subscription<robot_interfaces::msg::JointStates>(
    "control/hardware_states",
    rclcpp::SensorDataQoS(),
    std::bind(&RobotRouterNode::handleHardwareStates, this, std::placeholders::_1));

  // 状态管理器和驱动状态提供安全上下文，例如 FAULT 和硬件使能状态。
  state_sub_ = create_subscription<robot_interfaces::msg::RobotState>(
    "state",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&RobotRouterNode::handleSystemState, this, std::placeholders::_1));

  // 处理驱动状态消息，更新缓存。
  driver_status_sub_ = create_subscription<robot_interfaces::msg::DriverStatus>(
    "driver/status",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&RobotRouterNode::handleDriverStatus, this, std::placeholders::_1));

  // 已接受的意图与关节命令分别发布，方便下游节点只订阅经过准入的 topic。
  active_intent_pub_ = create_publisher<robot_interfaces::msg::RobotIntent>(
    "router/active_intent", rclcpp::SystemDefaultsQoS());

  // 被拒绝的 intent 单独发布，便于遥控/API 层感知拒绝，而不是静默丢弃。
  rejected_intent_pub_ = create_publisher<robot_interfaces::msg::RobotIntent>(
    "router/rejected_intent", rclcpp::SystemDefaultsQoS());

  // 处理已接受的关节命令消息，更新缓存。
  accepted_commands_pub_ = create_publisher<robot_interfaces::msg::JointCommands>(
    "router/accepted_commands", rclcpp::SensorDataQoS());  

  // 外部控制关节的硬件状态反馈，与外部指令 topic 使用相同 QoS 策略。
  external_states_pub_ = create_publisher<robot_interfaces::msg::JointStates>(
    "external/hardware_states", rclcpp::SensorDataQoS());
}

// 处理意图消息，根据准入判断是否发布。
void RobotRouterNode::handleIntent(const robot_interfaces::msg::RobotIntent::SharedPtr msg)
{
  const auto decision = router_->evaluateIntent(*msg);
  if (decision.accepted) {
    // 检测模式切换意图：intent_type=config, command=mode, action=rl/mix/app
    if (msg->intent_type == "config" && msg->command == "mode") {
      const bool into_app = (msg->action == "app");
      // 任何 mode 切换都先退出 app 模式；仅切到 app 时再开启。
      router_->setAppMode(false);
      if (into_app) {
        router_->setAppMode(true);
        // 清空之前遥控器留下的 active_intent，让 APP 能够接管控制权。
        router_->clearActiveIntent();
        RCLCPP_INFO(get_logger(), "switched to APP mode");
      } else if (msg->action == "rl") {
        router_->setControlMode(ControlMode::RL);
        RCLCPP_INFO(get_logger(), "switched to RL mode");
      } else if (msg->action == "mix") {
        router_->setControlMode(ControlMode::MIX);
        RCLCPP_INFO(get_logger(), "switched to MIX mode");
      } else {
        RCLCPP_INFO(get_logger(), "switched to mode %s", msg->action.c_str());
      }
    }
    active_intent_pub_->publish(*msg);
  } else {
    // app 模式下遥控器的 motion 是高频连续输入，被门 D 拦截属于预期，
    // 参考导航模式静默过滤，不发布 rejected_intent、不打日志。
    if (msg->source == "teleop" && msg->intent_type == "motion" && router_->appMode()) {
      return;
    }
    // 拒绝的 intent 保留原消息发布（下游依赖 rejected_intent 感知拒绝，必须每条都发）；
    // 日志改为限频 WARN，避免 FAULT 等持续拒绝场景下每条意图都打日志刷屏。
    rejected_intent_pub_->publish(*msg);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Rejected intent source=%s type=%s command=%s action=%s: %s",
      msg->source.c_str(), msg->intent_type.c_str(), msg->command.c_str(),
      msg->action.c_str(), decision.reason.c_str());
  }
}

// 处理 RL 管理器输出的关节命令：先合并外部指令，再准入。
void RobotRouterNode::handleRlJointCommands(const robot_interfaces::msg::JointCommands::SharedPtr msg)
{
  // 第一步：合并 RL 和外部指令。
  const auto merged = router_->merge(*msg);

  // 第二步：对合并后的指令做准入检查。
  const auto decision = router_->evaluateJointCommands(merged);
  if (decision.accepted) {
    auto accepted = router_->makeAcceptedJointCommands(merged);
    accepted.header.stamp = now();
    accepted_commands_pub_->publish(accepted);
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected joint commands: %s", decision.reason.c_str());
  }
}

// 缓存外部控制指令，供 merge 时使用。
void RobotRouterNode::handleExternalCommands(const robot_interfaces::msg::JointCommands::SharedPtr msg)
{
  router_->cacheExternalCommands(*msg);
}

// 从全量硬件状态中过滤出外部控制关节并转发。
void RobotRouterNode::handleHardwareStates(const robot_interfaces::msg::JointStates::SharedPtr msg)
{
  const auto external_joints = router_->externalJointNames();
  if (external_joints.empty()) {
    return;
  }

  auto external_states = robot_interfaces::msg::JointStates();
  external_states.header = msg->header;
  external_states.sequence = msg->sequence;
  for (const auto & state : msg->states) {
    if (external_joints.count(state.joint_name) > 0) {
      external_states.states.push_back(state);
    }
  }
  if (!external_states.states.empty()) {
    external_states_pub_->publish(external_states);
  }
}

// 处理系统状态消息，更新缓存。
void RobotRouterNode::handleSystemState(const robot_interfaces::msg::RobotState::SharedPtr msg)
{
  // 状态回调只更新缓存，避免在状态链路中直接触发控制输出。
  router_->updateSystemState(*msg);
}

// 处理驱动状态消息，更新缓存。
void RobotRouterNode::handleDriverStatus(const robot_interfaces::msg::DriverStatus::SharedPtr msg)
{
  // driver 状态包含硬件使能、故障等信息，是准入判断的重要安全输入。
  router_->updateDriverStatus(*msg);
}

}  // namespace robot_router
