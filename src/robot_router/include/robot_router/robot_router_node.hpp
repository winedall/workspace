#ifndef ROBOT_COMMAND_ROUTER__ROBOT_COMMAND_ROUTER_NODE_HPP_
#define ROBOT_COMMAND_ROUTER__ROBOT_COMMAND_ROUTER_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_router/command_router.hpp"
#include "robot_interfaces/msg/driver_status.hpp"
#include "robot_interfaces/msg/joint_commands.hpp"
#include "robot_interfaces/msg/joint_states.hpp"
#include "robot_interfaces/msg/robot_intent.hpp"
#include "robot_interfaces/msg/robot_state.hpp"

namespace robot_router
{

// ROS 2 节点封装层：负责参数、topic 订阅发布和日志。
// 具体准入规则集中在 CommandRouter，便于单元测试和后续复用。
class RobotRouterNode : public rclcpp::Node
{
public:
  explicit RobotRouterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 上层意图入口，例如 teleop、HTTP/API、云端等来源。
  void handleIntent(const robot_interfaces::msg::RobotIntent::SharedPtr msg);

  // RL 管理器输出的候选关节命令入口，先合并外部指令再准入。
  void handleRlJointCommands(const robot_interfaces::msg::JointCommands::SharedPtr msg);

  // 外部控制指令入口，缓存后供合并使用。
  void handleExternalCommands(const robot_interfaces::msg::JointCommands::SharedPtr msg);

  // 系统状态和驱动状态只更新路由器缓存，不在这里直接产生控制输出。
  void handleSystemState(const robot_interfaces::msg::RobotState::SharedPtr msg);
  void handleDriverStatus(const robot_interfaces::msg::DriverStatus::SharedPtr msg);

  // 硬件状态入口，过滤出外部控制关节并转发。
  void handleHardwareStates(const robot_interfaces::msg::JointStates::SharedPtr msg);

  std::unique_ptr<CommandRouter> router_;

  // 输入订阅：一个意图入口、RL 命令入口、外部命令入口、硬件状态入口，以及两个安全状态入口。
  rclcpp::Subscription<robot_interfaces::msg::RobotIntent>::SharedPtr intent_sub_;
  rclcpp::Subscription<robot_interfaces::msg::JointCommands>::SharedPtr rl_commands_sub_;
  rclcpp::Subscription<robot_interfaces::msg::JointCommands>::SharedPtr external_commands_sub_;
  rclcpp::Subscription<robot_interfaces::msg::JointStates>::SharedPtr hardware_states_sub_;
  rclcpp::Subscription<robot_interfaces::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::Subscription<robot_interfaces::msg::DriverStatus>::SharedPtr driver_status_sub_;

  // 输出发布：被接受的意图/关节命令进入后续控制链路，被拒绝的意图单独发布便于诊断。
  rclcpp::Publisher<robot_interfaces::msg::RobotIntent>::SharedPtr active_intent_pub_;
  rclcpp::Publisher<robot_interfaces::msg::JointCommands>::SharedPtr accepted_commands_pub_;
  rclcpp::Publisher<robot_interfaces::msg::RobotIntent>::SharedPtr rejected_intent_pub_;
  // 外部控制关节的硬件状态反馈。
  rclcpp::Publisher<robot_interfaces::msg::JointStates>::SharedPtr external_states_pub_;
};

}  // namespace robot_router

#endif  // ROBOT_COMMAND_ROUTER__ROBOT_COMMAND_ROUTER_NODE_HPP_
