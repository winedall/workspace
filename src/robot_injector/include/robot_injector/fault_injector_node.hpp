#ifndef ROBOT_INJECTOR__FAULT_INJECTOR_NODE_HPP_
#define ROBOT_INJECTOR__FAULT_INJECTOR_NODE_HPP_

#include <cstddef>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/driver_status.hpp"
#include "robot_interfaces/msg/joint_states.hpp"
#include "robot_interfaces/msg/robot_state.hpp"

namespace robot_injector
{

/// 面向 robot_recorder 的简单事件注入节点。
///
/// 支持三种模式：
/// - system_fault: 发布 /state 系统故障
/// - joint_warning: 发布 control/hardware_states 关节告警
/// - both: 按 order 发布系统故障和关节告警
class FaultInjectorNode : public rclcpp::Node
{
public:
  explicit FaultInjectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /// 定时发布一次或多次注入消息。
  void publishTick();

  /// 发布 recorder 可识别的系统故障事件。
  void publishSystemFault();

  /// 发布 recorder 可识别的关节 warning 事件。
  void publishJointWarning();

  /// 发布 driver/status，仅用于给 recorder 的系统故障原因补充 driver 文本。
  void publishDriverStatus();

  std::string mode_;
  std::string order_;
  std::string joint_name_;
  int motor_error_{2};
  double delay_sec_{0.0};
  double period_ms_{100.0};
  int count_{3};
  std::size_t published_{0};

  rclcpp::Publisher<robot_interfaces::msg::RobotState>::SharedPtr state_pub_;
  rclcpp::Publisher<robot_interfaces::msg::JointStates>::SharedPtr joints_pub_;
  rclcpp::Publisher<robot_interfaces::msg::DriverStatus>::SharedPtr driver_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_injector

#endif  // ROBOT_INJECTOR__FAULT_INJECTOR_NODE_HPP_
