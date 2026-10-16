#include "robot_injector/fault_injector_node.hpp"

#include <algorithm>
#include <chrono>

#include "rclcpp/qos.hpp"

namespace robot_injector
{

namespace
{

bool validMode(const std::string & mode)
{
  return mode == "system_fault" || mode == "joint_warning" || mode == "both";
}

bool warningFirst(const std::string & order)
{
  return order != "fault_first";
}

}  // namespace

FaultInjectorNode::FaultInjectorNode(const rclcpp::NodeOptions & options)
: Node("fault_injector_node", options)
{
  mode_ = declare_parameter<std::string>("mode", "");
  order_ = declare_parameter<std::string>("order", "warning_first");
  joint_name_ = declare_parameter<std::string>("joint_name", "waist_yaw");
  motor_error_ = declare_parameter<int>("motor_error", 2);
  delay_sec_ = declare_parameter<double>("delay_sec", 0.0);
  period_ms_ = declare_parameter<double>("period_ms", 100.0);
  count_ = declare_parameter<int>("count", 3);

  state_pub_ = create_publisher<robot_interfaces::msg::RobotState>(
    "state", rclcpp::SystemDefaultsQoS());
  joints_pub_ = create_publisher<robot_interfaces::msg::JointStates>(
    "control/hardware_states", rclcpp::SensorDataQoS());
  driver_pub_ = create_publisher<robot_interfaces::msg::DriverStatus>(
    "driver/status", rclcpp::SystemDefaultsQoS());

  if (!validMode(mode_)) {
    RCLCPP_WARN(
      get_logger(),
      "No injection started. Set mode to system_fault, joint_warning, or both.");
    return;
  }

  const auto delay = std::chrono::duration<double>(std::max(0.0, delay_sec_));
  const auto period = std::chrono::duration<double>(std::max(1.0, period_ms_) / 1000.0);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(delay + period),
    std::bind(&FaultInjectorNode::publishTick, this));

  RCLCPP_INFO(
    get_logger(),
    "injector ready: mode=%s order=%s joint=%s count=%d period=%.0fms",
    mode_.c_str(), order_.c_str(), joint_name_.c_str(), count_, period_ms_);
}

void FaultInjectorNode::publishTick()
{
  if (count_ > 0 && published_ >= static_cast<std::size_t>(count_)) {
    RCLCPP_INFO(get_logger(), "injection complete: published %zu ticks", published_);
    timer_->cancel();
    return;
  }

  if (mode_ == "system_fault") {
    publishSystemFault();
  } else if (mode_ == "joint_warning") {
    publishJointWarning();
  } else if (warningFirst(order_)) {
    publishJointWarning();
    publishSystemFault();
  } else {
    publishSystemFault();
    publishJointWarning();
  }

  ++published_;
}

void FaultInjectorNode::publishSystemFault()
{
  publishDriverStatus();

  robot_interfaces::msg::RobotState msg;
  msg.header.stamp = now();
  msg.header.frame_id = "base_link";
  msg.state = robot_interfaces::msg::RobotState::STATE_FAULT;
  msg.state_name = "fault";
  msg.fault_active = true;
  msg.fault_reason = "injected system fault";
  state_pub_->publish(msg);
}

void FaultInjectorNode::publishJointWarning()
{
  robot_interfaces::msg::JointStates msg;
  msg.header.stamp = now();
  msg.header.frame_id = "base_link";
  msg.sequence = static_cast<std::uint32_t>(published_ + 1);
  msg.last_command_sequence = static_cast<std::uint32_t>(published_);

  robot_interfaces::msg::JointState joint;
  joint.joint_name = joint_name_;
  joint.enabled = true;
  joint.position = 0.5f;
  joint.joint_error = robot_interfaces::msg::JointState::JOINT_ERR_MOTOR_WARN;
  joint.motor_error = static_cast<std::uint8_t>(std::clamp(motor_error_, 0, 255));
  joint.error_message = "injected joint warning";
  msg.states.push_back(joint);

  joints_pub_->publish(msg);
}

void FaultInjectorNode::publishDriverStatus()
{
  robot_interfaces::msg::DriverStatus msg;
  msg.header.stamp = now();
  msg.header.frame_id = "base_link";
  msg.online = true;
  msg.fault_active = true;
  msg.status = robot_interfaces::msg::DriverStatus::STATUS_FAULT;
  msg.status_message = "injected driver fault";
  msg.faulted_joints = 1;
  driver_pub_->publish(msg);
}

}  // namespace robot_injector
