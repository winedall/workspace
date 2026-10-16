#include "robot_recorder/event_manager.hpp"

#include <gtest/gtest.h>

namespace robot_recorder
{
namespace
{

robot_interfaces::msg::JointStates makeWarningStates()
{
  robot_interfaces::msg::JointStates msg;
  msg.sequence = 11;
  msg.last_command_sequence = 10;

  robot_interfaces::msg::JointState joint;
  joint.joint_name = "joint_a";
  joint.joint_error = robot_interfaces::msg::JointState::JOINT_ERR_MOTOR_WARN;
  joint.motor_error = 7;
  joint.error_message = "motor warning";
  msg.states.push_back(joint);
  return msg;
}

robot_interfaces::msg::RobotState makeFaultState()
{
  robot_interfaces::msg::RobotState msg;
  msg.state = robot_interfaces::msg::RobotState::STATE_FAULT;
  msg.state_name = "fault";
  msg.fault_active = true;
  msg.fault_reason = "state fault";
  return msg;
}

TEST(EventManagerTest, KeepsFaultAfterPendingJointWarning)
{
  EventManager manager(2.0);

  EXPECT_TRUE(manager.onJointStates(makeWarningStates(), 1000));
  EXPECT_TRUE(manager.onRobotState(makeFaultState(), 1001, 11, 10, "driver fault"));

  ASSERT_TRUE(manager.hasPendingEvent());
  auto first = manager.takePendingEvent();
  EXPECT_EQ(first.event_type, "joint_warning");
  EXPECT_EQ(first.joint_name, "joint_a");

  ASSERT_TRUE(manager.hasPendingEvent());
  auto second = manager.takePendingEvent();
  EXPECT_EQ(second.event_type, "system_fault");
  EXPECT_EQ(second.trigger_fault_reason, "driver fault");

  EXPECT_FALSE(manager.hasPendingEvent());
}

TEST(EventManagerTest, KeepsJointWarningAfterPendingFault)
{
  EventManager manager(2.0);

  EXPECT_TRUE(manager.onRobotState(makeFaultState(), 1000, 1, 2, "driver fault"));
  EXPECT_TRUE(manager.onJointStates(makeWarningStates(), 1001));

  ASSERT_TRUE(manager.hasPendingEvent());
  auto first = manager.takePendingEvent();
  EXPECT_EQ(first.event_type, "system_fault");

  ASSERT_TRUE(manager.hasPendingEvent());
  auto second = manager.takePendingEvent();
  EXPECT_EQ(second.event_type, "joint_warning");
  EXPECT_EQ(second.motor_error, 7);

  EXPECT_FALSE(manager.hasPendingEvent());
}

}  // namespace
}  // namespace robot_recorder
