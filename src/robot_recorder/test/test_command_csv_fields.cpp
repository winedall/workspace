#include "robot_recorder/joint_record_sample.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace robot_recorder
{
namespace
{

TEST(CommandCsvFieldsTest, BuildsHeadersForEveryCommandValue)
{
  const auto headers = commandValueHeaders({"joint_a", "joint_b"});

  const std::vector<std::string> expected{
    "joint_a_pos", "joint_b_pos", "---",
    "joint_a_vel", "joint_b_vel", "---",
    "joint_a_tor", "joint_b_tor", "---",
    "joint_a_kp", "joint_b_kp", "---",
    "joint_a_kd", "joint_b_kd", "---",
  };

  EXPECT_EQ(headers, expected);
}

TEST(CommandCsvFieldsTest, BuildsHeadersForEveryStateValue)
{
  const auto headers = stateValueHeaders({"joint_a", "joint_b"});

  const std::vector<std::string> expected{
    "joint_a_pos", "joint_b_pos", "---",
    "joint_a_vel", "joint_b_vel", "---",
    "joint_a_tor", "joint_b_tor", "---",
    "joint_a_err", "joint_b_err", "---",
    "joint_a_merr", "joint_b_merr", "---",
    "joint_a_temp", "joint_b_temp", "---",
    "joint_a_mos", "joint_b_mos", "---",
  };

  EXPECT_EQ(headers, expected);
}

TEST(CommandCsvFieldsTest, AppendsRowsForEveryCommandValue)
{
  JointCommandValues first;
  first.position = 1.0f;
  first.velocity = 2.0f;
  first.effort = 3.0f;
  first.kp = 4.0f;
  first.kd = 5.0f;

  JointCommandValues second;
  second.position = 10.0f;
  second.velocity = 20.0f;
  second.effort = 30.0f;
  second.kp = 40.0f;
  second.kd = 50.0f;

  std::vector<std::string> row{"metadata"};
  appendCommandValues(row, {first, second});

  const std::vector<std::string> expected{
    "metadata",
    "1.000000", "10.000000", "",
    "2.000000", "20.000000", "",
    "3.000000", "30.000000", "",
    "4.000000", "40.000000", "",
    "5.000000", "50.000000", "",
  };

  EXPECT_EQ(row, expected);
}

TEST(CommandCsvFieldsTest, AppendsRowsForEveryStateValue)
{
  JointStateValues first;
  first.position = 1.0f;
  first.velocity = 2.0f;
  first.effort = 3.0f;
  first.joint_error = 4;
  first.motor_error = 5;
  first.motor_temp = 6.0f;
  first.mos_temp = 7.0f;

  JointStateValues second;
  second.position = 10.0f;
  second.velocity = 20.0f;
  second.effort = 30.0f;
  second.joint_error = 40;
  second.motor_error = 50;
  second.motor_temp = 60.0f;
  second.mos_temp = 70.0f;

  std::vector<std::string> row{"metadata"};
  appendStateValues(row, {first, second});

  const std::vector<std::string> expected{
    "metadata",
    "1.000000", "10.000000", "",
    "2.000000", "20.000000", "",
    "3.000000", "30.000000", "",
    "4", "40", "",
    "5", "50", "",
    "6.000000", "60.000000", "",
    "7.000000", "70.000000", "",
  };

  EXPECT_EQ(row, expected);
}

TEST(CommandCsvFieldsTest, BuildsStronglyTypedCommandRecord)
{
  robot_interfaces::msg::JointCommands msg;
  msg.header.stamp.sec = 1;
  msg.header.stamp.nanosec = 500000000;
  msg.trace_id = 42;
  msg.sequence = 7;
  msg.parent_sequence = 6;
  msg.source = "rl";
  msg.parent_source = "policy";
  msg.stage = "desired";
  msg.type = 3;

  robot_interfaces::msg::JointCommand cmd;
  cmd.joint_name = "joint_b";
  cmd.position = 1.0f;
  cmd.velocity = 2.0f;
  cmd.effort = 3.0f;
  cmd.kp = 4.0f;
  cmd.kd = 5.0f;
  msg.commands.push_back(cmd);

  const auto record = makeJointCommandsRecord(msg, {"joint_a", "joint_b"}, 1234, toMillis(msg.header.stamp));

  EXPECT_EQ(record.recorder_time_ms, 1234);
  EXPECT_EQ(record.frame.msg_stamp_ms, 1500);
  EXPECT_EQ(record.frame.trace_id, 42u);
  EXPECT_EQ(record.frame.joints.size(), 2u);
  EXPECT_FLOAT_EQ(record.frame.joints[0].position, 0.0f);
  EXPECT_FLOAT_EQ(record.frame.joints[1].position, 1.0f);
  EXPECT_EQ(record.frame.stage, "desired");
}

TEST(CommandCsvFieldsTest, BuildsStateRecordWithConfiguredJointOrder)
{
  robot_interfaces::msg::JointStates msg;
  msg.sequence = 9;

  robot_interfaces::msg::JointState state;
  state.joint_name = "joint_b";
  state.position = 2.0f;
  msg.states.push_back(state);

  const auto record = makeJointStatesRecord(msg, {"joint_a", "joint_b"}, 2000, 1000);

  EXPECT_EQ(record.recorder_time_ms, 2000);
  EXPECT_EQ(record.frame.joints.size(), 2u);
  EXPECT_FLOAT_EQ(record.frame.joints[0].position, 0.0f);
  EXPECT_FLOAT_EQ(record.frame.joints[1].position, 2.0f);
}

}  // namespace
}  // namespace robot_recorder
