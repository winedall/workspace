// 钉住 SimStateFill 的填充语义（单机器人 SimNodeBase 与多机器人 RobotRosChannel
// 共用这两个函数，语义必须完全一致）：
// - FillJointStatesMessage：仿真关节 -> JointStates.states 的下标映射、
//   q[7+i]/dq[6+i]/tau[6+i] 的取值位置、enabled 的两种模式、占位关节零状态；
// - FillImuMessage：NavAll 的 pos/vel/acc/quat/euler/gyro 字段、四元数分量顺序
//   与 degree 单位。
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "sim_common/SimStateFill.hpp"

namespace
{

sim_common::JointStatesLayout MakeLayout()
{
  // 3 个仿真关节，并在关节 "b" 之前插入两个未建模占位关节。
  return sim_common::BuildJointStatesLayout({"a", "b", "c"},
                                            {sim_common::PlaceholdersBefore("b", {"x", "y"})});
}

std::vector<sim_common::SimJointCommand> MakeCommands()
{
  std::vector<sim_common::SimJointCommand> commands(3);
  commands[0].enable = true;
  commands[1].enable = false;
  commands[2].enable = true;
  return commands;
}

}  // namespace

TEST(StateFill, JointStatesFollowLayoutAndSystemQOffsets)
{
  const auto layout = MakeLayout();
  const auto commands = MakeCommands();

  // q = [base_pos(3), base_quat(4), joints(3)]；dq/tau = [base_vel(3), gyro(3), joints(3)]
  Eigen::VectorXd q = Eigen::VectorXd::Zero(10);
  Eigen::VectorXd dq = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(9);
  for (int i = 0; i < 3; ++i) {
    q[7 + i] = 0.1 * (i + 1);
    dq[6 + i] = 0.2 * (i + 1);
    tau[6 + i] = 0.3 * (i + 1);
  }

  robot_interfaces::msg::JointStates msg;
  msg.states.resize(5);
  sim_common::FillJointStatesMessage(commands, layout, false, true, q, dq, tau, msg);

  // 布局顺序：a, x, y, b, c -> 仿真关节 a/b/c 落在 0/3/4。
  ASSERT_EQ(msg.states.size(), 5u);
  EXPECT_EQ(layout.sim_to_msg_index[0], 0);
  EXPECT_EQ(layout.sim_to_msg_index[1], 3);
  EXPECT_EQ(layout.sim_to_msg_index[2], 4);

  // JointState 的位置/速度/力矩字段是 float32，用 FLOAT_EQ 比对。
  EXPECT_FLOAT_EQ(msg.states[0].position, 0.1f);
  EXPECT_FLOAT_EQ(msg.states[3].position, 0.2f);
  EXPECT_FLOAT_EQ(msg.states[4].position, 0.3f);
  EXPECT_FLOAT_EQ(msg.states[4].velocity, 0.6f);
  EXPECT_FLOAT_EQ(msg.states[4].effort, 0.9f);

  // 占位关节固定零状态 + 无故障。
  for (const auto index : {1u, 2u}) {
    EXPECT_FLOAT_EQ(msg.states[index].position, 0.0f);
    EXPECT_FLOAT_EQ(msg.states[index].velocity, 0.0f);
    EXPECT_FLOAT_EQ(msg.states[index].effort, 0.0f);
    EXPECT_EQ(msg.states[index].joint_error, robot_interfaces::msg::JointState::JOINT_ERR_NONE);
  }
}

TEST(StateFill, JointEnabledSemanticsForBothSimulatorFamilies)
{
  const auto layout = MakeLayout();
  const auto commands = MakeCommands();

  Eigen::VectorXd q = Eigen::VectorXd::Zero(10);
  Eigen::VectorXd dq = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(9);

  // 人形包语义：state.enabled 直接取命令使能位，与 enable_control 无关。
  robot_interfaces::msg::JointStates humanoid;
  humanoid.states.resize(5);
  sim_common::FillJointStatesMessage(commands, layout, false, false, q, dq, tau, humanoid);
  EXPECT_TRUE(humanoid.states[0].enabled);
  EXPECT_FALSE(humanoid.states[3].enabled);
  EXPECT_TRUE(humanoid.states[4].enabled);

  // 狗形包语义：enable_control 与命令使能位必须同时成立。
  robot_interfaces::msg::JointStates dog;
  dog.states.resize(5);
  sim_common::FillJointStatesMessage(commands, layout, true, false, q, dq, tau, dog);
  EXPECT_FALSE(dog.states[0].enabled);
  EXPECT_FALSE(dog.states[4].enabled);

  robot_interfaces::msg::JointStates dog_enabled;
  dog_enabled.states.resize(5);
  sim_common::FillJointStatesMessage(commands, layout, true, true, q, dq, tau, dog_enabled);
  EXPECT_TRUE(dog_enabled.states[0].enabled);
  EXPECT_FALSE(dog_enabled.states[3].enabled);
  EXPECT_TRUE(dog_enabled.states[4].enabled);
}

TEST(StateFill, ImuMessageCarriesBaseStateInNavAllConvention)
{
  Eigen::VectorXd q = Eigen::VectorXd::Zero(10);
  Eigen::VectorXd dq = Eigen::VectorXd::Zero(9);
  q[0] = 1.5;  // base x -> pos.longitude
  q[1] = -2.5;
  q[2] = 0.9;
  // 绕 z 轴 +90°：MuJoCo 四元数序 (w,x,y,z) 存在 q[6],q[3],q[4],q[5]。
  q[6] = std::sqrt(0.5);
  q[3] = 0.0;
  q[4] = 0.0;
  q[5] = std::sqrt(0.5);
  for (int i = 0; i < 3; ++i) {
    dq[i] = 0.5 * (i + 1);        // base 线速度
    dq[3 + i] = 0.1 * (i + 1);    // gyro（rad/s）
  }
  const Eigen::Vector3d acc(0.1, -0.2, 9.81);

  yesense_interface::msg::NavAll imu;
  sim_common::FillImuMessage(q, dq, acc, imu);

  // NavAll 的位置/速度/加速度/陀螺/欧拉角字段是 float32，用 FLOAT_EQ/近邻比对。
  EXPECT_FLOAT_EQ(imu.pos.longitude, 1.5);
  EXPECT_FLOAT_EQ(imu.pos.latitude, -2.5);
  EXPECT_FLOAT_EQ(imu.pos.altitude, 0.9);

  EXPECT_FLOAT_EQ(imu.vel.vel_e, 0.5f);
  EXPECT_FLOAT_EQ(imu.vel.vel_n, 1.0f);
  EXPECT_FLOAT_EQ(imu.vel.vel_u, 1.5f);

  EXPECT_FLOAT_EQ(imu.acc.x, 0.1f);
  EXPECT_FLOAT_EQ(imu.acc.y, -0.2f);
  EXPECT_FLOAT_EQ(imu.acc.z, 9.81f);

  // 四元数按 (w,x,y,z) 原样透出（float64 字段）。
  EXPECT_NEAR(imu.quat.q0, std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(imu.quat.q3, std::sqrt(0.5), 1e-12);

  // gyro 为 degree/s；euler 为 degree，yaw 应为 +90°。
  EXPECT_NEAR(imu.gyro.x, 0.1 * 180.0 / M_PI, 1e-4);
  EXPECT_NEAR(imu.gyro.y, 0.2 * 180.0 / M_PI, 1e-4);
  EXPECT_NEAR(imu.gyro.z, 0.3 * 180.0 / M_PI, 1e-4);
  EXPECT_NEAR(imu.euler.yaw, 90.0, 1e-4);
  EXPECT_NEAR(imu.euler.roll, 0.0, 1e-4);
  EXPECT_NEAR(imu.euler.pitch, 0.0, 1e-4);
}
