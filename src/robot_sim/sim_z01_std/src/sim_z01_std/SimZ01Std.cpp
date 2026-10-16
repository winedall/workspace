// SimZ01Std.cpp
#include "sim_z01_std/SimZ01Std.h"

#include "mj_sim/MotorSpec.h"
#include "robot_header/Z01_std.h"
#include "sim_common/control_math.hpp"

using namespace Z01_std;

namespace
{
const std::array<motor_spec::MotorSpec, 29> kJointMotorSpecs = {
    motor_spec::kMotorTienKungHRA58P,  //  0: M_arm_l_01
    motor_spec::kMotorTienKungHRA58P,  //  1: M_arm_l_02
    motor_spec::kMotorTienKungHRA58P,  //  2: M_arm_l_03
    motor_spec::kMotorTienKungHRA58P,  //  3: M_arm_l_04
    motor_spec::kMotorTienKungHRA58P,  //  4: M_arm_l_05
    motor_spec::kMotorEncos2806,  //  5: M_arm_l_06
    motor_spec::kMotorEncos2806,  //  6: M_arm_l_07
    motor_spec::kMotorTienKungHRA58P,  //  7: M_arm_r_01
    motor_spec::kMotorTienKungHRA58P,  //  8: M_arm_r_02
    motor_spec::kMotorTienKungHRA58P,  //  9: M_arm_r_03
    motor_spec::kMotorTienKungHRA58P,  // 10: M_arm_r_04
    motor_spec::kMotorTienKungHRA58P,  // 11: M_arm_r_05
    motor_spec::kMotorEncos2806,  // 12: M_arm_r_06
    motor_spec::kMotorEncos2806,  // 13: M_arm_r_07
    motor_spec::kMotorUnlimited,  // 14: M_waist_pitch
    motor_spec::kMotorUnlimited,  // 15: M_waist_roll
    motor_spec::kMotorTienKungHRA88P,  // 16: M_waist_yaw
    motor_spec::kMotorTienKungHRA88P,  // 17: M_hip_l_pitch
    motor_spec::kMotorTienKungHRA88P,  // 18: M_hip_l_roll
    motor_spec::kMotorTienKungHRA88P,  // 19: M_hip_l_yaw
    motor_spec::kMotorTienKungHRA88P,  // 20: M_knee_l_pitch
    motor_spec::kMotorTienKungHRA58PParallelPitch,  // 21: M_ankle_l_pitch
    motor_spec::kMotorTienKungHRA58P,  // 22: M_ankle_l_roll
    motor_spec::kMotorTienKungHRA88P,  // 23: M_hip_r_pitch
    motor_spec::kMotorTienKungHRA88P,  // 24: M_hip_r_roll
    motor_spec::kMotorTienKungHRA88P,  // 25: M_hip_r_yaw
    motor_spec::kMotorTienKungHRA88P,  // 26: M_knee_r_pitch
    motor_spec::kMotorTienKungHRA58PParallelPitch,  // 27: M_ankle_r_pitch
    motor_spec::kMotorTienKungHRA58P,  // 28: M_ankle_r_roll
};


// 本模型的构造参数：节点名、控制关节名（MuJoCo T4_std 模型中实际存在
// 执行器的 29 个关节，采用控制系统统一 joint_name）、发布布局
// （头部关节作为占位排在最前）、基座与传感器名。
sim_common::SimNodeOptions MakeSimNodeOptions()
{
  sim_common::SimNodeOptions opts;
  opts.ros.node_name = "sim_z01_std";
  opts.ros.control_joint_names = {
      "left_shoulder_pitch",  "left_shoulder_roll",   "left_shoulder_yaw",   "left_elbow",
      "left_wrist_yaw",       "left_wrist_pitch",     "left_wrist_roll",     "right_shoulder_pitch",
      "right_shoulder_roll",  "right_shoulder_yaw",   "right_elbow",         "right_wrist_yaw",
      "right_wrist_pitch",    "right_wrist_roll",     "waist_pitch",         "waist_roll",
      "waist_yaw",            "left_hip_pitch",       "left_hip_roll",       "left_hip_yaw",
      "left_knee",            "left_ankle_pitch",     "left_ankle_roll",     "right_hip_pitch",
      "right_hip_roll",       "right_hip_yaw",        "right_knee",          "right_ankle_pitch",
      "right_ankle_roll",
  };
  // 对外发布的 JointStates 要保持全系统 31 关节视图。
  // 本模型没有头部执行器：head_yaw/head_pitch 作为零状态占位排在最前。
  opts.ros.joint_states_layout = sim_common::BuildJointStatesLayout(
      opts.ros.control_joint_names,
      {sim_common::PlaceholdersFirst({"head_yaw", "head_pitch"})});
  opts.ros.driver_status_message = "mujoco simulation online";

  opts.mj_config.joint_names.assign(kJointsName.begin(), kJointsName.end());
  opts.mj_config.actuator_names.assign(kActuatorsName.begin(), kActuatorsName.end());
  opts.mj_config.base_name = kBaseName;
  opts.mj_config.orientation_sensor_name = kOrientationSensorName;
  opts.mj_config.vel_sensor_name = kVelSensorName;
  opts.mj_config.gyro_sensor_name = kGyroSensorName;
  opts.mj_config.acc_sensor_name = kAccSensorName;
  return opts;
}

}  // namespace

SimZ01Std::SimZ01Std(const rclcpp::NodeOptions& options)
    : sim_common::SimNodeBase(MakeSimNodeOptions(), options) {}

void SimZ01Std::ComputeControl(const mjModel* m, mjData* d) {
  (void)m;
  if (!interface_ready_ || d == nullptr) {
    return;
  }
  // 读取当前仿真位置和速度，用于 PD 控制计算。
  mj_interface_.ReadJointState(d, q_rel_, dq_rel_);

  // 只取队列中的最新一帧并合并到缓存。排空队列保证不会执行陈旧目标；
  // 缓存设计可以在没有新命令时保持上一帧目标。
  robot_interfaces::msg::JointCommands joint_commands;
  if (TryTakeLatestCommand(joint_commands)) {
    UpdateCommandCache(joint_commands, d->time);

    // 仿真侧经验 Kp/Kd 覆盖：部分关节需要比上游默认值更高的刚度来维持行走稳定。
    // 腰部关节需要高刚度以保持躯干刚性
    sim_joint_commands_[14].kp = 1000.0;
    sim_joint_commands_[14].kd = 50.0;
    sim_joint_commands_[15].kp = 1000.0;
    sim_joint_commands_[15].kd = 50.0;
  }

  // 统一力矩计算路径（与狗形仿真一致）：前馈 + PD → 电机力矩指令限幅 → TN 曲线。
  const auto& motor_ids = mj_interface_.GetMotorIds();
  for (std::size_t i = 0; i < mj_interface_.GetJointCount(); i++) {
    const Eigen::Index idx = static_cast<Eigen::Index>(i);
    const auto& command = sim_joint_commands_[i];
    d->ctrl[motor_ids[i]] = sim_common::ComputeJointTorque(
        kJointMotorSpecs[i], enable_control_ && command.enable,
        command.effort, command.kp, command.kd,
        command.position, q_rel_[idx + 7],
        command.velocity, dq_rel_[idx + 6]);
  }
}
