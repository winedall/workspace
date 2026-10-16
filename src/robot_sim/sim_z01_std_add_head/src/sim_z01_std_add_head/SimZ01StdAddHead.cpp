// SimZ01StdAddHead.cpp
#include "sim_z01_std_add_head/SimZ01StdAddHead.h"

#include "mj_sim/MotorSpec.h"
#include "robot_header/Z01_std_add_head.h"
#include "sim_common/control_math.hpp"

using namespace Z01_std_add_head;

namespace
{

// z01_std_add_head 的电机参数与 z01_std 不同：kv1=1.0, kv2=2.0，腰部速度 466.0 rad/s。
// 基于通用电机型号覆写 kv1/kv2/腰部速度，生成该仿真的专用电机规格。
// 每个关节对应的电机规格，按 kActuatorsName / kJointsName 顺序排列。
// Z01_std 量产版电机 TN 曲线数据暂缺，先用无限制电机占位（力矩指令不限幅、
// TN 曲线不削减），后续拿到量产电机数据后再替换为实际型号。
// 每个关节对应的电机规格，按 kActuatorsName / kJointsName 顺序排列。
const std::array<motor_spec::MotorSpec, 29> kJointMotorSpecs = {
    motor_spec::kMotorEncos2806,
    motor_spec::kMotorEncos2806,
    motor_spec::kMotorTienKungHRA58P, 
    motor_spec::kMotorTienKungHRA58P,
    motor_spec::kMotorTienKungHRA58P,  
    motor_spec::kMotorTienKungHRA58P, 
    motor_spec::kMotorTienKungHRA58P, 
    motor_spec::kMotorEncos2806, 
    motor_spec::kMotorEncos2806,  
    motor_spec::kMotorTienKungHRA58P,  
    motor_spec::kMotorTienKungHRA58P,  
    motor_spec::kMotorTienKungHRA58P,  
    motor_spec::kMotorTienKungHRA58P,  
    motor_spec::kMotorTienKungHRA58P,  
    motor_spec::kMotorEncos2806,  
    motor_spec::kMotorEncos2806,  
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


// 本模型的构造参数。控制关节名为 29 个（头部为真实执行器，以 head_pitch
// 开头）；发布布局中腰部 pitch/roll 是占位（模型只有 waist_yaw）。
sim_common::SimNodeOptions MakeSimNodeOptions()
{
  sim_common::SimNodeOptions opts;
  opts.ros.node_name = "sim_z01_std_add_head";
  opts.ros.control_joint_names = {
      "head_pitch",           "head_yaw",            "left_shoulder_pitch", "left_shoulder_roll",
      "left_shoulder_yaw",    "left_elbow",          "left_wrist_yaw",      "left_wrist_pitch",
      "left_wrist_roll",      "right_shoulder_pitch","right_shoulder_roll", "right_shoulder_yaw",
      "right_elbow",          "right_wrist_yaw",     "right_wrist_pitch",   "right_wrist_roll",
      "waist_yaw",            "left_hip_pitch",      "left_hip_roll",       "left_hip_yaw",
      "left_knee",            "left_ankle_pitch",    "left_ankle_roll",     "right_hip_pitch",
      "right_hip_roll",       "right_hip_yaw",       "right_knee",          "right_ankle_pitch",
      "right_ankle_roll",
  };
  // 对外发布的 JointStates 要保持全系统 31 关节视图。
  // 本模型没有 waist_pitch/waist_roll 执行器：两者作为零状态占位，
  // 按真机顺序排在 waist_yaw 之前。
  opts.ros.joint_states_layout = sim_common::BuildJointStatesLayout(
      opts.ros.control_joint_names,
      {sim_common::PlaceholdersBefore("waist_yaw", {"waist_pitch", "waist_roll"})});
  opts.ros.driver_status_message = "mujoco simulation online";

  opts.mj_config.joint_names.assign(kJointsName.begin(), kJointsName.end());
  opts.mj_config.actuator_names.assign(kActuatorsName.begin(), kActuatorsName.end());
  opts.mj_config.base_name = kBaseName;
  opts.mj_config.orientation_sensor_name = kOrientationSensorName;
  opts.mj_config.vel_sensor_name = kVelSensorName;
  opts.mj_config.gyro_sensor_name = kGyroSensorName;
  opts.mj_config.acc_sensor_name = kAccSensorName;
  opts.mj_config.camera_names.assign(kCameraNames.begin(), kCameraNames.end());
  return opts;
}

}  // namespace

SimZ01StdAddHead::SimZ01StdAddHead(const rclcpp::NodeOptions& options)
    : sim_common::SimNodeBase(MakeSimNodeOptions(), options) {}

void SimZ01StdAddHead::ComputeControl(const mjModel* m, mjData* d) {
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
