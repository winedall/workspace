// SimZ01StdParallelAnkle.cpp
#include "sim_z01_std_parallel_ankle/SimZ01StdParallelAnkle.h"

#include "mj_sim/MotorSpec.h"
#include "robot_header/Z01_std_parallel_ankle.h"
#include "sim_common/control_math.hpp"

using namespace Z01_std_parallel_ankle;

namespace
{

// z01_std_parallel_ankle 的电机参数：并联腿踝关节使用 4310Parallel 电机，
// 其余关节与 z01_std_add_head 保持一致。
// 每个关节对应的电机规格，按 kActuatorsName / kJointsName 顺序排列。
// Z01_std 量产版电机 TN 曲线数据暂缺，先用无限制电机占位（力矩指令不限幅、
// TN 曲线不削减），后续拿到量产电机数据后再替换为实际型号。
// 每个关节对应的电机规格，按 kActuatorsName / kJointsName 顺序排列。
// const std::array<motor_spec::MotorSpec, 29> kJointMotorSpecs = {
//     motor_spec::kMotorEncos2806,
//     motor_spec::kMotorEncos2806,
//     motor_spec::kMotorTienKungHRA58P, 
//     motor_spec::kMotorTienKungHRA58P,
//     motor_spec::kMotorTienKungHRA58P,  
//     motor_spec::kMotorTienKungHRA58P, 
//     motor_spec::kMotorTienKungHRA58P, 
//     motor_spec::kMotorEncos2806, 
//     motor_spec::kMotorEncos2806,  
//     motor_spec::kMotorTienKungHRA58P,  
//     motor_spec::kMotorTienKungHRA58P,  
//     motor_spec::kMotorTienKungHRA58P,  
//     motor_spec::kMotorTienKungHRA58P,  
//     motor_spec::kMotorTienKungHRA58P,  
//     motor_spec::kMotorEncos2806,  
//     motor_spec::kMotorEncos2806,  
//     motor_spec::kMotorTienKungHRA88P,  // 16: M_waist_yaw
//     motor_spec::kMotorTienKungHRA88P,  // 17: M_hip_l_pitch
//     motor_spec::kMotorTienKungHRA88P,  // 18: M_hip_l_roll
//     motor_spec::kMotorTienKungHRA88P,  // 19: M_hip_l_yaw
//     motor_spec::kMotorTienKungHRA88P,  // 20: M_knee_l_pitch
//     motor_spec::kMotorTienKungHRA58P,  // 21: M_ankle_l_pitch
//     motor_spec::kMotorTienKungHRA58P,  // 22: M_ankle_l_roll
//     motor_spec::kMotorTienKungHRA88P,  // 23: M_hip_r_pitch
//     motor_spec::kMotorTienKungHRA88P,  // 24: M_hip_r_roll
//     motor_spec::kMotorTienKungHRA88P,  // 25: M_hip_r_yaw
//     motor_spec::kMotorTienKungHRA88P,  // 26: M_knee_r_pitch
//     motor_spec::kMotorTienKungHRA58P,  // 27: M_ankle_r_pitch
//     motor_spec::kMotorTienKungHRA58P,  // 28: M_ankle_r_roll
// };
const std::array<motor_spec::MotorSpec, 29> kJointMotorSpecs = {
    motor_spec::kMotorUnlimited,
    motor_spec::kMotorUnlimited,
    motor_spec::kMotorUnlimited, 
    motor_spec::kMotorUnlimited,
    motor_spec::kMotorUnlimited,  
    motor_spec::kMotorUnlimited, 
    motor_spec::kMotorUnlimited, 
    motor_spec::kMotorUnlimited, 
    motor_spec::kMotorUnlimited,  
    motor_spec::kMotorUnlimited,  
    motor_spec::kMotorUnlimited,  
    motor_spec::kMotorUnlimited,  
    motor_spec::kMotorUnlimited,  
    motor_spec::kMotorUnlimited,  
    motor_spec::kMotorUnlimited,  
    motor_spec::kMotorUnlimited,  
    motor_spec::kMotorUnlimited,  // 16: M_waist_yaw
    motor_spec::kMotorUnlimited,  // 17: M_hip_l_pitch
    motor_spec::kMotorUnlimited,  // 18: M_hip_l_roll
    motor_spec::kMotorUnlimited,  // 19: M_hip_l_yaw
    motor_spec::kMotorUnlimited,  // 20: M_knee_l_pitch
    motor_spec::kMotorUnlimited,  // 21: M_ankle_l_pitch
    motor_spec::kMotorUnlimited,  // 22: M_ankle_l_roll
    motor_spec::kMotorUnlimited,  // 23: M_hip_r_pitch
    motor_spec::kMotorUnlimited,  // 24: M_hip_r_roll
    motor_spec::kMotorUnlimited,  // 25: M_hip_r_yaw
    motor_spec::kMotorUnlimited,  // 26: M_knee_r_pitch
    motor_spec::kMotorUnlimited,  // 27: M_ankle_r_pitch
    motor_spec::kMotorUnlimited,  // 28: M_ankle_r_roll
};


// 本模型的构造参数。控制关节名以 head_yaw 开头（与 add_head 的
// head_pitch 开头相反）；并联腿模型对控制系统的接口仍使用
// left_ankle_pitch/left_ankle_roll 统一命名（MuJoCo 内部为 *_active）。
sim_common::SimNodeOptions MakeSimNodeOptions()
{
  sim_common::SimNodeOptions opts;
  opts.ros.node_name = "sim_z01_std_parallel_ankle";
  opts.ros.control_joint_names = {
      "head_yaw",            "head_pitch",          "left_shoulder_pitch", "left_shoulder_roll",
      "left_shoulder_yaw",   "left_elbow",          "left_wrist_yaw",      "left_wrist_pitch",
      "left_wrist_roll",     "right_shoulder_pitch","right_shoulder_roll", "right_shoulder_yaw",
      "right_elbow",         "right_wrist_yaw",     "right_wrist_pitch",   "right_wrist_roll",
      "waist_yaw",           "left_hip_pitch",      "left_hip_roll",       "left_hip_yaw",
      "left_knee",           "left_ankle_pitch",    "left_ankle_roll",     "right_hip_pitch",
      "right_hip_roll",      "right_hip_yaw",       "right_knee",          "right_ankle_pitch",
      "right_ankle_roll",
  };
  // 对外发布的 JointStates 要保持全系统 31 关节视图。
  // 本模型没有 waist_pitch/waist_roll 执行器：两者作为零状态占位，
  // 按真机顺序排在 waist_yaw 之前。
  opts.ros.joint_states_layout = sim_common::BuildJointStatesLayout(
      opts.ros.control_joint_names,
      {sim_common::PlaceholdersBefore("waist_yaw", {"waist_pitch", "waist_roll"})});
  opts.ros.driver_status_message = "mujoco simulation online (parallel leg)";

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

SimZ01StdParallelAnkle::SimZ01StdParallelAnkle(const rclcpp::NodeOptions& options)
    : sim_common::SimNodeBase(MakeSimNodeOptions(), options) {}


void SimZ01StdParallelAnkle::ComputeControl(const mjModel* m, mjData* d) {
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

