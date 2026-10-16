#include "sim_d040_poc/SimD040Poc.h"

#include <array>
#include <limits>

#include "mj_sim/MotorSpec.h"
#include "robot_header/D040_poc.h"
#include "sim_common/control_math.hpp"

namespace sim_d040_poc {
namespace {

constexpr std::array<double, D040_poc::kJointsNum> kInitialJointPositions = {
    0.0, 0.8, -1.5,
    0.0, 0.8, -1.5,
    0.0, 0.8, -1.5,
    0.0, 0.8, -1.5,
};

// D4 尚无电机 TN 曲线数据：TN 参数用无限制占位，力矩指令限幅取自 D4 模型 XML
// 的执行器限幅（髋 ±90 Nm，大腿 ±130 Nm，小腿 ±195 Nm），保持原仿真动力学不变。
// 限幅值由 max_torque 承载（TN 参数 v1=v2=inf、系数 1.0，故 TN 不再额外削减）。
constexpr motor_spec::MotorSpec MakeUnlimitedSpec(double torque_limit, const char* name) {
  return {
      std::numeric_limits<double>::max(),   // v1
      std::numeric_limits<double>::max(),   // v2
      torque_limit,                         // max_torque — 兼作力矩指令限幅
      1.0,
      1.0,
      name,
  };
}

constexpr std::array<motor_spec::MotorSpec, D040_poc::kJointsNum> kJointMotorSpecs = {
    MakeUnlimitedSpec(90.0, "Unlimited FL_hip"),
    MakeUnlimitedSpec(130.0, "Unlimited FL_thigh"),
    MakeUnlimitedSpec(195.0, "Unlimited FL_calf"),
    MakeUnlimitedSpec(90.0, "Unlimited FR_hip"),
    MakeUnlimitedSpec(130.0, "Unlimited FR_thigh"),
    MakeUnlimitedSpec(195.0, "Unlimited FR_calf"),
    MakeUnlimitedSpec(90.0, "Unlimited RL_hip"),
    MakeUnlimitedSpec(130.0, "Unlimited RL_thigh"),
    MakeUnlimitedSpec(195.0, "Unlimited RL_calf"),
    MakeUnlimitedSpec(90.0, "Unlimited RR_hip"),
    MakeUnlimitedSpec(130.0, "Unlimited RR_thigh"),
    MakeUnlimitedSpec(195.0, "Unlimited RR_calf"),
};

// 本模型的构造参数。与 d015 相同（恒等布局、FIFO 消费、enabled AND 语义、
// 不输出基座位置/速度真值），区别在基座高度 0.52、全零初始髋角。
sim_common::SimNodeOptions MakeSimNodeOptions()
{
  sim_common::SimNodeOptions opts;
  opts.ros.node_name = "sim_d040_poc";
  opts.ros.control_joint_names.assign(D040_poc::kJointsName.begin(), D040_poc::kJointsName.end());
  // 发布的关节与仿真关节一一对应（无未建模关节，故无占位）。
  opts.ros.joint_states_layout =
      sim_common::BuildJointStatesLayout(opts.ros.control_joint_names);
  opts.ros.driver_status_message = "D4 MuJoCo simulation online";
  opts.ros.joint_enabled_requires_enable_control = true;

  opts.initial_joint_positions.assign(kInitialJointPositions.begin(), kInitialJointPositions.end());

  opts.mj_config.joint_names.assign(D040_poc::kJointsName.begin(), D040_poc::kJointsName.end());
  opts.mj_config.actuator_names.assign(D040_poc::kActuatorsName.begin(),
                                       D040_poc::kActuatorsName.end());
  opts.mj_config.base_name = D040_poc::kBaseName;
  opts.mj_config.orientation_sensor_name = D040_poc::kOrientationSensorName;
  opts.mj_config.gyro_sensor_name = D040_poc::kGyroSensorName;
  opts.mj_config.acc_sensor_name = D040_poc::kAccSensorName;
  opts.mj_config.fill_base_pose = false;
  return opts;
}

}  // namespace

SimD040Poc::SimD040Poc(const rclcpp::NodeOptions& options)
    : sim_common::SimNodeBase(MakeSimNodeOptions(), options) {}

void SimD040Poc::ComputeControl(const mjModel*, mjData* data) {
  if (!interface_ready_ || data == nullptr) {
    return;
  }

  mj_interface_.ReadJointState(data, q_rel_, dq_rel_);
  // 狗形控制器逐帧 FIFO 消费全部积压命令。
  robot_interfaces::msg::JointCommands commands;
  while (PopCommand(commands)) {
    UpdateCommandCache(commands, data->time);
  }

  // 统一力矩计算路径（与人形仿真一致）：前馈 + PD → 电机力矩指令限幅 → TN 曲线。
  // 力矩指令限幅来自 kJointMotorSpecs 的 max_torque，不再读取 XML ctrlrange。
  const auto& motor_ids = mj_interface_.GetMotorIds();
  for (std::size_t i = 0; i < mj_interface_.GetJointCount(); ++i) {
    const Eigen::Index idx = static_cast<Eigen::Index>(i);
    const auto& command = sim_joint_commands_[i];
    data->ctrl[motor_ids[i]] = sim_common::ComputeJointTorque(
        kJointMotorSpecs[i], enable_control_ && command.enable,
        command.effort, command.kp, command.kd,
        command.position, q_rel_[idx + 7],
        command.velocity, dq_rel_[idx + 6]);
  }
}

}  // namespace sim_d040_poc
