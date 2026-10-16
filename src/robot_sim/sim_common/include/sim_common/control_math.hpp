#pragma once

#include <algorithm>

#include "mj_sim/MotorSpec.h"

namespace sim_common {

// 统一的关节力矩计算路径，狗形/人形仿真共用：
//   1. 前馈 + PD 计算需求力矩（enabled=false 时直接输出 0）；
//   2. 限幅到电机峰值扭矩范围 [-spec.max_torque, spec.max_torque]；
//   3. 应用电机 TN 曲线做速度相关的物理限制。
//
// 限幅值来自 MotorSpec 的 max_torque（各电机类型的固有物理量），不再从
// MuJoCo XML 的 actuator ctrlrange 读取，保证两类仿真的力矩计算方式完全一致。
inline double ComputeJointTorque(
    const motor_spec::MotorSpec& spec,
    bool enabled,
    double effort, double kp, double kd,
    double target_position, double current_position,
    double target_velocity, double current_velocity) {
  if (!enabled) {
    return 0.0;
  }

  const double torque = std::clamp(
      effort + kp * (target_position - current_position) +
          kd * (target_velocity - current_velocity),
      -spec.max_torque, spec.max_torque);
  return motor_spec::clampTorque(spec, current_velocity, torque);
}

}  // namespace sim_common
