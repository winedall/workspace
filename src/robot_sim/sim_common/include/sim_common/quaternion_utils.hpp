#pragma once

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sim_common {

// 将旋转四元数转换为 ZYX 内旋欧拉角，返回 {roll, pitch, yaw}（弧度）。
// 原狗形仿真包返回 {roll,pitch,yaw}，人形包返回 {yaw,pitch,roll}；
// 两者经各自消费路径映射到 NavAll 的字段完全一致，此处统一为狗形版本的
// 数学实现（含万向节分支），所有消费方统一按 {roll,pitch,yaw} 读取。
inline Eigen::Vector3d QuaternionToEulerRPY(const Eigen::Quaterniond& quaternion) {
  const Eigen::Matrix3d rotation = quaternion.normalized().toRotationMatrix();
  const double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));

  double roll = 0.0;
  double yaw = 0.0;
  if (std::abs(std::cos(pitch)) > 1.0e-8) {
    roll = std::atan2(rotation(2, 1), rotation(2, 2));
    yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  } else {
    roll = std::atan2(-rotation(1, 2), rotation(1, 1));
  }
  return {roll, pitch, yaw};
}

}  // namespace sim_common
