#pragma once

// SimStateFill 把"MuJoCo 状态 -> ROS 消息"的填充逻辑抽成自由函数：
// 单机器人仿真（SimNodeBase）与多机器人仿真（sim_z01_std_robocup 的
// RobotRosChannel）都从这里取同一份实现，避免关节状态/IMU 语义出现两份副本。
//
// 这些函数的入参都是内部数组约定（与 robot_header 的 SystemQ/SystemDq/SystemTau
// 一致）：q = [base_pos(3), base_quat(x,y,z,w)(4), joints(n)]，
// dq/tau = [base_vel(3), gyro(3), joints(n)]。

#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include <Eigen/Core>
#include <robot_interfaces/msg/joint_states.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <yesense_interface/msg/nav_all.hpp>

#include "sim_common/SimRosNodeBase.hpp"
#include "sim_common/quaternion_utils.hpp"

namespace sim_common {

// 把仿真关节状态写入 JointStates 消息。
// sim_joint_commands 的顺序与 joint_states_layout 的仿真关节顺序一致；
// enabled 语义由 joint_enabled_requires_enable_control 决定（狗形包要求
// enable_control 与关节使能同时成立，人形包只看关节使能位）。
inline void FillJointStatesMessage(const std::vector<SimJointCommand>& sim_joint_commands,
                                   const JointStatesLayout& layout,
                                   bool joint_enabled_requires_enable_control,
                                   bool enable_control,
                                   const Eigen::VectorXd& q,
                                   const Eigen::VectorXd& dq,
                                   const Eigen::VectorXd& tau,
                                   robot_interfaces::msg::JointStates& out) {
  const std::size_t n = layout.sim_to_msg_index.size();
  for (std::size_t i = 0; i < n; ++i) {
    const int msg_index = layout.sim_to_msg_index[i];
    if (msg_index < 0) {
      continue;
    }
    auto& state = out.states[static_cast<std::size_t>(msg_index)];
    state.position = q[static_cast<Eigen::Index>(i) + 7];
    state.velocity = dq[static_cast<Eigen::Index>(i) + 6];
    state.effort = tau[static_cast<Eigen::Index>(i) + 6];
    state.enabled = joint_enabled_requires_enable_control
                        ? (enable_control && sim_joint_commands[i].enable)
                        : sim_joint_commands[i].enable;
    state.joint_error = robot_interfaces::msg::JointState::JOINT_ERR_NONE;
    state.error_message.clear();
  }
  // 占位关节（头部/腰部等未建模关节）固定发布零数据 + 无故障。
  for (const auto& placeholder : layout.placeholders) {
    out.states[placeholder.msg_index].joint_error =
        robot_interfaces::msg::JointState::JOINT_ERR_NONE;
    out.states[placeholder.msg_index].error_message.clear();
  }
}

// 把基座位姿/速度/加速度写入 NavAll（兼容真实 yesense 驱动的字段约定）。
inline void FillImuMessage(const Eigen::VectorXd& q,
                           const Eigen::VectorXd& dq,
                           const Eigen::Vector3d& acc,
                           yesense_interface::msg::NavAll& out) {
  // MuJoCo 基座姿态四元数按浮动基座约定读取，再转换为 NavAll 欧拉角字段。
  Eigen::Quaterniond quat(q[6], q[3], q[4], q[5]);
  if (quat.norm() < 1.0e-9) {
    quat = Eigen::Quaterniond::Identity();
  } else {
    quat.normalize();
  }
  const Eigen::Vector3d euler = QuaternionToEulerRPY(quat);

  // NavAll 中 gyro/euler 使用旧 yesense 驱动兼容单位：角速度和欧拉角均为 degree 系。
  // robot_rl_manager 会在订阅后转换成 rad/s 和 projected_gravity。
  out.pos.longitude = q[0];
  out.pos.latitude = q[1];
  out.pos.altitude = q[2];

  out.vel.vel_e = dq[0];
  out.vel.vel_n = dq[1];
  out.vel.vel_u = dq[2];

  out.acc.x = acc[0];
  out.acc.y = acc[1];
  out.acc.z = acc[2];

  out.quat.q0 = quat.w();
  out.quat.q1 = quat.x();
  out.quat.q2 = quat.y();
  out.quat.q3 = quat.z();

  out.gyro.x = dq[3] * 180.0 / M_PI;
  out.gyro.y = dq[4] * 180.0 / M_PI;
  out.gyro.z = dq[5] * 180.0 / M_PI;

  out.euler.roll = euler[0] * 180.0 / M_PI;
  out.euler.pitch = euler[1] * 180.0 / M_PI;
  out.euler.yaw = euler[2] * 180.0 / M_PI;
}

// 用模型里的相机参数填充 CameraInfo（分辨率与 fovy 直接读模型，避免与 XML 手动同步）。
inline void FillCameraInfoMessage(const mjModel& model, int cam_id,
                                  const std::string& frame_id,
                                  sensor_msgs::msg::CameraInfo& out) {
  if (cam_id < 0 || cam_id >= model.ncam) {
    return;
  }
  const int width = model.cam_resolution[cam_id * 2];
  const int height = model.cam_resolution[cam_id * 2 + 1];
  const double fovy = model.cam_fovy[cam_id];

  // 计算焦距（像素单位）
  const double fy = (height / 2.0) / std::tan(fovy / 2.0);
  const double fx = fy;  // 假设正方形像素

  out.header.frame_id = frame_id;
  out.width = width;
  out.height = height;
  out.k[0] = fx;
  out.k[1] = 0.0;
  out.k[2] = width / 2.0;
  out.k[3] = 0.0;
  out.k[4] = fy;
  out.k[5] = height / 2.0;
  out.k[6] = 0.0;
  out.k[7] = 0.0;
  out.k[8] = 1.0;

  // 无畸变
  out.d.resize(0);
  out.distortion_model = "plumb_bob";
}

}  // namespace sim_common
