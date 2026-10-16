#pragma once

// RobotRosChannel 是多机器人仿真（sim_z01_std_robocup）里"一台机器人"的 ROS 通道：
// 每个实例就是一个 rclcpp::Node（话题前缀由 SimRosNodeOptions::topic_prefix 指定），
// 把 SimRosNodeBase 的受保护能力以明确接口暴露给不继承 Node 的物理层。
//
// 线程约定：
//   - ConsumeLatestCommand / PublishJointStateFrame / PublishImuFrame /
//     PublishCameraFrame / UpdateCameraInfo 只允许物理线程调用；
//   - 订阅回调与 DriverStatus 定时器由 executor 线程驱动，它们只做入队（spsc_queue）。
//
// 关节状态的填充复用 sim_common/SimStateFill.hpp 的自由函数，与单机器人仿真
// （SimNodeBase）共用同一份语义。

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/joint_commands.hpp>

#include "sim_common/SimRosNodeBase.hpp"
#include "sim_common/SimStateFill.hpp"

namespace sim_common {

class RobotRosChannel : public SimRosNodeBase {
 public:
  RobotRosChannel(const SimRosNodeOptions& opts,
                  const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions())
      : SimRosNodeBase(opts, node_options) {}

  // 物理线程：排空命令队列并只保留最新一帧（缓存语义见 TryTakeLatestCommand）。
  // time 取仿真时间（d->time），供 errors.command_delay_ms 的延迟线索引。
  void ConsumeLatestCommand(double time) {
    robot_interfaces::msg::JointCommands msg;
    if (TryTakeLatestCommand(msg)) {
      UpdateCommandCache(msg, time);
    }
  }

  // 物理线程：写入并发布一帧 JointStates（调用前需先填好 q/dq/tau）。
  void PublishJointStateFrame(const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                              const Eigen::VectorXd& tau) {
    FillJointStatesMessage(sim_joint_commands_, joint_states_layout_,
                           joint_enabled_requires_enable_control_, enable_control_, q, dq, tau,
                           joint_states_msg_);
    PublishJointStates();
  }

  // 物理线程：写入并发布一帧 IMU（NavAll）。
  void PublishImuFrame(const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                       const Eigen::Vector3d& acc) {
    FillImuMessage(q, dq, acc, imu_data_);
    PublishImu();
  }

  // 物理线程：用模型相机参数准备 CameraInfo（只需在模型加载后调用一次）。
  void UpdateCameraInfo(const mjModel& model, int cam_id) {
    FillCameraInfoMessage(model, cam_id, camera_frame_id_, camera_info_msg_);
    camera_info_ready_ = true;
  }

  // 物理线程：发布一帧深度/RGB 图像（含 camera_info）。
  void PublishCameraFrame(const std::vector<unsigned char>& rgb,
                          const std::vector<float>& depth, int width, int height) {
    PublishImages(rgb, depth, width, height);
  }

  // 物理线程：把命令缓存清回初始状态（position/velocity/effort/kp/kd = 0、
  // enable = false），即"关节力矩回到初始设置值"。用于 /reload_*。
  void ClearCommands() {
    for (auto& command : sim_joint_commands_) {
      command = SimJointCommand{};
    }
  }

  const std::vector<SimJointCommand>& commands() const { return sim_joint_commands_; }
  bool enable_control() const { return enable_control_; }
  std::int64_t camera_period_us() const { return camera_period_us_; }
  void set_camera_period_us(std::int64_t period_us) { camera_period_us_ = period_us; }
  const std::string& camera_frame_id() const { return camera_frame_id_; }
};

}  // namespace sim_common
