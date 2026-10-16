#ifndef SIMZ01STDROBOCUP_ROBOTCHANNEL_H
#define SIMZ01STDROBOCUP_ROBOTCHANNEL_H

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>

#include "mj_sim/MotorSpec.h"
#include "mj_sim/MujocoSimBase.h"  // mj::Simulate（深度相机槽位 API）
#include "sim_common/MjInterface.hpp"
#include "sim_common/RobotRosChannel.hpp"
#include "sim_common/SimRosNodeBase.hpp"
#include "sim_z01_std_robocup/RobotModelSpec.h"

namespace sim_z01_std_robocup {

// 一台机器人的出生/复位姿态（世界系）。
struct SpawnPose {
  double x{0.0};
  double y{0.0};
  double z{1.0};
  double yaw{0.0};
};

struct RobotChannelOptions {
  sim_common::SimRosNodeOptions ros;    // 节点名/话题前缀/控制关节名/布局/状态消息
  sim_common::MjInterfaceConfig mj_config;  // 名字已带 robot_0X_ 前缀
  // 初始关节角（控制关节顺序，29 个）。空 = 全部为 0。
  // 在初始化（InitRobotState）与 /reload_* 时使用；/reset_* 不碰关节。
  std::vector<double> initial_joint_positions;
  std::int64_t state_publish_period_us{5000};  // JointStates/IMU 发布节拍
  std::int64_t camera_period_us{33333};        // 深度相机发布周期（0 = 关闭）
  // 物理线程等待深度帧的上限。0 = 异步（推荐）：本轮只请求，下一轮取帧发布，
  // 物理线程不会因为 6 路相机而阻塞。
  std::int64_t depthcam_wait_us{0};
  // 误差注入（传感器误差 + 执行延迟）。每台机器人一份，种子由 spec.index 派生，
  // 所以 6 台的零位偏置/IMU 零偏各不相同。
  sim_common::SimErrorOptions errors{};
};

// RobotChannel = 模型里一台机器人的 MuJoCo 绑定 + 一套 ROS 话题。
//
// 线程约定：
//   - ComputeControl / PublishTick / CameraTick / ApplyPose 只允许物理线程调用；
//   - ROS 侧只有订阅回调与 DriverStatus 定时器跑在 executor 线程，它们只入队/置位。
class RobotChannel {
 public:
  RobotChannel(const RobotModelSpec& spec, const RobotChannelOptions& opts,
               const rclcpp::NodeOptions& node_options);
  ~RobotChannel() = default;

  // 模型加载后第一步：建立 MuJoCo 名字->索引映射、缓存基座地址。
  // 必须在 InitRobotState 之前调用（初始姿态按 MuJoCo 名字写 qpos）。
  bool LoadModel(mjModel* model);

  // 深度相机槽位与 camera_info（需要 mj::Simulate，即必须在 sim_->Load 之后调用）。
  bool ConfigureCamera(const mjModel* model, mj::Simulate* sim);

  // 每个仿真步：读取当前状态 + 消费最新命令 + 写 d->ctrl（前馈 + PD + 电机限幅）。
  void ComputeControl(const mjModel* model, mjData* data);

  // 本机器人执行器的延迟施加（由主节点的 PostComputeControl 按机器人循环调用）。
  void ApplyTorqueDelay(mjData* data);

  // 按 state_publish_period_us 节拍发布 JointStates/IMU。
  // time_reset：仿真时间被重置/重新对齐（由主节点按 sync_sim 判定）。
  void PublishTick(const mjData* data, std::int64_t time_us, bool time_reset);

  // 按 camera_period_us 节拍请求并发布深度相机图像。
  void CameraTick(std::int64_t time_us, bool time_reset);

  // 启动初始化：基座位姿 + 关节角（initial_joint_positions）+ 关节速度清零。
  // 不动命令缓存（启动时本来就是空的）。
  void ApplyPose(mjData* data, const SpawnPose& pose);

  // /reload_*：**完整重载到初始状态**——基座位姿 + 关节角回到 initial_joint_positions
  // + 关节/基座速度清零 + 清空命令缓存（关节力矩回到初始的 0/未使能）。
  void ReloadToInitial(mjData* data, const SpawnPose& pose);

  // /reset_*：只把基座挪到场边（位置 + 朝向 + **基座**速度清零），
  // 关节角、关节速度与关节力矩（命令缓存）全部保持不变，控制侧可无缝继续。
  // 返回 false 表示基座地址无效（未初始化/名字不匹配）。
  bool ResetBase(mjData* data, const SpawnPose& pose);

  // 复位后让下一个 tick 立即发布（上层能马上看到新状态）。
  void ForcePublishNextTick() {
    next_pub_time_us_ = -1;
    next_cam_time_us_ = -1;
  }

  const RobotModelSpec& spec() const { return spec_; }
  int index() const { return spec_.index; }
  int team() const { return spec_.team(); }
  int base_body_id() const { return mj_interface_.GetBaseBodyId(); }
  int depth_camera_slot() const { return depth_slot_; }
  bool ready() const { return ready_; }
  sim_common::MjInterface& mj_interface() { return mj_interface_; }
  sim_common::RobotRosChannel& ros() { return *ros_; }
  // 本机器人的 ROS 节点（主节点把它加入共享 executor）。
  std::shared_ptr<rclcpp::Node> node() const { return ros_; }

  // 当前基座水平位置（出生点避碰用）。
  Eigen::Vector2d BaseXy(const mjData* data) const;

 private:
  void PublishDepthCameraFrame();

  RobotModelSpec spec_;
  RobotChannelOptions opts_;
  sim_common::MjInterface mj_interface_;
  // 误差注入：加载时按 opts_.errors.seed + spec_.index 抽一次，Reload 后不变。
  sim_common::SimErrorModel error_model_;
  std::shared_ptr<sim_common::RobotRosChannel> ros_;
  std::vector<motor_spec::MotorSpec> motor_specs_;  // 与 spec 的 29 个关节对齐

  mj::Simulate* sim_{nullptr};
  const mjModel* model_{nullptr};  // LoadModel 后有效（重载需要 nv）
  bool ready_{false};
  int depth_slot_{-1};
  unsigned int last_depth_frame_{0};
  std::vector<unsigned char> depth_rgb_buf_;
  std::vector<float> depth_buf_;
  int depth_width_{0};
  int depth_height_{0};

  // 状态向量约定与 robot_header 的 SystemQ/SystemDq/SystemTau 一致：
  // q 前 7 位是基座位姿、关节从 7 开始；dq/tau 前 6 位是基座速度/角速度。
  Eigen::VectorXd q_rel_;
  Eigen::VectorXd dq_rel_;
  Eigen::VectorXd tau_rel_;
  Eigen::Vector3d acc_{Eigen::Vector3d::Zero()};

  // 发布出去的那份快照（q_rel_ 经 errors.feedback_* 后的结果），与 q_rel_ 分开：
  // q_rel_ 是 PD 回路输入，原地改它会让控制环也吃到滞后。PublishImuFrame 仍用新鲜值。
  Eigen::VectorXd q_pub_;
  Eigen::VectorXd dq_pub_;
  Eigen::VectorXd tau_pub_;

  std::int64_t next_pub_time_us_{0};
  std::int64_t next_cam_time_us_{0};
};

}  // namespace sim_z01_std_robocup

#endif  // SIMZ01STDROBOCUP_ROBOTCHANNEL_H
