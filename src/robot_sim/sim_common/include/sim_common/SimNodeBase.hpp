#pragma once

#include <atomic>
#include <cstdint>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "mj_sim/MujocoSimBase.h"
#include "sim_common/MjInterface.hpp"
#include "sim_common/SimRosNodeBase.hpp"

namespace sim_common {

// 仿真节点通用构造参数。模型无关部分（生命周期/话题/节奏）由 SimNodeBase
// 处理，各机器人特有逻辑（电机表、控制器、附加场景）留在派生类。
struct SimNodeOptions {
  SimRosNodeOptions ros;  // 节点名、关节名列表、JointStates 布局、状态消息
  // 初始关节位置（控制名顺序）。为空 = 全部关节从 0 位开始（人形路径）；
  // 非空 = 按名称设置并清零速度（狗形路径）。
  std::vector<double> initial_joint_positions;
  MjInterfaceConfig mj_config;
};

// SimNodeBase = MujocoSimBase（物理/渲染生命周期）+ SimRosNodeBase（ROS 通信层）。
//
// 已由基类实现：Start/RunRosExecutor/executor 线程样板、初始位姿、模型接口
// 初始化、跟踪相机（含深度相机）、200 Hz JointStates/NavAll 发布节奏、
// 相机图像发布。派生类只需实现 ComputeControl()（每个仿真步写入 d->ctrl），
// 并按需重写 HandleInterfaceReady()（模型加载后的附加查表）和
// PublishExtra()（200 Hz 发布块内的附加发布，如足球目标点）。
class SimNodeBase : public MujocoSimBase, public SimRosNodeBase {
 public:
  SimNodeBase(const SimNodeOptions& opts,
              const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
  ~SimNodeBase() override;

  // 单独启动 ROS executor 线程，必须在 std::make_shared 之后调用，
  // 确保 enable_shared_from_this 已就绪。
  void Start();

 protected:
  // MuJoCo 生命周期钩子：由基类实现。
  void InitRobotState(mjData* d) override;
  void InitInterface() override;
  void InitCamera() override;
  void Cleanup() override;
  void ExchangeMessages(double sync_sim) override;
  // 每步写 d->ctrl 之后的单点后处理：施加 errors.delay_ms 的执行延迟。
  void PostComputeControl(const mjModel* m, mjData* d) override;

  // MuJoCo 控制回调：每个仿真步根据缓存的关节命令写入 d->ctrl。
  // 各机器人控制律差异较大（电机表/限幅/消费路径），留给派生类实现。
  void ComputeControl(const mjModel* m, mjData* d) override = 0;

  // InitDataHandle 成功后调用（parallel 的足球查表放在这里）。
  virtual void HandleInterfaceReady() {}
  // 200 Hz 发布块内、IMU 发布后调用（parallel 的足球目标点发布放在这里）。
  virtual void PublishExtra(const mjData* d) {}

  MjInterface mj_interface_;
  bool interface_ready_{false};
  // 误差注入：options 来自 YAML 的 errors.*，model 在 InitInterface() 里按模型
  // 加载结果初始化（初次加载与 GUI Reload 都会重新走一遍）。
  SimErrorOptions error_options_{};
  SimErrorModel error_model_;
  // 状态向量约定（与 robot_header 的 SystemQ/SystemDq/SystemTau 一致）：
  // q 前 7 位为基座位姿，关节从 7 开始；dq/tau 前 6 位为基座速度/角速度。
  Eigen::VectorXd q_rel_;
  Eigen::VectorXd dq_rel_;
  Eigen::VectorXd tau_rel_;
  Eigen::Vector3d acc_ = Eigen::Vector3d::Zero();

  // 发布出去的那份快照（q_rel_ 经 errors.feedback_* 的采样保持/延迟后的结果）。
  // 必须与 q_rel_ 分开：q_rel_ 是 PD 回路的输入，原地改它会让控制环也吃到滞后。
  // FillImu 仍用新鲜的 q_rel_/dq_rel_——真机上 IMU 与关节反馈是两路独立传感器。
  Eigen::VectorXd q_pub_;
  Eigen::VectorXd dq_pub_;
  Eigen::VectorXd tau_pub_;

  // 物理量注入（发布动作与消息对象在 SimRosNodeBase 通信层）：
  // FillJointStates/FillImu 把 MuJoCo 状态写入通信层消息对象，调用方随后调
  // SimRosNodeBase 的 Publish* 发出；PrepareCameraInfo 用模型相机参数
  // 填充通信层的 camera_info_msg_。
  void FillJointStates();
  void FillImu();
  void PrepareCameraInfo();
  // 请求深度相机渲染并按相机周期发布 RGB/深度图像。
  void PublishDepthCamera();

 private:
  void RunRosExecutor();
  void InitTrackingCamera(int base_id);
  void InitDepthCamera();

  SimNodeOptions opts_;
  // 可调参数（来自各仿真包自己的 config YAML）：
  // base_z_ — 初始基座高度；camera_distance_/azimuth_/elevation_ — 相机几何；
  // camera_index_ — 初始窗口相机（0 自由 / 1 跟踪基座 / >=2 固定相机）；
  // state_publish_period_us_ — JointStates/IMU 发布节拍（约 200 Hz）。
  double base_z_{1.0};
  double camera_distance_{3.0};
  double camera_azimuth_{145.0};
  double camera_elevation_{-20.0};
  int camera_index_{1};
  std::int64_t state_publish_period_us_{5000};

  std::thread ros_spin_thread_;
  std::atomic<bool> ros_spin_running_{false};
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::int64_t next_pub_time_us_{0};
  std::int64_t next_cam_pub_time_us_{0};

  // 深度相机使用 mj::Simulate 的槽位 API（多机器人仿真按机器人分槽位）：
  // 单机器人仿真固定用槽位 0；缓冲由本类持有，超时时沿用上一次的帧。
  int depth_camera_slot_{-1};
  unsigned int last_depth_frame_{0};
  std::vector<unsigned char> depth_rgb_buf_;
  std::vector<float> depth_buf_;
  int depth_width_{0};
  int depth_height_{0};
};

}  // namespace sim_common
