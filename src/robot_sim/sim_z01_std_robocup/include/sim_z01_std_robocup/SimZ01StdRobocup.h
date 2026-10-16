#ifndef SIMZ01STDROBOCUP_SIMZ01STDROBOCUP_H
#define SIMZ01STDROBOCUP_SIMZ01STDROBOCUP_H

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "mj_sim/MujocoSimBase.h"
#include "sim_z01_std_robocup/RobotChannel.h"
#include "sim_z01_std_robocup/RobotModelSpec.h"
#include "sim_z01_std_robocup/SimRosExecutor.h"

namespace sim_z01_std_robocup {

// SimZ01StdRobocup 是 RoboCup 多机器人仿真节点：**一个进程、一个 MuJoCo 窗口**，
// 模型里放 6 台 Z01_std_parallel_ankle + 足球场 + 两个球门 + 位于场心的足球。
//
// 为什么必须是一个仿真实例：mjcb_control 是全局函数指针、
// MujocoSimBase::controller_instance_ 是 static，一个进程只能有一个仿真实例；
// 6 台机器人的力矩计算在同一个 ComputeControl 里循环完成。
//
// ROS 侧：1 个主节点（参数、球/球门全局话题、重载/复位服务）+ 6 个机器人节点
// （每个一套 /robot_0X/* 话题），共用一个 SingleThreadedExecutor。
//
// 话题/服务接口（详见包 README）：
//   /robot_0X/{control/hardware_commands, driver/joint_states, driver/status,
//              imu/nav_all, camera/*}
//   /sim/{ball_pose, ball_twist, goal_team_a, goal_team_b,
//         set_ball_pose, set_ball_twist, ball_preview}
//   /reload_robot_01..06, /reload_all_robot（完整重载到初始状态）
//   /reset_robot_01..06, /reset_all_robot（只挪基座，关节状态不变）
//   /reset_ball（球复位）—— 全部 std_srvs/Trigger
class SimZ01StdRobocup : public MujocoSimBase {
 public:
  explicit SimZ01StdRobocup(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~SimZ01StdRobocup() override;

  // 在 std::make_shared 之后调用（与 SimNodeBase::Start 约定一致）。
  void Start();

 protected:
  void InitRobotState(mjData* d) override;
  void InitInterface() override;
  void InitCamera() override;
  void ComputeControl(const mjModel* m, mjData* d) override;
  // 每步写 d->ctrl 之后的单点后处理：6 台各自施加 errors.delay_ms 的执行延迟。
  void PostComputeControl(const mjModel* m, mjData* d) override;
  void ExchangeMessages(double sync_sim) override;
  void Cleanup() override;

 private:
  // ---- 参数与初始化 ----
  void DeclareParameters();
  void CreateRobotChannels();
  bool LookupSceneObjects();

  // ---- 重载 / 复位 ----
  // 在物理线程里执行（ROS 回调只置原子请求位），避免跨线程改 mjData。
  // /reload_*：完整回到初始状态（基座位姿 + 关节角 + 速度清零 + 命令缓存清空）；
  // /reset_*：只挪基座（位置/朝向/基座速度），关节角/速度/力矩保持不变。
  void ApplyPendingRequests(mjData* d);
  SpawnPose SampleSpawn(int index, const std::vector<Eigen::Vector2d>& teammates);
  void ApplyBallPose(mjData* d, const double* xyz);
  void ApplyBallTwist(mjData* d, const double* vxyz);

  // ---- 发布 ----
  void PublishTick(const mjData* d, std::int64_t time_us, bool time_reset);
  void PublishGoals();
  void CheckCapacity(const mjData* d);

  // ---- 服务回调（executor 线程，只置位）----
  void HandleReloadRobot(int index, const std_srvs::srv::Trigger::Request::SharedPtr& req,
                         std_srvs::srv::Trigger::Response::SharedPtr res);
  void HandleReloadAll(const std_srvs::srv::Trigger::Request::SharedPtr& req,
                       std_srvs::srv::Trigger::Response::SharedPtr res);
  void HandleResetRobot(int index, const std_srvs::srv::Trigger::Request::SharedPtr& req,
                        std_srvs::srv::Trigger::Response::SharedPtr res);
  void HandleResetAll(const std_srvs::srv::Trigger::Request::SharedPtr& req,
                      std_srvs::srv::Trigger::Response::SharedPtr res);
  void HandleResetBall(const std_srvs::srv::Trigger::Request::SharedPtr& req,
                       std_srvs::srv::Trigger::Response::SharedPtr res);
  void HandleSetBallPose(const geometry_msgs::msg::Point::SharedPtr msg);
  void HandleSetBallTwist(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void HandleBallPreview(const geometry_msgs::msg::Point::SharedPtr msg);

  std::string StatusMessage(bool reload, uint32_t robot_mask, bool ball_reset) const;

  // ---- ROS ----
  // 成员声明顺序即析构顺序的反序：executor_ 必须比它持有的节点先销毁，
  // 因此 master_node_/robots_ 声明在 executor_ 之前。
  std::shared_ptr<rclcpp::Node> master_node_;
  std::array<std::unique_ptr<RobotChannel>, RobotModelSpec::kRobotCount> robots_;
  std::unique_ptr<SimRosExecutor> executor_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ball_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr ball_twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goal_team_a_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goal_team_b_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr set_ball_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr set_ball_twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ball_preview_sub_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> reload_robot_srvs_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_all_srv_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> reset_robot_srvs_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_all_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_ball_srv_;
  rclcpp::TimerBase::SharedPtr goal_pub_timer_;

  // ---- 场景对象索引 ----
  int ball_body_id_{-1};
  int ball_qpos_adr_{-1};
  int ball_dof_adr_{-1};
  int ball_preview_mocap_id_{-1};
  double goal_world_pos_[2][3] = {{0.0, 0.0, 0.11}, {0.0, 0.0, 0.11}};  // [0]=team A 进攻球门

  // ---- 参数 ----
  // 误差注入（传感器误差 + 执行延迟）：声明在主节点，注入给每台机器人。
  sim_common::SimErrorOptions error_options_{};
  std::vector<double> initial_joint_positions_;
  std::vector<int64_t> camera_period_us_list_;
  double base_z_{1.0};
  double ball_spawn_[3] = {0.0, 0.0, 0.11};
  double camera_distance_{16.0};
  double camera_azimuth_{90.0};
  double camera_elevation_{-35.0};
  double camera_lookat_z_{0.5};
  double sideline_margin_{0.7};
  double team_a_x_range_[2] = {0.5, 6.5};
  double team_b_x_range_[2] = {-6.5, -0.5};
  double min_teammate_distance_{1.0};
  int spawn_max_attempts_{50};
  int track_robot_index_{0};
  std::int64_t depthcam_wait_us_{0};
  std::int64_t state_publish_period_us_{5000};
  // 场地半宽/半高（与 soccer_field.xml 一致）：边线 y=±4.5。
  static constexpr double kSidelineY = 4.5;

  // ---- 运行时状态 ----
  bool interface_ready_{false};
  std::int64_t next_ball_pub_time_us_{0};
  std::int64_t next_capacity_check_us_{0};
  std::mt19937 rng_;

  // ROS -> 物理线程的请求（只有原子量，物理线程在 ExchangeMessages 里消费）。
  std::atomic<std::uint32_t> reload_robot_mask_{0};
  std::atomic<std::uint32_t> reset_robot_mask_{0};
  std::atomic<bool> reset_ball_pending_{false};
  std::atomic<bool> ball_pose_pending_{false};
  std::array<std::atomic<double>, 3> ball_pose_req_{};
  std::atomic<bool> ball_twist_pending_{false};
  std::array<std::atomic<double>, 3> ball_twist_req_{};
  std::atomic<bool> ball_preview_pending_{false};
  std::array<std::atomic<double>, 3> ball_preview_req_{};
};

}  // namespace sim_z01_std_robocup

#endif  // SIMZ01STDROBOCUP_SIMZ01STDROBOCUP_H
