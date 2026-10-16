#ifndef SIMZ01STDFOOTBALLPLAYER_H
#define SIMZ01STDFOOTBALLPLAYER_H

#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/bool.hpp>

#include "sim_common/SimNodeBase.hpp"

// SimZ01StdFootballPlayer 是 Z01_std_football_player 模型（并联踝 + 足球场景 +
// 深度相机）的 MuJoCo 仿真节点。
//
// 继承关系说明：
// - sim_common::SimNodeBase（= MujocoSimBase + SimRosNodeBase）：负责
//   MuJoCo 模型加载/窗口/仿真主循环/Run() 生命周期，以及 ROS 2 topic、
//   状态消息、命令缓存、200 Hz 发布节奏、相机图像发布；
// - SimZ01StdFootballPlayer：保留机器人特有逻辑（电机表、控制律）和
//   足球场景（球/球门显示切换、目标点发布、遥操作回调）。
//
// 该节点在控制链路中扮演"仿真驱动"角色，替代 robot_driver_node：
// robot_control -> /robot/control/hardware_commands -> sim_z01_std_football_player
// sim_z01_std_football_player -> /robot/joint_states 和 /robot/driver/status
class SimZ01StdFootballPlayer : public sim_common::SimNodeBase {
  public:
  SimZ01StdFootballPlayer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~SimZ01StdFootballPlayer() override = default;

  protected:
    // MuJoCo 控制回调：每个仿真步根据缓存的关节命令写入 d->ctrl。
    void ComputeControl(const mjModel* m, mjData* d) override;
    // 模型加载完成后查找足球相关 MuJoCo 对象并隐藏球门（初始非足球模式）。
    void HandleInterfaceReady() override;
    // 200 Hz 发布块内发布基座标系下的球/球门位置（仅足球模式）。
    void PublishExtra(const mjData* d) override;

  private:
    // 足球相关：发布基座标系下的球/球门位置，订阅自定义球位置/速度/预览/模式。
    int soccer_ball_body_id_ = -1;
    int soccer_ball_joint_qposadr_ = -1;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_point_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_destination_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr set_ball_pos_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr set_ball_vel_sub_;

    // 足球模式：开启时显示球门与球；关闭时隐藏（避免干扰行走）。
    struct GeomState {
      int id = -1;
      float rgba[4] = {1.0f, 1.0f, 1.0f, 1.0f};
      int contype = 1;
      int conaffinity = 1;
    };
    std::vector<GeomState> soccer_geoms_;    // 双方球门（随模式显隐）
    std::vector<GeomState> field_geoms_;     // 草坪/白线/中圈（随模式显隐）
    GeomState ball_geom_state_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr soccer_mode_sub_;
    bool soccer_mode_ = false;

    // 对方球门 body：target_destination 用其门口位置（替代硬编码坐标）。
    int goal_body_id_ = -1;
    double goal_world_pos_[3] = {0.0, 0.0, 0.0};

    // 足球放置预览点（mocap body，放置模式下显示目标位置）。
    int ball_preview_body_id_ = -1;
    int ball_preview_mocap_id_ = -1;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ball_preview_sub_;

    void HandleBallPosition(const geometry_msgs::msg::Point::SharedPtr msg);
    void HandleBallVelocity(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void PublishTargetPoints(const mjData* d);
    void HandleBallPreview(const geometry_msgs::msg::Point::SharedPtr msg);
    void HandleSoccerMode(const std_msgs::msg::Bool::SharedPtr msg);
    void SetSoccerMode(bool enabled);
    void CollectBodyGeoms(const char* body_name, std::vector<GeomState>& out);
    void CollectGeomsByPrefix(const char* prefix, std::vector<GeomState>& out);
    void SetGeomsVisible(const std::vector<GeomState>& geoms, bool visible);
};

#endif
