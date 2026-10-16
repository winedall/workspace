#ifndef SIMZ01EDMPARALLELANKLE_H
#define SIMZ01EDMPARALLELANKLE_H



#include "sim_common/SimNodeBase.hpp"

// SimZ01EdmParallelAnkle 是 Z01_edm_parallel_ankle 模型（并联踝 + 深度相机）的 MuJoCo 仿真节点。
//
// 继承关系说明：
// - sim_common::SimNodeBase（= MujocoSimBase + SimRosNodeBase）：负责
//   MuJoCo 模型加载/窗口/仿真主循环/Run() 生命周期，以及 ROS 2 topic、
//   状态消息、命令缓存、200 Hz 发布节奏、相机图像发布；
// - SimZ01EdmParallelAnkle：保留机器人特有逻辑（电机表、控制律）和
//   足球场景（球/球门显示切换、目标点发布、遥操作回调）。
//
// 该节点在控制链路中扮演"仿真驱动"角色，替代 robot_driver_node：
// robot_control -> /robot/control/hardware_commands -> sim_z01_edm_parallel_ankle
// sim_z01_edm_parallel_ankle -> /robot/joint_states 和 /robot/driver/status
class SimZ01EdmParallelAnkle : public sim_common::SimNodeBase {
  public:
  SimZ01EdmParallelAnkle(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~SimZ01EdmParallelAnkle() override = default;

  protected:
    // MuJoCo 控制回调：每个仿真步根据缓存的关节命令写入 d->ctrl。
    void ComputeControl(const mjModel* m, mjData* d) override;
};

#endif
