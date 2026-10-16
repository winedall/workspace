#ifndef SIMZ01EDM_H
#define SIMZ01EDM_H

#include "sim_common/SimNodeBase.hpp"

// SimZ01Edm 是 Z01_edm 标准模型的 MuJoCo 仿真节点。
//
// 继承关系说明：
// - sim_common::SimNodeBase（= MujocoSimBase + SimRosNodeBase）：负责
//   MuJoCo 模型加载/窗口/仿真主循环/Run() 生命周期，以及 ROS 2 topic、
//   状态消息、命令缓存、200 Hz 发布节奏；
// - SimZ01Edm：仅保留机器人特有逻辑（初始位姿参数、电机表、控制律）。
//
// 该节点在控制链路中扮演“仿真驱动”角色，替代 robot_driver_node：
// robot_control -> /robot/control/hardware_commands -> sim_z01_edm
// sim_z01_edm -> /robot/joint_states 和 /robot/driver/status
class SimZ01Edm : public sim_common::SimNodeBase {
  public:
    SimZ01Edm(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~SimZ01Edm() override = default;

  protected:
    // MuJoCo 控制回调：每个仿真步根据缓存的关节命令写入 d->ctrl。
    void ComputeControl(const mjModel* m, mjData* d) override;
};

#endif
