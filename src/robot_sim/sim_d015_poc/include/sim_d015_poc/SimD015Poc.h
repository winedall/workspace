#pragma once

#include "sim_common/SimNodeBase.hpp"

namespace sim_d015_poc {

// SimD015Poc 是 D015（D2）四足模型的 MuJoCo 仿真节点。
//
// 继承关系说明：
// - sim_common::SimNodeBase（= MujocoSimBase + SimRosNodeBase）：负责
//   MuJoCo 模型加载/窗口/仿真主循环/Run() 生命周期，以及 ROS 2 topic、
//   状态消息、命令缓存、200 Hz 发布节奏；
// - SimD015Poc：仅保留机器人特有逻辑（初始位姿、armature 表、电机表、
//   控制律）。
//
// 该节点在控制链路中扮演“仿真驱动”角色，替代 robot_driver_node：
// robot_control -> /robot/control/hardware_commands -> sim_d015_poc
// sim_d015_poc -> /robot/joint_states 和 /robot/driver/status
class SimD015Poc : public sim_common::SimNodeBase {
 public:
  explicit SimD015Poc(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~SimD015Poc() override = default;

 protected:
  void ComputeControl(const mjModel* model, mjData* data) override;
};

}  // namespace sim_d015_poc
