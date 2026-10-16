#pragma once

#include "sim_common/SimNodeBase.hpp"

namespace sim_d040_poc {

// SimD040Poc 是 D040（D4）四足模型的 MuJoCo 仿真节点。
//
// 继承关系说明：
// - sim_common::SimNodeBase（= MujocoSimBase + SimRosNodeBase）：负责
//   MuJoCo 模型加载/窗口/仿真主循环/Run() 生命周期，以及 ROS 2 topic、
//   状态消息、命令缓存、200 Hz 发布节奏；
// - SimD040Poc：仅保留机器人特有逻辑（初始位姿、电机表、控制律）。
//
// 该节点在控制链路中扮演“仿真驱动”角色，替代 robot_driver_node：
// robot_control -> /robot/control/hardware_commands -> sim_d040_poc
// sim_d040_poc -> /robot/joint_states 和 /robot/driver/status
class SimD040Poc : public sim_common::SimNodeBase {
 public:
  explicit SimD040Poc(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~SimD040Poc() override = default;

 protected:
  void ComputeControl(const mjModel* model, mjData* data) override;
};

}  // namespace sim_d040_poc
