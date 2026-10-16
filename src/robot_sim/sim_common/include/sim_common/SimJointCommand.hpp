#pragma once

// 仿真内部使用的单关节命令缓存。
//
// 从 SimRosNodeBase.hpp 抽出来单独放，是为了让 SimErrorModel 也能用它做"指令延迟"：
// SimErrorModel 必须保持与 ROS 无关（只依赖 Eigen + mujoco），而 SimRosNodeBase.hpp
// 会拖进 rclcpp/消息类型。本结构体是纯 POD，不带任何 ROS 类型，两边都能 include。
//
// robot_control 输出的是 robot_interfaces/msg/JointCommand，这里只保留 MuJoCo 控制器
// 真正需要的字段，避免控制循环频繁解析整帧 ROS 消息。

namespace sim_common {

struct SimJointCommand {
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  double kp{0.0};
  double kd{0.0};
  bool enable{false};
};

}  // namespace sim_common
