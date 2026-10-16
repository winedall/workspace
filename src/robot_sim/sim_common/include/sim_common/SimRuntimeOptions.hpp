#pragma once

#include <rclcpp/rclcpp.hpp>

#include "mj_sim/MujocoSimBase.h"
#include "sim_common/SimErrorModel.hpp"

namespace sim_common {

// 从 ROS 参数读取两组"运行期选项"，供 SimNodeBase（单机）与 SimZ01StdRobocup（多机）
// 共用：参数名与默认值只在这里定义一处，避免两个节点各写一套 YAML 键名而漂移。
//
// 参数按影响面分组（YAML 里就是嵌套段）：
//   viewer.*   —— 只影响窗口与界面显示；不影响物理推进，也不影响任何话题；
//   dynamics.* —— 只影响物理线程的推进节奏与运行状态；不改变模型本身；
//   errors.*   —— 只影响传感器读数与执行时序；不改变模型本身的物理参数。
// 未在 YAML 里出现的项保持结构体默认值（= 改造前 Simulate 的行为）。
SimViewerOptions DeclareViewerOptions(rclcpp::Node* node);
SimDynamicsOptions DeclareDynamicsOptions(rclcpp::Node* node);
SimErrorOptions DeclareErrorOptions(rclcpp::Node* node);

}  // namespace sim_common
