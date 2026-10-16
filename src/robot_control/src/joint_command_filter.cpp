#include "robot_control/joint_command_filter.hpp"

#include <algorithm>
#include <cmath>

namespace robot_control
{

JointCommandFilter::JointCommandFilter(const std::string & robot_joints_config)
: joints_config_(robot_common::loadRobotJointsConfig(robot_joints_config))
{
  // 构造阶段加载 robot_joints.yaml。配置非法时由 robot_common 抛出异常，
  // 节点启动失败比带着错误限幅继续运行更安全。
}

JointCommandFilterResult JointCommandFilter::filter(
  const robot_interfaces::msg::JointCommands & commands)
{
  // 保存原始输入，只用于调试和统计。是否能下发以后续 accepted 结果为准。
  last_received_joint_commands_ = commands;

  // 空帧没有明确控制语义，直接拒绝，避免下游误认为这是有效的安全停止命令。
  if (commands.commands.empty()) {
    return {false, "empty joint command frame", commands};
  }

  // 保留原始 header/sequence/source/timeout，只替换 commands 数组中的逐关节内容。
  robot_interfaces::msg::JointCommands filtered = commands;
  filtered.commands.clear();
  filtered.commands.reserve(commands.commands.size());

  for (const auto & command : commands.commands) {
    // robot_control 是进入 driver 前的最后一道关节白名单检查，未知关节整帧拒绝。
    const auto * joint_config = joints_config_.getJoint(command.joint_name);
    if (joint_config == nullptr) {
      return {false, "unknown joint name", commands};
    }
    // NaN/Inf 会污染控制计算和硬件输出，必须在限幅前拒绝。
    if (!isFiniteCommand(command)) {
      return {false, "non-finite joint command", commands};
    }
    filtered.commands.push_back(filterOne(command, *joint_config));
  }

  return {true, "joint commands filtered", filtered};
}

// 获取最近收到的关节命令。
const robot_interfaces::msg::JointCommands & JointCommandFilter::lastReceivedJointCommands() const
{
  return last_received_joint_commands_;
}

// 获取关节配置。
const robot_common::RobotJointsConfig & JointCommandFilter::jointsConfig() const
{
  return joints_config_;
}

// 对单个已知关节执行限幅。
robot_interfaces::msg::JointCommand JointCommandFilter::filterOne(
  const robot_interfaces::msg::JointCommand & command,
  const robot_common::JointConfig & joint_config) const
{
  auto filtered = command;

  // 物理边界来自 robot_joints.yaml；这里做最终限幅，command_router 只负责准入。
  filtered.position = static_cast<float>(std::clamp<double>(command.position, joint_config.min_pos, joint_config.max_pos));
  filtered.velocity = static_cast<float>(std::clamp<double>(command.velocity, -joint_config.max_vel, joint_config.max_vel));
  filtered.effort = static_cast<float>(std::clamp<double>(command.effort, -joint_config.max_effort, joint_config.max_effort));

  // kp/kd 和 enable 原样透传，由上游模型/运控策略决定。
  filtered.kp = command.kp;
  filtered.kd = command.kd;
  filtered.enable = command.enable;
  return filtered;
}

// 检查所有浮点字段是否有限，防止 NaN/Inf 进入控制链路。
bool JointCommandFilter::isFiniteCommand(
  const robot_interfaces::msg::JointCommand & command) const
{
  return std::isfinite(command.position) &&
    std::isfinite(command.velocity) &&
    std::isfinite(command.effort) &&
    std::isfinite(command.kp) &&
    std::isfinite(command.kd);
}

}  // namespace robot_control
