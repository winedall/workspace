#ifndef ROBOT_CONTROL__JOINT_COMMAND_FILTER_HPP_
#define ROBOT_CONTROL__JOINT_COMMAND_FILTER_HPP_

#include <string>

#include "robot_common/joint_config.hpp"
#include "robot_interfaces/msg/joint_commands.hpp"

namespace robot_control
{

// 单次过滤结果。accepted=false 时 commands 保留原始命令，方便发布 rejected topic。
struct JointCommandFilterResult
{
  bool accepted{false};
  std::string reason;
  robot_interfaces::msg::JointCommands commands;
};

// 关节命令过滤器。
//
// 该类负责白名单、位置/速度/力矩限幅和有限数检查。
// 它不做跳变平滑，也不发布 ROS topic；因此可以在 gtest 中直接测试。
class JointCommandFilter
{
public:
  // robot_joints_config 为空时由 robot_common 按默认规则查找 config/robot_joints.yaml。
  explicit JointCommandFilter(const std::string & robot_joints_config);

  // 对一帧 JointCommands 做逐关节安全过滤。
  // 未知关节、空命令帧或 NaN/Inf 命令会整体拒绝，避免半帧命令继续下发。
  JointCommandFilterResult filter(const robot_interfaces::msg::JointCommands & commands);

  // 最近收到的原始命令，仅用于调试、统计或诊断，不代表已经通过安全检查。
  const robot_interfaces::msg::JointCommands & lastReceivedJointCommands() const;

  // 暴露只读关节配置，便于节点或测试确认过滤器加载的关节白名单。
  const robot_common::RobotJointsConfig & jointsConfig() const;

private:
  // 对单个已知关节执行限幅。
  robot_interfaces::msg::JointCommand filterOne(
    const robot_interfaces::msg::JointCommand & command,
    const robot_common::JointConfig & joint_config) const;

  // 检查所有浮点字段是否有限，防止 NaN/Inf 进入控制链路。
  bool isFiniteCommand(const robot_interfaces::msg::JointCommand & command) const;

  // 关节逻辑配置，包含白名单、限幅和 enable_default。
  robot_common::RobotJointsConfig joints_config_;

  // 最近收到的命令缓存，不用于安全输出决策。
  robot_interfaces::msg::JointCommands last_received_joint_commands_;
};

}  // namespace robot_control

#endif  // ROBOT_CONTROL__JOINT_COMMAND_FILTER_HPP_
