#include "robot_common/joint_limits.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace robot_common
{

JointLimits::JointLimits(const RobotJointsConfig & config)
: config_(config)
{
  // 复制后重新校验，确保传入配置即使来自手工构造，也满足安全前置条件。
  validateRobotJointsConfig(config_);
}

double JointLimits::clampPosition(const std::string & joint_name, double value) const
{
  const auto & joint = requireJoint(joint_name);
  // 位置限幅使用配置中的非对称范围，例如某些膝关节只允许单向较大弯曲。
  return std::clamp(value, joint.min_pos, joint.max_pos);
}

double JointLimits::clampVelocity(const std::string & joint_name, double value) const
{
  const auto & joint = requireJoint(joint_name);
  // 速度限制按绝对值对称处理，正负方向共用 max_vel。
  return std::clamp(value, -joint.max_vel, joint.max_vel);
}

double JointLimits::clampEffort(const std::string & joint_name, double value) const
{
  const auto & joint = requireJoint(joint_name);
  // 力矩/推力限制按绝对值对称处理，避免任一方向输出超过配置边界。
  return std::clamp(value, -joint.max_effort, joint.max_effort);
}

bool JointLimits::isPositionValid(const std::string & joint_name, double value) const
{
  const auto & joint = requireJoint(joint_name);
  return value >= joint.min_pos && value <= joint.max_pos;
}

bool JointLimits::isVelocityValid(const std::string & joint_name, double value) const
{
  const auto & joint = requireJoint(joint_name);
  return std::abs(value) <= joint.max_vel;
}

bool JointLimits::isEffortValid(const std::string & joint_name, double value) const
{
  const auto & joint = requireJoint(joint_name);
  return std::abs(value) <= joint.max_effort;
}

const JointConfig & JointLimits::requireJoint(const std::string & joint_name) const
{
  const auto * joint = config_.getJoint(joint_name);
  if (joint == nullptr) {
    // 未知关节必须显式失败；静默返回原值或 0 都可能掩盖上游命名错误。
    throw std::out_of_range("unknown joint '" + joint_name + "'");
  }
  return *joint;
}

}  // namespace robot_common
