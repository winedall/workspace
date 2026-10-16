#ifndef ROBOT_COMMON__JOINT_LIMITS_HPP_
#define ROBOT_COMMON__JOINT_LIMITS_HPP_

#include <string>

#include "robot_common/joint_config.hpp"

namespace robot_common
{

// 基于 RobotJointsConfig 的关节安全限幅工具。
//
// 该类只做纯计算和白名单查询，不发布 ROS topic，也不接触硬件。所有函数
// 对未知 joint_name 都会抛出 std::out_of_range，避免调用方误把未知关节
// 当作“无需限幅”的合法命令继续下发。
class JointLimits
{
public:
  // 构造时复制配置，并重新执行一次校验，确保 joint_map 与 joints 一致。
  explicit JointLimits(const RobotJointsConfig & config);

  // 将位置限制在 [min_pos, max_pos] 范围内。
  double clampPosition(const std::string & joint_name, double value) const;

  // 将速度限制在 [-max_vel, max_vel] 范围内。
  double clampVelocity(const std::string & joint_name, double value) const;

  // 将力矩/推力限制在 [-max_effort, max_effort] 范围内。
  double clampEffort(const std::string & joint_name, double value) const;

  // 检查位置是否位于 [min_pos, max_pos] 范围内。
  bool isPositionValid(const std::string & joint_name, double value) const;

  // 检查速度绝对值是否不超过 max_vel。
  bool isVelocityValid(const std::string & joint_name, double value) const;

  // 检查力矩/推力绝对值是否不超过 max_effort。
  bool isEffortValid(const std::string & joint_name, double value) const;

private:
  // 获取关节配置；未知关节直接抛错，避免控制链路静默放行。
  const JointConfig & requireJoint(const std::string & joint_name) const;

  RobotJointsConfig config_;
};

}  // namespace robot_common

#endif  // ROBOT_COMMON__JOINT_LIMITS_HPP_
