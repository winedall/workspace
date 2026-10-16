#ifndef ROBOT_COMMON__JOINT_CONFIG_HPP_
#define ROBOT_COMMON__JOINT_CONFIG_HPP_

#include <string>
#include <unordered_map>
#include <vector>

namespace robot_common
{

// 单个逻辑关节的控制配置。
//
// 这里保存的是上层控制逻辑需要理解的“机器人关节”信息，而不是硬件驱动
// 的 motor_id、slave_id、通道号等厂家/总线字段。硬件映射应由 robot_driver
// 读取 hw_encos.yaml 后单独处理。
struct JointConfig
{
  // 关节唯一名称。必须与 robot_interfaces 中 JointCommand.joint_name /
  // JointState.joint_name 使用同一套命名。
  std::string name;

  // 逻辑分组，例如 head、left_arm、right_leg。状态机和控制器可按分组
  // 判断关键关节集合。
  std::string group;

  // 物理单位。旋转关节通常为 rad，直线关节通常为 m。
  std::string unit;

  // 启动或默认控制模式下是否默认参与控制。
  bool enable_default{false};

  // 位置上下限。旋转关节单位为 rad，直线关节单位为 m。
  double min_pos{0.0};
  double max_pos{0.0};

  // 最大速度绝对值。旋转关节单位为 rad/s，直线关节单位为 m/s。
  double max_vel{0.0};

  // 最大力矩/推力绝对值。旋转关节单位为 Nm，直线关节单位为 N。
  double max_effort{0.0};

};

// robot_joints.yaml 加载后的完整配置。
//
// joints 保留 YAML 中的关节顺序，便于 RL action 向量、全身关节命令等模块
// 使用稳定顺序；joint_map 用于按 joint_name 快速查询。
struct RobotJointsConfig
{
  std::vector<JointConfig> joints;
  std::unordered_map<std::string, JointConfig> joint_map;

  // 判断关节名是否存在于配置白名单中。
  bool hasJoint(const std::string & name) const;

  // 查询关节配置；未知关节返回 nullptr，调用方可据此拒绝命令或降级处理。
  const JointConfig * getJoint(const std::string & name) const;

  // 按 joints 原始顺序返回关节名列表。
  std::vector<std::string> jointNames() const;
};

// 加载并校验 robot_joints.yaml。
//
// config_path 为空时，会从当前工作目录向父目录查找 config/robot_joints.yaml；
// 非空时使用调用方传入路径。加载失败或配置非法时抛出 std::runtime_error。
RobotJointsConfig loadRobotJointsConfig(const std::string & config_path = "");

// 校验配置并重建 joint_map。
//
// 该函数会检查关节列表非空、关节名唯一和限幅合法等基础安全条件。
void validateRobotJointsConfig(RobotJointsConfig & config);

}  // namespace robot_common

#endif  // ROBOT_COMMON__JOINT_CONFIG_HPP_
