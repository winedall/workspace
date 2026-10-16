#include "robot_common/joint_config.hpp"

#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include "robot_common/yaml_loader.hpp"

namespace robot_common
{

namespace
{

// 从 YAML 节点中读取必填标量字段。
//
// 这里集中处理“字段缺失”错误，让 parseJointConfig 保持清晰；类型转换错误
// 由 yaml-cpp 抛出，调用方最终都会看到加载失败。
template<typename T>
T requireScalar(const YAML::Node & node, const std::string & key, const std::string & joint_name)
{
  if (!node[key]) {
    throw std::runtime_error("Joint '" + joint_name + "' is missing required field '" + key + "'");
  }
  return node[key].as<T>();
}

// 将单个 joints[] 条目转换为 JointConfig。
//
// 每个关节必须显式给出全部逻辑字段；物理限幅不使用全局 defaults 兜底。
JointConfig parseJointConfig(const YAML::Node & node)
{
  const auto name = requireScalar<std::string>(node, "name", "<unknown>");
  JointConfig joint;
  joint.name = name;
  joint.group = requireScalar<std::string>(node, "group", name);
  joint.enable_default = requireScalar<bool>(node, "enable_default", name);
  joint.unit = requireScalar<std::string>(node, "unit", name);
  joint.min_pos = requireScalar<double>(node, "min_pos", name);
  joint.max_pos = requireScalar<double>(node, "max_pos", name);
  joint.max_vel = requireScalar<double>(node, "max_vel", name);
  joint.max_effort = requireScalar<double>(node, "max_effort", name);
  return joint;
}

// 兼容两种常见 YAML 结构：
//   joints: [...]
//   robot_joints:
//     joints: [...]
//
// 这样既能读取根目录 config/robot_joints.yaml，也能兼容后续按命名空间组织
// 的配置文件。
YAML::Node jointListNode(const YAML::Node & root)
{
  if (root["joints"]) {
    return root["joints"];
  }
  if (root["robot_joints"] && root["robot_joints"]["joints"]) {
    return root["robot_joints"]["joints"];
  }
  throw std::runtime_error("robot_joints config must contain a 'joints' list");
}

}  // namespace

bool RobotJointsConfig::hasJoint(const std::string & name) const
{
  // joint_map 在 validateRobotJointsConfig 中构建，查询复杂度为 O(1)。
  return joint_map.find(name) != joint_map.end();
}

const JointConfig * RobotJointsConfig::getJoint(const std::string & name) const
{
  const auto iter = joint_map.find(name);
  if (iter == joint_map.end()) {
    return nullptr;
  }
  return &iter->second;
}

std::vector<std::string> RobotJointsConfig::jointNames() const
{
  // 返回 joints 的原始顺序，而不是 unordered_map 的哈希顺序。
  std::vector<std::string> names;
  names.reserve(joints.size());
  for (const auto & joint : joints) {
    names.push_back(joint.name);
  }
  return names;
}

RobotJointsConfig loadRobotJointsConfig(const std::string & config_path)
{
  // 先解析路径，再读取 YAML，最后做业务校验。路径解析和 YAML 读取放在
  // yaml_loader 中，便于后续节点参数、launch 参数复用。
  const auto resolved_path = resolveRobotJointsConfigPath(config_path);
  const auto root = loadYamlFile(resolved_path);
  const auto joints_node = jointListNode(root);
  if (!joints_node.IsSequence()) {
    throw std::runtime_error("robot_joints config field 'joints' must be a list");
  }

  RobotJointsConfig config;
  for (const auto & joint_node : joints_node) {
    config.joints.push_back(parseJointConfig(joint_node));
  }
  validateRobotJointsConfig(config);
  return config;
}

void validateRobotJointsConfig(RobotJointsConfig & input_config)
{
  // 关节配置是控制命令白名单和安全限幅的基础。配置非法时应尽早拒绝启动，
  // 而不是在控制回调中降级猜测。
  if (input_config.joints.empty()) {
    throw std::runtime_error("robot_joints config must contain at least one joint");
  }

  std::unordered_set<std::string> names;

  for (const auto & joint : input_config.joints) {
    // name/group/unit 是最小语义字段：name 用于跨模块匹配，group 用于状态机
    // 和控制策略分组，unit 用于解释位置/速度/力矩字段。
    if (joint.name.empty()) {
      throw std::runtime_error("joint name cannot be empty");
    }
    if (!names.insert(joint.name).second) {
      throw std::runtime_error("duplicate joint name '" + joint.name + "'");
    }
    if (joint.group.empty()) {
      throw std::runtime_error("joint '" + joint.name + "' group cannot be empty");
    }
    if (joint.unit.empty()) {
      throw std::runtime_error("joint '" + joint.name + "' unit cannot be empty");
    }

    // 位置范围、速度和力矩限制都必须是明确的正向安全约束。
    if (!(joint.min_pos < joint.max_pos)) {
      throw std::runtime_error("joint '" + joint.name + "' min_pos must be less than max_pos");
    }
    if (!(joint.max_vel > 0.0)) {
      throw std::runtime_error("joint '" + joint.name + "' max_vel must be greater than zero");
    }
    if (!(joint.max_effort > 0.0)) {
      throw std::runtime_error("joint '" + joint.name + "' max_effort must be greater than zero");
    }
  }

  // 校验全部通过后再重建 joint_map，避免部分更新后的配置被调用方使用。
  input_config.joint_map.clear();
  for (const auto & joint : input_config.joints) {
    input_config.joint_map.emplace(joint.name, joint);
  }
}

}  // namespace robot_common
