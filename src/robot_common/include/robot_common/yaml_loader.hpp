#ifndef ROBOT_COMMON__YAML_LOADER_HPP_
#define ROBOT_COMMON__YAML_LOADER_HPP_

#include <string>

#include <yaml-cpp/yaml.h>

namespace robot_common
{

// 读取 YAML 文件并返回 yaml-cpp 节点。
// 该函数只负责文件存在性和文件类型检查，不解释具体业务字段。
YAML::Node loadYamlFile(const std::string & config_path);

// 解析 robot_joints.yaml 路径。
// 传入路径非空时直接返回；传入空路径时从当前目录开始逐级向上查找
// config/robot_joints.yaml，适合 colcon 工作区根目录或包内测试环境使用。
std::string resolveRobotJointsConfigPath(const std::string & config_path = "");

}  // namespace robot_common

#endif  // ROBOT_COMMON__YAML_LOADER_HPP_
