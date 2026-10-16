#include "robot_common/yaml_loader.hpp"

#include <filesystem>
#include <stdexcept>
#include <string>

namespace robot_common
{

YAML::Node loadYamlFile(const std::string & config_path)
{
  // 文件系统检查先于 YAML::LoadFile，错误信息会更直接，便于 launch 或测试定位。
  const auto path = std::filesystem::path(config_path);
  if (config_path.empty()) {
    throw std::runtime_error("YAML config path cannot be empty");
  }
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("YAML config file does not exist: " + path.string());
  }
  if (!std::filesystem::is_regular_file(path)) {
    throw std::runtime_error("YAML config path is not a file: " + path.string());
  }

  // 具体 YAML 语法和字段类型错误由 yaml-cpp 抛出；上层根据异常拒绝启动。
  return YAML::LoadFile(path.string());
}

std::string resolveRobotJointsConfigPath(const std::string & config_path)
{
  if (!config_path.empty()) {
    // 调用方显式传入路径时不做额外搜索，便于 launch 参数覆盖默认配置。
    return config_path;
  }

  // 默认从当前工作目录向上查找 colcon 工作区根目录下的 config/robot_joints.yaml。
  // 这样无论节点从工作区根目录、包目录还是测试进程启动，都有机会找到根配置。
  auto current = std::filesystem::current_path();
  while (true) {
    const auto candidate = current / "config" / "robot_joints.yaml";
    if (std::filesystem::exists(candidate)) {
      return candidate.string();
    }

    if (!current.has_parent_path() || current == current.parent_path()) {
      break;
    }
    current = current.parent_path();
  }

  throw std::runtime_error(
    "Unable to find config/robot_joints.yaml from current directory or parent directories");
}

}  // namespace robot_common
