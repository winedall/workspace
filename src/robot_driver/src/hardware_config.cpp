#include "robot_driver/hardware_config.hpp"

#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace robot_driver
{

namespace
{

// 读取 YAML 必填字段。
//
// 缺失字段会直接抛出异常，避免硬件通道、ID、方向等关键配置静默使用默认值。
template<typename T>
T requireField(const YAML::Node & node, const std::string & field, const std::string & joint_name)
{
  // 硬件映射缺字段时直接拒绝启动，避免真实 backend 用默认 0 写到错误通道。
  if (!node[field]) {
    throw std::runtime_error("hardware joint '" + joint_name + "' is missing required field '" + field + "'");
  }
  return node[field].as<T>();
}

// 读取 YAML 可选字段。
//
// 字段不存在时返回调用方提供的默认值，用于 cycle_time、feedback_type 等可选配置。
template<typename T>
T optionalField(const YAML::Node & node, const std::string & field, const T & default_value)
{
  if (!node[field]) {
    return default_value;
  }
  return node[field].as<T>();
}

// 读取 YAML 可选字符串字段。
std::string optionalString(const YAML::Node & node, const std::string & field)
{
  return optionalField<std::string>(node, field, "");
}

// 校验数值范围配置必须满足 min < max。
//
// Encos 协议的量化/反量化依赖这些范围，配置错误会直接影响运动命令打包。
void validateRange(
  double min_value,
  double max_value,
  const std::string & min_field,
  const std::string & max_field,
  const std::string & joint_name)
{
  if (min_value >= max_value) {
    throw std::runtime_error(
      "hardware joint '" + joint_name + "' requires " + min_field + " < " + max_field);
  }
}

// 解析单个 Encos 关节的厂家扩展配置。
//
// 该函数只处理 encos: 子块，并校验量化范围、kt、slave/passage 等 Encos 必需字段。
void parseEncosJoint(const YAML::Node & node, HardwareJointConfig & joint, bool require_gateway_fields)
{
  const auto encos = node["encos"];
  if (!encos || !encos.IsMap()) {
    throw std::runtime_error("hardware joint '" + joint.name + "' requires encos: mapping");
  }
  joint.encos.slave_id = require_gateway_fields ? requireField<int>(encos, "slave_id", joint.name) : 0;
  joint.encos.passage = require_gateway_fields ? requireField<int>(encos, "passage", joint.name) : 0;
  joint.encos.kt = requireField<double>(encos, "kt", joint.name);
  joint.encos.kp_min = requireField<double>(encos, "kp_min", joint.name);
  joint.encos.kp_max = requireField<double>(encos, "kp_max", joint.name);
  joint.encos.kd_min = requireField<double>(encos, "kd_min", joint.name);
  joint.encos.kd_max = requireField<double>(encos, "kd_max", joint.name);
  joint.encos.pos_min = requireField<double>(encos, "pos_min", joint.name);
  joint.encos.pos_max = requireField<double>(encos, "pos_max", joint.name);
  joint.encos.spd_min = requireField<double>(encos, "spd_min", joint.name);
  joint.encos.spd_max = requireField<double>(encos, "spd_max", joint.name);
  joint.encos.tor_min = requireField<double>(encos, "tor_min", joint.name);
  joint.encos.tor_max = requireField<double>(encos, "tor_max", joint.name);
  joint.encos.cur_min = requireField<double>(encos, "cur_min", joint.name);
  joint.encos.cur_max = requireField<double>(encos, "cur_max", joint.name);

  if (require_gateway_fields && joint.encos.slave_id < 0) {
    throw std::runtime_error("hardware joint '" + joint.name + "' encos.slave_id must be >= 0");
  }
  if (require_gateway_fields && joint.encos.passage <= 0) {
    throw std::runtime_error("hardware joint '" + joint.name + "' encos.passage must be > 0");
  }
  if (joint.encos.kt <= 0.0) {
    throw std::runtime_error("hardware joint '" + joint.name + "' encos.kt must be > 0");
  }

  validateRange(joint.encos.kp_min, joint.encos.kp_max, "encos.kp_min", "encos.kp_max", joint.name);
  validateRange(joint.encos.kd_min, joint.encos.kd_max, "encos.kd_min", "encos.kd_max", joint.name);
  validateRange(joint.encos.pos_min, joint.encos.pos_max, "encos.pos_min", "encos.pos_max", joint.name);
  validateRange(joint.encos.spd_min, joint.encos.spd_max, "encos.spd_min", "encos.spd_max", joint.name);
  validateRange(joint.encos.tor_min, joint.encos.tor_max, "encos.tor_min", "encos.tor_max", joint.name);
  validateRange(joint.encos.cur_min, joint.encos.cur_max, "encos.cur_min", "encos.cur_max", joint.name);
}

// 解析单个 Tienkung 关节的厂家扩展配置。
//
// 天工关节需要落到 EtherCAT 网关的 slave_id、CAN-FD bus 和 slot 上。
void parseTienkungJoint(const YAML::Node & node, HardwareJointConfig & joint, bool require_gateway_fields)
{
  const auto tienkung = node["tienkung"];
  if (!tienkung || !tienkung.IsMap()) {
    throw std::runtime_error("hardware joint '" + joint.name + "' requires tienkung: mapping");
  }
  joint.tienkung.slave_id = require_gateway_fields ? requireField<int>(tienkung, "slave_id", joint.name) : 0;
  joint.tienkung.bus = require_gateway_fields ? requireField<int>(tienkung, "bus", joint.name) : 0;
  joint.tienkung.slot = require_gateway_fields ? requireField<int>(tienkung, "slot", joint.name) : 0;
  joint.tienkung.can_mode = optionalField<std::string>(tienkung, "can_mode", "canfd");
  joint.tienkung.feedback_type = optionalField<std::string>(tienkung, "feedback_type", "default");
  joint.tienkung.kt = optionalField<double>(tienkung, "kt", 0.0);

  if (require_gateway_fields && joint.tienkung.slave_id < 0) {
    throw std::runtime_error("hardware joint '" + joint.name + "' tienkung.slave_id must be >= 0");
  }
  if (require_gateway_fields && (joint.tienkung.bus < 0 || joint.tienkung.bus > 1)) {
    throw std::runtime_error("hardware joint '" + joint.name + "' tienkung.bus must be 0 or 1");
  }
  if (require_gateway_fields && (joint.tienkung.slot < 0 || joint.tienkung.slot > 6)) {
    throw std::runtime_error("hardware joint '" + joint.name + "' tienkung.slot must be in [0, 6]");
  }
  if (joint.tienkung.can_mode != "canfd") {
    throw std::runtime_error("hardware joint '" + joint.name + "' tienkung.can_mode must be 'canfd'");
  }
  if (joint.tienkung.feedback_type != "default") {
    throw std::runtime_error("hardware joint '" + joint.name + "' tienkung.feedback_type must be 'default'");
  }
  if (tienkung["kt"] && joint.tienkung.kt <= 0.0) {
    throw std::runtime_error("hardware joint '" + joint.name + "' tienkung.kt must be > 0 when configured");
  }
}

// 解析单个关节的 CANFD 总线定位字段。
//
// mix/canfd 模式下，具体厂家仍由 vendor 决定，canfd 子块只描述 SocketCAN 总线号。
void parseCanfdJoint(const YAML::Node & node, HardwareJointConfig & joint)
{
  // mix/canfd 模式下，关节不再通过 EtherCAT slave/slot 定位，而是通过 canfd.bus
  // 选择实际 SocketCAN 通道；motor_id 仍作为线上 CAN id 使用。
  const auto canfd = node["canfd"];
  if (!canfd || !canfd.IsMap()) {
    throw std::runtime_error("hardware joint '" + joint.name + "' requires canfd: mapping");
  }
  joint.canfd.bus = requireField<int>(canfd, "bus", joint.name);
  if (joint.canfd.bus < 0) {
    throw std::runtime_error("hardware joint '" + joint.name + "' canfd.bus must be >= 0");
  }
}

// 解析单个关节的 EtherCAT-CANFD 网关槽位字段。
//
// gateway 是中性拓扑字段，既可承载 Encos 帧，也可承载 Tienkung 帧；厂家协议参数
// 仍放在 encos/tienkung 子块中，避免把通道位置和厂家协议耦合在一起。
void parseGatewayJoint(const YAML::Node & node, HardwareJointConfig & joint)
{
  const auto gateway = node["gateway"];
  if (!gateway || !gateway.IsMap()) {
    throw std::runtime_error("hardware joint '" + joint.name + "' requires gateway: mapping");
  }
  joint.gateway.slave_id = requireField<int>(gateway, "slave_id", joint.name);
  joint.gateway.bus = requireField<int>(gateway, "bus", joint.name);
  joint.gateway.slot = requireField<int>(gateway, "slot", joint.name);

  if (joint.gateway.slave_id <= 0) {
    throw std::runtime_error("hardware joint '" + joint.name + "' gateway.slave_id must be >= 1");
  }
  if (joint.gateway.bus < 0 || joint.gateway.bus > 1) {
    throw std::runtime_error("hardware joint '" + joint.name + "' gateway.bus must be 0 or 1");
  }
  if (joint.gateway.slot < 0 || joint.gateway.slot > 6) {
    throw std::runtime_error("hardware joint '" + joint.name + "' gateway.slot must be in [0, 6]");
  }
}

// 解析单个硬件关节配置。
//
// 先解析通用字段，再根据 bus_type/vendor 分发到 CANFD、Encos 或 Tienkung 扩展解析。
HardwareJointConfig parseJoint(
  const YAML::Node & node,
  const std::string & vendor_type,
  const std::string & bus_type)
{
  // 先读取 name，这样后续错误信息可以定位到具体关节。
  const auto name = requireField<std::string>(node, "name", "<unknown>");
  HardwareJointConfig joint;
  joint.name = name;
  joint.vendor = optionalField<std::string>(node, "vendor", vendor_type);
  joint.motor_id = requireField<int>(node, "motor_id", name);
  joint.direction = requireField<int>(node, "direction", name);
  joint.offset = requireField<double>(node, "offset", name);
  joint.ratio = optionalField<double>(node, "ratio", 1.0);
  joint.motor_type = optionalString(node, "motor_type");

  if (joint.name.empty()) {
    throw std::runtime_error("hardware joint name cannot be empty");
  }
  // 通用字段只做基础合法性检查；更细的协议范围由具体厂商 backend 根据硬件手册继续约束。
  if (joint.motor_id <= 0) {
    throw std::runtime_error("hardware joint '" + name + "' motor_id must be > 0");
  }
  if (joint.direction != 1 && joint.direction != -1) {
    throw std::runtime_error("hardware joint '" + name + "' direction must be 1 or -1");
  }
  if (joint.ratio <= 0.0) {
    throw std::runtime_error("hardware joint '" + name + "' ratio must be > 0");
  }

  if (bus_type == "canfd") {
    // CANFD 拓扑字段是总线层字段，厂商字段仍分别放在 encos/tienkung 子块中。
    parseCanfdJoint(node, joint);
  } else if (bus_type == "ethercat" && vendor_type == "mix") {
    // mix/ethercat 通过中性 gateway 字段定位 EtherCAT-CANFD 槽位。
    parseGatewayJoint(node, joint);
  }

  // mix 模式允许每个关节选择厂家协议；单厂家模式则默认等于 hardware.vendor_type。
  const auto joint_vendor = joint.vendor.empty() ? vendor_type : joint.vendor;
  if (joint_vendor == "encos") {
    parseEncosJoint(node, joint, vendor_type != "mix");
  } else if (joint_vendor == "tienkung") {
    parseTienkungJoint(node, joint, vendor_type != "mix");
  } else if (vendor_type == "mix") {
    throw std::runtime_error("hardware joint '" + joint.name + "' vendor must be 'encos' or 'tienkung'");
  }
  return joint;
}

}  // namespace

// 判断硬件配置中是否存在指定逻辑关节。
bool HardwareConfig::hasJoint(const std::string & name) const
{
  // 使用 joint_map 做 O(1) 查询，供 factory 校验 robot_joints.yaml 与硬件映射一致。
  return joint_map.find(name) != joint_map.end();
}

// 查询指定逻辑关节的硬件映射。
//
// 返回 nullptr 表示配置中不存在该关节，由调用方决定是忽略还是报错。
const HardwareJointConfig * HardwareConfig::getJoint(const std::string & name) const
{
  const auto iter = joint_map.find(name);
  if (iter == joint_map.end()) {
    return nullptr;
  }
  return &iter->second;
}

// 返回硬件映射中的全部关节名。
//
// 顺序保持 YAML 中 hardware.joints 的配置顺序，便于诊断和测试稳定。
std::vector<std::string> HardwareConfig::jointNames() const
{
  // 返回顺序与 YAML 中 hardware.joints 一致，便于日志和测试稳定。
  std::vector<std::string> names;
  names.reserve(joints.size());
  for (const auto & joint : joints) {
    names.push_back(joint.name);
  }
  return names;
}

// 加载并校验 hardware YAML。
//
// 该函数只解析静态配置，不打开 EtherCAT/CANFD/串口设备；通讯初始化由 backend 完成。
HardwareConfig loadHardwareConfig(const std::string & path)
{
  // 路径为空通常表示 launch/参数配置错误，应在 driver 启动时暴露，而不是降级。
  if (path.empty()) {
    throw std::runtime_error("hardware_config path is empty");
  }

  const auto root = YAML::LoadFile(path);
  // 当前约定根节点为 hardware，与现有 config/hw_encos.yaml 保持一致。
  const auto hardware = root["hardware"];
  if (!hardware) {
    throw std::runtime_error("hardware config missing root key 'hardware'");
  }

  // 解析 vendor_type 和 bus_type
  HardwareConfig config;
  config.vendor_type = optionalString(hardware, "vendor_type");
  config.bus_type = optionalString(hardware, "bus_type");

  // bus_type 不能为空
  if (config.bus_type != "ethercat" && config.bus_type != "serial" && config.bus_type != "canfd") {
    throw std::runtime_error("hardware.bus_type must be 'ethercat', 'serial', or 'canfd'");
  }

  // 解析 EtherCAT 总线级配置
  const auto ethercat = hardware["ethercat"];
  if (ethercat) {
    config.ethercat.network_interface = optionalString(ethercat, "network_interface");
    config.ethercat.cycle_time_ms = optionalField<double>(ethercat, "cycle_time_ms", 0.0);
    config.ethercat.slave_count = optionalField<int>(ethercat, "slave_count", 0);
  }
  if (config.bus_type == "canfd" && ethercat) {
    throw std::runtime_error("hardware.ethercat must not be set when bus_type is canfd");
  }
  if (config.ethercat.cycle_time_ms < 0.0) {
    throw std::runtime_error("hardware.ethercat.cycle_time_ms must be >= 0");
  }
  if (config.bus_type == "ethercat" && config.ethercat.slave_count <= 0) {
    throw std::runtime_error(
      "hardware.ethercat.slave_count must be > 0 when bus_type is ethercat; "
      "set it to the number of physical EtherCAT sub-boards");
  }

  // 解析串口总线级配置
  const auto serial = hardware["serial"];
  if (serial) {
    config.serial.device = optionalString(serial, "device");
    config.serial.baudrate = optionalField<int>(serial, "baudrate", 0);
  }
  if (config.bus_type == "serial" && config.serial.baudrate < 0) {
    throw std::runtime_error("hardware.serial.baudrate must be >= 0");
  }

  // 解析 CANFD 总线配置。它只在 bus_type=canfd 时强制要求存在。
  const auto canfd = hardware["canfd"];
  if (canfd) {
    config.canfd.cycle_time_ms = optionalField<double>(canfd, "cycle_time_ms", 0.0);
    config.canfd.poll_timeout_ms = optionalField<double>(canfd, "poll_timeout_ms", 0.0);
    const auto interfaces = canfd["interfaces"];
    if (interfaces) {
      if (!interfaces.IsSequence()) {
        throw std::runtime_error("hardware.canfd.interfaces must be a sequence");
      }
      std::unordered_set<int> used_buses;
      for (const auto & item : interfaces) {
        CanfdInterfaceConfig iface;
        iface.bus = requireField<int>(item, "bus", "canfd.interfaces");
        iface.name = requireField<std::string>(item, "name", "canfd.interfaces");
        if (iface.bus < 0) {
          throw std::runtime_error("hardware.canfd.interfaces bus must be >= 0");
        }
        if (iface.name.empty()) {
          throw std::runtime_error("hardware.canfd.interfaces name cannot be empty");
        }
        if (!used_buses.insert(iface.bus).second) {
          throw std::runtime_error("duplicate hardware.canfd.interfaces bus: " + std::to_string(iface.bus));
        }
        config.canfd.interfaces.push_back(iface);
      }
    }
  }
  if (config.bus_type == "canfd") {
    // 当前新增 CANFD 通讯机制只支持混装后端，避免和旧 Encos/Tienkung EtherCAT 配置混用。
    if (!canfd) {
      throw std::runtime_error("hardware.canfd is required when bus_type is canfd");
    }
    if (config.vendor_type != "mix") {
      throw std::runtime_error("hardware.vendor_type must be 'mix' when bus_type is canfd");
    }
    if (config.canfd.cycle_time_ms < 0.0) {
      throw std::runtime_error("hardware.canfd.cycle_time_ms must be >= 0");
    }
    if (config.canfd.poll_timeout_ms < 0) {
      throw std::runtime_error("hardware.canfd.poll_timeout_ms must be >= 0");
    }
    if (config.canfd.interfaces.empty()) {
      throw std::runtime_error("hardware.canfd.interfaces must be non-empty when bus_type is canfd");
    }
  }
  // 解析关节配置
  const auto joints = hardware["joints"];
  if (!joints || !joints.IsSequence() || joints.size() == 0) {
    throw std::runtime_error("hardware.joints must be a non-empty sequence");
  }

  config.joints.reserve(joints.size());
  std::unordered_set<std::string> used_canfd_endpoints;
  std::unordered_set<std::string> used_gateway_slots;
  std::unordered_set<std::string> used_gateway_endpoints;
  for (const auto & item : joints) {
    auto joint = parseJoint(item, config.vendor_type, config.bus_type);
    // 重复关节会导致同一个 JointCommand 映射到多个硬件通道，必须拒绝。
    if (config.hasJoint(joint.name)) {
      throw std::runtime_error("duplicate hardware joint name: " + joint.name);
    }
    if (config.bus_type == "canfd") {
      // 允许不同 CANFD 总线复用同一个 motor_id，但同一路 bus 内必须唯一。
      bool bus_exists = false;
      for (const auto & iface : config.canfd.interfaces) {
        if (iface.bus == joint.canfd.bus) {
          bus_exists = true;
          break;
        }
      }
      if (!bus_exists) {
        throw std::runtime_error(
          "hardware joint '" + joint.name + "' references undefined canfd.bus " +
          std::to_string(joint.canfd.bus));
      }
      const auto endpoint =
        std::to_string(joint.canfd.bus) + ":" + std::to_string(joint.motor_id);
      if (!used_canfd_endpoints.insert(endpoint).second) {
        throw std::runtime_error(
          "duplicate CANFD motor mapping on bus " + std::to_string(joint.canfd.bus) +
          " motor_id=" + std::to_string(joint.motor_id));
      }
    }
    if (config.bus_type == "ethercat" && config.vendor_type == "mix") {
      const auto slot =
        std::to_string(joint.gateway.slave_id) + ":" +
        std::to_string(joint.gateway.bus) + ":" +
        std::to_string(joint.gateway.slot);
      if (!used_gateway_slots.insert(slot).second) {
        throw std::runtime_error(
          "duplicate gateway slot mapping: slave_id=" +
          std::to_string(joint.gateway.slave_id) +
          " bus=" + std::to_string(joint.gateway.bus) +
          " slot=" + std::to_string(joint.gateway.slot));
      }

      const auto endpoint =
        std::to_string(joint.gateway.slave_id) + ":" +
        std::to_string(joint.gateway.bus) + ":" +
        std::to_string(joint.motor_id);
      if (!used_gateway_endpoints.insert(endpoint).second) {
        throw std::runtime_error(
          "duplicate gateway motor mapping: slave_id=" +
          std::to_string(joint.gateway.slave_id) +
          " bus=" + std::to_string(joint.gateway.bus) +
          " motor_id=" + std::to_string(joint.motor_id));
      }
    }
    config.joint_map.emplace(joint.name, joint);
    config.joints.push_back(joint);
  }

  return config;
}

}  // namespace robot_driver
