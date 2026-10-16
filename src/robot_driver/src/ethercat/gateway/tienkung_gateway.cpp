#include "robot_driver/ethercat/gateway/tienkung_gateway.hpp"

#include <algorithm>

namespace robot_driver
{

namespace
{

constexpr std::size_t kGatewayPdoSize = 400;

#pragma pack(push, 1)
// Tienkung EtherCAT 网关的 CAN-FD 槽布局，必须与 400 字节 PDO 定义一致。
struct CanFdMessage
{
  std::uint32_t id{0};
  std::uint8_t type{0};
  std::uint8_t length{0};
  std::uint8_t data[16]{};
};

// 网关 PDO 中也保留了普通 CAN 槽位，当前 robot_driver 只使用 CAN-FD 槽。
struct CanMessage
{
  std::uint32_t id{0};
  std::uint8_t type{0};
  std::uint8_t length{0};
  std::uint8_t data[8]{};
};

// 完整 PDO 布局：两路 CAN-FD 总线各 7 个槽，外加预留区和普通 CAN 区。
struct GatewayMessage
{
  std::uint8_t reserved1[8]{};
  CanFdMessage can_fd0[7]{};
  CanFdMessage can_fd1[7]{};
  CanMessage can2[5]{};
  std::uint8_t reserved2[14]{};
};
#pragma pack(pop)

static_assert(sizeof(CanFdMessage) == 22, "Tienkung gateway CAN-FD frame must be 22 bytes");
static_assert(sizeof(GatewayMessage) == kGatewayPdoSize, "Tienkung gateway PDO must be 400 bytes");

// 将输出 PDO buffer 转成网关结构体，并检查调用方传入的内存是否可用。
GatewayMessage * asGateway(std::uint8_t * buffer, std::size_t size, std::string & error)
{
  if (buffer == nullptr) {
    error = "Tienkung EtherCAT gateway output buffer is null";
    return nullptr;
  }
  if (size < sizeof(GatewayMessage)) {
    error = "Tienkung EtherCAT gateway output buffer is too small";
    return nullptr;
  }
  return reinterpret_cast<GatewayMessage *>(buffer);
}

// 输入 PDO 的只读版本，用于从 EtherCAT 输入区取回 CAN-FD 报文。
const GatewayMessage * asConstGateway(const std::uint8_t * buffer, std::size_t size, std::string & error)
{
  if (buffer == nullptr) {
    error = "Tienkung EtherCAT gateway input buffer is null";
    return nullptr;
  }
  if (size < sizeof(GatewayMessage)) {
    error = "Tienkung EtherCAT gateway input buffer is too small";
    return nullptr;
  }
  return reinterpret_cast<const GatewayMessage *>(buffer);
}

// 根据关节配置选择 PDO 槽位。bus 表示 CAN-FD 总线，slot 表示该总线内的槽。
CanFdMessage * slot(GatewayMessage & gateway, const HardwareJointConfig & joint, std::string & error)
{
  const auto & config = joint.tienkung;
  if (config.bus == 0 && config.slot >= 0 && config.slot < 7) {
    return &gateway.can_fd0[config.slot];
  }
  if (config.bus == 1 && config.slot >= 0 && config.slot < 7) {
    return &gateway.can_fd1[config.slot];
  }
  error = "Tienkung EtherCAT gateway slot out of range for joint '" + joint.name + "'";
  return nullptr;
}

// 只读槽位查找，用于输入 PDO 中的反馈或命令响应。
const CanFdMessage * slot(const GatewayMessage & gateway, const HardwareJointConfig & joint, std::string & error)
{
  const auto & config = joint.tienkung;
  if (config.bus == 0 && config.slot >= 0 && config.slot < 7) {
    return &gateway.can_fd0[config.slot];
  }
  if (config.bus == 1 && config.slot >= 0 && config.slot < 7) {
    return &gateway.can_fd1[config.slot];
  }
  error = "Tienkung EtherCAT gateway slot out of range for joint '" + joint.name + "'";
  return nullptr;
}

// 把通用 CanfdFrame 写入网关槽。长度限制由统一的 CAN-FD 最大数据长度控制。
void copyToGateway(const CanfdFrame & source, CanFdMessage & target)
{
  target = CanFdMessage{};
  target.id = source.id;
  target.type = 0;
  target.length = std::min<std::uint8_t>(source.length, kCanfdMaxDataLength);
  std::copy_n(source.data.begin(), target.length, target.data);
}

// 从网关槽恢复通用 CanfdFrame，协议解析由 TienkungProtocol 继续完成。
void copyFromGateway(const CanFdMessage & source, CanfdFrame & target)
{
  target = CanfdFrame{};
  target.id = source.id;
  target.length = std::min<std::uint8_t>(source.length, kCanfdMaxDataLength);
  std::copy_n(std::begin(source.data), target.length, target.data.begin());
}

}  // namespace

// 将通用 CanfdFrame 写入天工 EtherCAT 网关中关节配置对应的 CAN-FD 槽。
//
// bus/slot 的寻址来自 HardwareJointConfig，函数本身不关心帧内命令字含义。
bool TienkungEthercatGateway::writeFrame(
  const HardwareJointConfig & joint,
  const CanfdFrame & frame,
  std::uint8_t * output_buffer,
  std::size_t output_size,
  std::string & error) const
{
  // 只处理槽位寻址和字节复制，不理解 0x10、0x14 等厂家命令。
  auto * gateway = asGateway(output_buffer, output_size, error);
  if (gateway == nullptr) {
    return false;
  }
  auto * target = slot(*gateway, joint, error);
  if (target == nullptr) {
    return false;
  }
  copyToGateway(frame, *target);
  return true;
}

// 从天工 EtherCAT 网关中关节配置对应的 CAN-FD 槽读取一帧 CanfdFrame。
//
// 该入口适合周期反馈这种“一关节固定一槽”的读取场景。
bool TienkungEthercatGateway::readFrame(
  const HardwareJointConfig & joint,
  const std::uint8_t * input_buffer,
  std::size_t input_size,
  CanfdFrame & frame,
  std::string & error) const
{
  // 按配置读取固定槽位，适合周期反馈这类一关节一槽的场景。
  const auto * gateway = asConstGateway(input_buffer, input_size, error);
  if (gateway == nullptr) {
    return false;
  }
  const auto * source = slot(*gateway, joint, error);
  if (source == nullptr) {
    return false;
  }
  copyFromGateway(*source, frame);
  return true;
}

// 清空天工 EtherCAT 网关中关节对应的输出槽。
//
// 周期控制和低频启停命令都会用它避免旧帧残留。
bool TienkungEthercatGateway::clearFrame(
  const HardwareJointConfig & joint,
  std::uint8_t * output_buffer,
  std::size_t output_size,
  std::string & error) const
{
  // 清空槽位时写入空帧，保持输出 PDO 的结构完整。
  return writeFrame(joint, CanfdFrame{}, output_buffer, output_size, error);
}

// 写入天工广播命令帧。
//
// 广播命令没有单关节 bus/slot，因此统一使用网关约定的 can_fd0[0] 槽。
bool TienkungEthercatGateway::writeBroadcastFrame(
  const CanfdFrame & frame,
  std::uint8_t * output_buffer,
  std::size_t output_size,
  std::string & error) const
{
  // 广播命令没有具体关节槽位，统一放入网关约定的第一个 CAN-FD 槽。
  auto * gateway = asGateway(output_buffer, output_size, error);
  if (gateway == nullptr) {
    return false;
  }
  copyToGateway(frame, gateway->can_fd0[0]);
  return true;
}

// 在天工 EtherCAT 输入 PDO 的所有 CAN-FD 槽中查找指定命令响应。
//
// use_expected_id=true 时同时匹配 CAN id；广播查询这类响应可用该入口从任意槽取回。
bool TienkungEthercatGateway::findFrame(
  std::uint8_t command,
  std::uint32_t expected_id,
  bool use_expected_id,
  const std::uint8_t * input_buffer,
  std::size_t input_size,
  CanfdFrame & frame,
  std::string & error) const
{
  // 查询类回包可能出现在任意 CAN-FD 输入槽中，因此需要扫描两个总线数组。
  const auto * gateway = asConstGateway(input_buffer, input_size, error);
  if (gateway == nullptr) {
    return false;
  }

  const auto matches = [&](const CanFdMessage & candidate) {
    // 第 0 字节是 Tienkung 协议命令字；id 匹配按调用方需要决定。
    if (candidate.length == 0 || candidate.data[0] != command) {
      return false;
    }
    return !use_expected_id || candidate.id == expected_id;
  };

  for (const auto & candidate : gateway->can_fd0) {
    if (matches(candidate)) {
      copyFromGateway(candidate, frame);
      return true;
    }
  }
  for (const auto & candidate : gateway->can_fd1) {
    if (matches(candidate)) {
      copyFromGateway(candidate, frame);
      return true;
    }
  }

  error = "Tienkung EtherCAT gateway frame not found";
  return false;
}

}  // namespace robot_driver
