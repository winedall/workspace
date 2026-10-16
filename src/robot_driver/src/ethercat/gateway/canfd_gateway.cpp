#include "robot_driver/ethercat/gateway/canfd_gateway.hpp"

#include <algorithm>

namespace robot_driver
{

namespace
{

constexpr std::size_t kGatewayPdoSize = 400;

#pragma pack(push, 1)
struct CanFdMessage
{
  // 网关 PDO 中的 CAN-FD 槽位格式。id 是线上 CAN id，data 是厂家协议原始载荷。
  std::uint32_t id{0};
  std::uint8_t type{0};
  std::uint8_t length{0};
  std::uint8_t data[16]{};
};

struct CanMessage
{
  std::uint32_t id{0};
  std::uint8_t type{0};
  std::uint8_t length{0};
  std::uint8_t data[8]{};
};

// 当前 IGH 网关的 400 字节 PDO 布局：两路 CAN-FD 各 7 个槽。
// 厂家协议只看到 CanfdFrame，槽位寻址统一由 gateway 配置决定。
struct GatewayMessage
{
  std::uint8_t reserved1[8]{};
  CanFdMessage can_fd0[7]{};
  CanFdMessage can_fd1[7]{};
  CanMessage can2[5]{};
  std::uint8_t reserved2[14]{};
};
#pragma pack(pop)

static_assert(sizeof(CanFdMessage) == 22, "EtherCAT gateway CAN-FD frame must be 22 bytes");
static_assert(sizeof(GatewayMessage) == kGatewayPdoSize, "EtherCAT gateway PDO must be 400 bytes");

// 将可写 PDO 缓冲区转换成网关布局结构。
GatewayMessage * asGateway(std::uint8_t * buffer, std::size_t size, std::string & error)
{
  // 所有槽位读写都先做边界检查，避免配置错误时直接 reinterpret_cast 到非法内存。
  if (buffer == nullptr) {
    error = "EtherCAT CANFD gateway output buffer is null";
    return nullptr;
  }
  if (size < sizeof(GatewayMessage)) {
    error = "EtherCAT CANFD gateway output buffer is too small";
    return nullptr;
  }
  return reinterpret_cast<GatewayMessage *>(buffer);
}

// 将只读 PDO 缓冲区转换成网关布局结构。
const GatewayMessage * asConstGateway(const std::uint8_t * buffer, std::size_t size, std::string & error)
{
  if (buffer == nullptr) {
    error = "EtherCAT CANFD gateway input buffer is null";
    return nullptr;
  }
  if (size < sizeof(GatewayMessage)) {
    error = "EtherCAT CANFD gateway input buffer is too small";
    return nullptr;
  }
  return reinterpret_cast<const GatewayMessage *>(buffer);
}

// 根据关节 gateway 配置定位可写 CAN-FD 槽位。
CanFdMessage * slot(GatewayMessage & gateway, const HardwareJointConfig & joint, std::string & error)
{
  // gateway.bus/slot 是硬件通道位置，不属于任何一家电机协议。
  const auto & config = joint.gateway;
  if (config.bus == 0 && config.slot >= 0 && config.slot < 7) {
    return &gateway.can_fd0[config.slot];
  }
  if (config.bus == 1 && config.slot >= 0 && config.slot < 7) {
    return &gateway.can_fd1[config.slot];
  }
  error = "EtherCAT CANFD gateway slot out of range for joint '" + joint.name + "'";
  return nullptr;
}

// 根据关节 gateway 配置定位只读 CAN-FD 槽位。
const CanFdMessage * slot(const GatewayMessage & gateway, const HardwareJointConfig & joint, std::string & error)
{
  const auto & config = joint.gateway;
  if (config.bus == 0 && config.slot >= 0 && config.slot < 7) {
    return &gateway.can_fd0[config.slot];
  }
  if (config.bus == 1 && config.slot >= 0 && config.slot < 7) {
    return &gateway.can_fd1[config.slot];
  }
  error = "EtherCAT CANFD gateway slot out of range for joint '" + joint.name + "'";
  return nullptr;
}

// 将通用 CanfdFrame 复制到网关 PDO 槽位。
void copyToGateway(const CanfdFrame & source, CanFdMessage & target)
{
  // 写入前清空整个槽位，避免上一周期较长数据残留到本周期短帧中。
  target = CanFdMessage{};
  target.id = source.id;
  target.type = 0;
  target.length = std::min<std::uint8_t>(source.length, kCanfdMaxDataLength);
  std::copy_n(source.data.begin(), target.length, target.data);
}

// 将网关 PDO 槽位内容恢复成通用 CanfdFrame。
void copyFromGateway(const CanFdMessage & source, CanfdFrame & target)
{
  target = CanfdFrame{};
  target.id = source.id;
  target.length = std::min<std::uint8_t>(source.length, kCanfdMaxDataLength);
  std::copy_n(std::begin(source.data), target.length, target.data.begin());
}

}  // namespace

// 把指定关节的一帧 CANFD 请求写入它的 gateway 输出槽位。
bool CanfdGateway::writeFrame(
  const HardwareJointConfig & joint,
  const CanfdFrame & frame,
  std::uint8_t * output_buffer,
  std::size_t output_size,
  std::string & error) const
{
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

// 从指定关节的 gateway 输入槽位读取一帧 CANFD 反馈。
bool CanfdGateway::readFrame(
  const HardwareJointConfig & joint,
  const std::uint8_t * input_buffer,
  std::size_t input_size,
  CanfdFrame & frame,
  std::string & error) const
{
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

// 清空指定关节的 gateway 输出槽位，避免旧帧在后续周期重复发送。
bool CanfdGateway::clearFrame(
  const HardwareJointConfig & joint,
  std::uint8_t * output_buffer,
  std::size_t output_size,
  std::string & error) const
{
  return writeFrame(joint, CanfdFrame{}, output_buffer, output_size, error);
}

// 在网关输入 PDO 的所有 CAN-FD 槽位中扫描指定命令响应。
bool CanfdGateway::findFrame(
  std::uint8_t command,
  std::uint32_t expected_id,
  bool use_expected_id,
  const std::uint8_t * input_buffer,
  std::size_t input_size,
  CanfdFrame & frame,
  std::string & error) const
{
  // 低频查询类回包可能不在固定关节槽位中，因此提供按命令字扫描全部 CAN-FD 槽位的入口。
  const auto * gateway = asConstGateway(input_buffer, input_size, error);
  if (gateway == nullptr) {
    return false;
  }

  const auto matches = [&](const CanFdMessage & candidate) {
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

  error = "EtherCAT CANFD gateway frame not found";
  return false;
}

}  // namespace robot_driver
