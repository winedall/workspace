#include "robot_driver/ethercat/gateway/encos_gateway.hpp"

#include <algorithm>

namespace robot_driver
{

namespace
{

#pragma pack(push, 1)
// Encos EtherCAT 从站将底层 CAN 报文压入固定长度的 PDO 槽。
// 这里的结构体必须与从站 PDO 二进制布局保持一致。
struct MotorMsg
{
  std::uint32_t id{0};
  std::uint8_t rtr{0};
  std::uint8_t dlc{0};
  std::uint8_t data[8]{};
};

struct EthercatMsg
{
  // motor_num 和 can_ide 是网关侧需要的 PDO 头字段，不属于厂家协议内容。
  std::uint8_t motor_num{0};
  std::uint8_t can_ide{0};
  MotorMsg motor[EncosEthercatGateway::kFramesPerSlave]{};
};
#pragma pack(pop)

// 将原始 PDO buffer 转成可访问结构，同时完成空指针和长度检查。
EthercatMsg * asMessage(std::uint8_t * buffer, std::size_t size, std::string & error)
{
  if (buffer == nullptr) {
    error = "Encos EtherCAT gateway output buffer is null";
    return nullptr;
  }
  if (size < sizeof(EthercatMsg)) {
    error = "Encos EtherCAT gateway output buffer is too small";
    return nullptr;
  }
  return reinterpret_cast<EthercatMsg *>(buffer);
}

// 输入 PDO 的只读版本，用于读取反馈或查询响应。
const EthercatMsg * asConstMessage(const std::uint8_t * buffer, std::size_t size, std::string & error)
{
  if (buffer == nullptr) {
    error = "Encos EtherCAT gateway input buffer is null";
    return nullptr;
  }
  if (size < sizeof(EthercatMsg)) {
    error = "Encos EtherCAT gateway input buffer is too small";
    return nullptr;
  }
  return reinterpret_cast<const EthercatMsg *>(buffer);
}

// Encos passage 来自硬件配置，约定为 1 到 kFramesPerSlave。
bool validatePassage(int passage, std::string & error)
{
  if (passage < 1 || passage > EncosEthercatGateway::kFramesPerSlave) {
    error = "Encos EtherCAT gateway passage out of range: " + std::to_string(passage);
    return false;
  }
  return true;
}

// 清空单个 PDO 槽，避免数据区残留影响下一帧短报文。
void clearMotor(MotorMsg & motor)
{
  motor.id = 0;
  motor.rtr = 0;
  motor.dlc = 0;
  std::fill(std::begin(motor.data), std::end(motor.data), 0);
}

}  // namespace

// 将通用 CanfdFrame 写入 Encos EtherCAT PDO 的指定 passage 槽位。
//
// 该函数只做内存布局转换和边界检查，不解释 CAN id 或数据区中的厂家协议字段。
bool EncosEthercatGateway::writeFrame(
  int passage,
  const CanfdFrame & frame,
  std::uint8_t * output_buffer,
  std::size_t output_size,
  std::string & error) const
{
  // 这里仅做 CAN 帧到 PDO 槽的布局转换，不修改厂家协议字段。
  if (!validatePassage(passage, error)) {
    return false;
  }
  auto * message = asMessage(output_buffer, output_size, error);
  if (message == nullptr) {
    return false;
  }
  auto & motor = message->motor[passage - 1];
  clearMotor(motor);
  message->motor_num = kFramesPerSlave;
  message->can_ide = 0;
  motor.id = frame.id;
  motor.rtr = 0;
  motor.dlc = std::min<std::uint8_t>(frame.length, 8);
  std::copy_n(frame.data.begin(), motor.dlc, motor.data);
  return true;
}

// 从 Encos EtherCAT PDO 的指定 passage 槽位读取一帧通用 CanfdFrame。
//
// 读取结果保持原始 CAN 帧形态，后续由 EncosProtocol 判断反馈类型和命令含义。
bool EncosEthercatGateway::readFrame(
  int passage,
  const std::uint8_t * input_buffer,
  std::size_t input_size,
  CanfdFrame & frame,
  std::string & error) const
{
  // 读取时同样保持原始 CAN 帧语义，后续交给 EncosProtocol 解码。
  if (!validatePassage(passage, error)) {
    return false;
  }
  const auto * message = asConstMessage(input_buffer, input_size, error);
  if (message == nullptr) {
    return false;
  }
  const auto & motor = message->motor[passage - 1];
  frame = CanfdFrame{};
  frame.id = motor.id;
  frame.length = std::min<std::uint8_t>(motor.dlc, 8);
  std::copy_n(std::begin(motor.data), frame.length, frame.data.begin());
  return true;
}

// 清空 Encos EtherCAT PDO 的指定 passage 输出槽。
//
// 用于停止继续发送上一周期命令，或者在 disable/config 事务结束后清理输出区。
bool EncosEthercatGateway::clearFrame(
  int passage,
  std::uint8_t * output_buffer,
  std::size_t output_size,
  std::string & error) const
{
  // 复用 writeFrame 的边界检查和槽位清空逻辑。
  return writeFrame(passage, CanfdFrame{}, output_buffer, output_size, error);
}

}  // namespace robot_driver
