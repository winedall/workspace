#include "robot_driver/protocol/tienkung_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>

namespace robot_driver
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kGainScale = 10.0;
constexpr double kPowerScale = 20.0;

// 将浮点值夹紧到 uint16_t 范围，用于 kp/kd 等无符号量化字段。
std::uint16_t clampU16(double value)
{
  const auto clamped = std::max(0.0, std::min(65535.0, value));
  return static_cast<std::uint16_t>(std::llround(clamped));
}

// 将浮点值夹紧到 int16_t 范围，用于力矩/电流等有符号量化字段。
std::int16_t clampI16(double value)
{
  const auto clamped = std::max(
    static_cast<double>(std::numeric_limits<std::int16_t>::min()),
    std::min(static_cast<double>(std::numeric_limits<std::int16_t>::max()), value));
  return static_cast<std::int16_t>(std::llround(clamped));
}

// 天工协议多字节整数使用大端序。
void writeU16Be(std::uint8_t * data, std::uint16_t value)
{
  data[0] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
  data[1] = static_cast<std::uint8_t>(value & 0xFF);
}

// 有符号整数按补码原始位写入大端序字段。
void writeI16Be(std::uint8_t * data, std::int16_t value)
{
  writeU16Be(data, static_cast<std::uint16_t>(value));
}

// 从大端序字段读取无符号 16 位整数。
std::uint16_t readU16Be(const std::uint8_t * data)
{
  return static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(data[0]) << 8) |
    static_cast<std::uint16_t>(data[1]));
}

// 从大端序字段读取有符号 16 位整数。
std::int16_t readI16Be(const std::uint8_t * data)
{
  return static_cast<std::int16_t>(readU16Be(data));
}

// 天工协议中的 float 字段按 IEEE754 原始位的大端序传输。
void writeFloatBe(std::uint8_t * data, float value)
{
  std::uint32_t raw = 0;
  std::memcpy(&raw, &value, sizeof(raw));
  data[0] = static_cast<std::uint8_t>((raw >> 24) & 0xFF);
  data[1] = static_cast<std::uint8_t>((raw >> 16) & 0xFF);
  data[2] = static_cast<std::uint8_t>((raw >> 8) & 0xFF);
  data[3] = static_cast<std::uint8_t>(raw & 0xFF);
}

// 从大端序 IEEE754 原始位恢复 float。
float readFloatBe(const std::uint8_t * data)
{
  std::uint32_t raw =
    (static_cast<std::uint32_t>(data[0]) << 24) |
    (static_cast<std::uint32_t>(data[1]) << 16) |
    (static_cast<std::uint32_t>(data[2]) << 8) |
    static_cast<std::uint32_t>(data[3]);
  float value = 0.0F;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

// 初始化一帧面向指定电机 ID 的天工 CAN-FD 报文。
void prepareJointFrame(const HardwareJointConfig & joint, CanfdFrame & frame)
{
  frame = CanfdFrame{};
  frame.id = static_cast<std::uint32_t>(joint.motor_id);
}

}  // namespace

// 返回当前协议实现名称。
std::string TienkungProtocol::name() const
{
  return "tienkung";
}

// 编码天工 0x11 力位混合控制帧。
//
// 输入是统一关节空间命令，输出前完成方向、零位和减速比换算。
bool TienkungProtocol::encodeMotion(
  const HardwareJointConfig & hardware_joint,
  const robot_interfaces::msg::JointCommand & command,
  std::uint8_t counter,
  CanfdFrame & frame,
  std::string & error) const
{
  prepareJointFrame(hardware_joint, frame);
  if (!command.enable) {
    (void)error;
    return true;
  }

  const auto direction = static_cast<double>(hardware_joint.direction);
  const auto ratio = hardware_joint.ratio;
  const auto motor_position_rad = (command.position * direction + hardware_joint.offset * kDegToRad) * ratio;
  const auto motor_velocity_radps = command.velocity * direction * ratio;
  const auto motor_torque_raw = command.effort * direction * ratio;

  frame.length = 16;
  frame.data[0] = 0x11;
  writeU16Be(&frame.data[1], clampU16(command.kp * kGainScale));
  writeU16Be(&frame.data[3], clampU16(command.kd * kGainScale));
  writeFloatBe(&frame.data[5], static_cast<float>(motor_position_rad * kRadToDeg));
  writeFloatBe(&frame.data[9], static_cast<float>(motor_velocity_radps * 30.0 / kPi));
  writeI16Be(&frame.data[13], clampI16(motor_torque_raw * kPowerScale));
  frame.data[15] = counter;
  (void)error;
  return true;
}

// 编码天工 0x10 使能帧。
//
// 该帧用于启动握手，不参与周期运动目标发送。
bool TienkungProtocol::encodeEnable(
  const HardwareJointConfig & hardware_joint,
  CanfdFrame & frame,
  std::string & error) const
{
  prepareJointFrame(hardware_joint, frame);
  frame.length = 2;
  frame.data[0] = 0x10;
  frame.data[1] = 1;
  (void)error;
  return true;
}

// 编码天工 0x10 失能帧。
//
// 该帧用于启动前清状态和退出时安全断使能。
bool TienkungProtocol::encodeDisable(
  const HardwareJointConfig & hardware_joint,
  CanfdFrame & frame,
  std::string & error) const
{
  prepareJointFrame(hardware_joint, frame);
  frame.length = 2;
  frame.data[0] = 0x10;
  frame.data[1] = 0;
  (void)error;
  return true;
}

// 编码天工 0x03 设置零点帧。
bool TienkungProtocol::encodeSetZero(
  const HardwareJointConfig & hardware_joint,
  CanfdFrame & frame,
  std::string & error) const
{
  prepareJointFrame(hardware_joint, frame);
  frame.length = 1;
  frame.data[0] = 0x03;
  (void)error;
  return true;
}

// 编码天工 0x05 查询 MCU ID 帧。
//
// 该帧与置零帧同样使用 dlc=1、CAN id=电机 id，仅把命令码换成 0x05；
// 电机侧应答帧的解析由 backend 原样回传，不在协议层解释。
bool TienkungProtocol::encodeQueryMcuId(
  const HardwareJointConfig & hardware_joint,
  CanfdFrame & frame,
  std::string & error) const
{
  prepareJointFrame(hardware_joint, frame);
  frame.length = 1;
  frame.data[0] = 0x05;
  (void)error;
  return true;
}

// 编码天工 0x02 修改电机 ID 帧。
//
// 帧中同时携带旧 ID 和新 ID，用于电机侧确认修改目标。
bool TienkungProtocol::encodeModifyId(
  const HardwareJointConfig & hardware_joint,
  std::uint8_t new_id,
  CanfdFrame & frame,
  std::string & error) const
{
  prepareJointFrame(hardware_joint, frame);
  frame.length = 3;
  frame.data[0] = 0x02;
  frame.data[1] = static_cast<std::uint8_t>(hardware_joint.motor_id);
  frame.data[2] = new_id;
  (void)error;
  return true;
}

// 编码天工广播查询 ID 帧。
//
// 广播帧使用 CAN id 0x000，具体放入哪个 PDO 槽由 gateway/backend 决定。
bool TienkungProtocol::encodeQueryId(CanfdFrame & frame, std::string & error) const
{
  frame = CanfdFrame{};
  frame.id = 0x000;
  frame.length = 8;
  std::fill(frame.data.begin(), frame.data.begin() + frame.length, 0);
  (void)error;
  return true;
}

// 编码天工广播重置 ID 帧。
//
// 数据区为厂家定义的固定 reset code。
bool TienkungProtocol::encodeResetId(CanfdFrame & frame, std::string & error) const
{
  frame = CanfdFrame{};
  frame.id = 0x000;
  frame.length = 8;
  const std::uint8_t reset_code[] = {0x01, 0xD5, 0xA3, 0xF1, 0xE8, 0xC2, 0xB7, 0x34};
  std::copy(std::begin(reset_code), std::end(reset_code), frame.data.begin());
  (void)error;
  return true;
}

// 解码天工 0x80 默认状态反馈帧。
//
// 解码后完成电机空间到统一关节空间的方向、零位、减速比和 kt 换算。
bool TienkungProtocol::decodeFeedback(
  const HardwareJointConfig & hardware_joint,
  const CanfdFrame & frame,
  TienkungJointFeedback & feedback,
  std::string & error) const
{
  if (frame.length == 0) {
    error.clear();
    return false;
  }
  if (frame.id != static_cast<std::uint32_t>(hardware_joint.motor_id)) {
    error = "Tienkung feedback motor_id mismatch for joint '" + hardware_joint.name + "'";
    return false;
  }
  if (frame.length != 16 || frame.data[0] != 0x80) {
    error = "Tienkung feedback does not contain default status for joint '" + hardware_joint.name + "'";
    return false;
  }

  const auto mode_error = readU16Be(&frame.data[1]);
  const auto motor_position_rad = static_cast<double>(readFloatBe(&frame.data[3])) * kDegToRad;
  const auto motor_velocity_radps = static_cast<double>(readFloatBe(&frame.data[7])) * kPi / 30.0;
  const auto motor_current_a = static_cast<double>(readI16Be(&frame.data[11])) / 100.0;
  const auto direction = static_cast<double>(hardware_joint.direction);
  const auto ratio = hardware_joint.ratio;
  const auto kt = hardware_joint.tienkung.kt > 0.0 ? hardware_joint.tienkung.kt : 1.0;

  feedback.position = (motor_position_rad / ratio - hardware_joint.offset * kDegToRad) * direction;
  feedback.velocity = motor_velocity_radps / ratio * direction;
  feedback.effort = motor_current_a * kt / ratio * direction;
  feedback.mode = static_cast<std::uint8_t>((mode_error >> 12) & 0x0F);
  feedback.motor_error = static_cast<std::uint16_t>(mode_error & 0x0FFF);
  feedback.motor_temp = static_cast<std::int8_t>(frame.data[13]) - 50;
  feedback.mos_temp = static_cast<std::int8_t>(frame.data[14]) - 50;
  feedback.bus_voltage = static_cast<double>(frame.data[15]) / 2.0;
  return true;
}

// 解码天工状态查询反馈，并转换成管理事务结果。
bool TienkungProtocol::decodeQueryStatus(
  const HardwareJointConfig & hardware_joint,
  const CanfdFrame & frame,
  TienkungAdminResult & result,
  std::string & error) const
{
  TienkungJointFeedback feedback;
  if (!decodeFeedback(hardware_joint, frame, feedback, error)) {
    return false;
  }

  result = TienkungAdminResult{};
  result.success = true;
  result.message = "tienkung query status completed";
  result.motor_id = static_cast<std::uint16_t>(hardware_joint.motor_id);
  result.feedback_code = 0;
  result.position =
    (feedback.position * static_cast<double>(hardware_joint.direction) +
    hardware_joint.offset * kDegToRad) * hardware_joint.ratio * kRadToDeg;
  result.velocity = feedback.velocity;
  result.effort = feedback.effort;
  result.motor_error = feedback.motor_error;
  result.motor_temp = static_cast<float>(feedback.motor_temp);
  result.mos_temp = static_cast<float>(feedback.mos_temp);
  return true;
}

// 解码天工 0x90 启停应答帧。
//
// enabled 输出电机侧确认后的实际启停状态。
bool TienkungProtocol::decodeStartStop(
  const HardwareJointConfig & hardware_joint,
  const CanfdFrame & frame,
  bool & enabled,
  std::string & error) const
{
  if (frame.length < 2 || frame.data[0] != 0x90) {
    error = "Tienkung start/stop ack frame invalid for joint '" + hardware_joint.name + "'";
    return false;
  }
  if (frame.id != static_cast<std::uint32_t>(hardware_joint.motor_id)) {
    error = "Tienkung start/stop ack motor id mismatch for joint '" + hardware_joint.name + "'";
    return false;
  }
  enabled = frame.data[1] != 0;
  return true;
}

// 解码天工 0x83 设置零点应答帧。
//
// success 表示厂家侧是否接受置零，zero_position_rad 为电机侧返回的零点位置。
bool TienkungProtocol::decodeSetZero(
  const HardwareJointConfig & hardware_joint,
  const CanfdFrame & frame,
  TienkungAdminResult & result,
  std::string & error) const
{
  if (frame.id != static_cast<std::uint32_t>(hardware_joint.motor_id)) {
    error = "Tienkung set zero motor_id mismatch for joint '" + hardware_joint.name + "'";
    return false;
  }
  if (frame.length < 6 || frame.data[0] != 0x83) {
    error = "Tienkung set zero frame invalid";
    return false;
  }
  result = TienkungAdminResult{};
  result.success = frame.data[1] != 0;
  result.message = result.success ? "tienkung set zero completed" : "tienkung set zero failed";
  result.motor_id = static_cast<std::uint16_t>(hardware_joint.motor_id);
  result.feedback_code = 0;
  result.zero_position_rad = readFloatBe(&frame.data[2]) * kDegToRad;
  return true;
}

// 解码天工 0x82 修改 ID 应答帧。
bool TienkungProtocol::decodeModifyId(
  const HardwareJointConfig & hardware_joint,
  const CanfdFrame & frame,
  TienkungAdminResult & result,
  std::string & error) const
{
  if (frame.id != static_cast<std::uint32_t>(hardware_joint.motor_id)) {
    error = "Tienkung modify id motor_id mismatch for joint '" + hardware_joint.name + "'";
    return false;
  }
  if (frame.length < 3 || frame.data[0] != 0x82) {
    error = "Tienkung modify id frame invalid";
    return false;
  }
  result = TienkungAdminResult{};
  result.success = true;
  result.message = "tienkung modify id completed";
  result.motor_id = static_cast<std::uint16_t>(hardware_joint.motor_id);
  result.new_motor_id = frame.data[2];
  result.feedback_code = 0;
  return true;
}

// 解码天工广播查询 ID 响应。
//
// 广播响应的 CAN id 固定为 0x000，电机 ID 位于数据区第 1 字节。
bool TienkungProtocol::decodeQueryId(
  const CanfdFrame & frame,
  TienkungAdminResult & result,
  std::string & error) const
{
  if (frame.length < 2 || frame.id != 0x000 || frame.data[0] != 0x80) {
    error = "Tienkung query id frame invalid";
    return false;
  }
  result = TienkungAdminResult{};
  result.success = true;
  result.message = "tienkung query id completed";
  result.motor_id = frame.data[1];
  result.feedback_code = 0;
  return true;
}

// 解码天工广播重置 ID 响应。
bool TienkungProtocol::decodeResetId(
  const CanfdFrame & frame,
  TienkungAdminResult & result,
  std::string & error) const
{
  if (frame.length < 2 || frame.id != 0x000 || frame.data[0] != 0x81) {
    error = "Tienkung reset id frame invalid";
    return false;
  }
  result = TienkungAdminResult{};
  result.success = frame.data[1] != 0;
  result.message = result.success ? "tienkung reset id completed" : "tienkung reset id failed";
  result.feedback_code = 0;
  return true;
}

}  // namespace robot_driver
