#include "robot_driver/protocol/encos_protocol.hpp"

#include <algorithm>
#include <cstring>
#include <cstdint>
#include <string>

namespace robot_driver
{

namespace
{

// 配置文件中的 offset 沿用旧驱动配置，单位是 degree；统一 JointCommand 使用 rad。
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

// 将浮点物理量按指定范围压缩为无符号整数。
int floatToUint(double value, double min_value, double max_value, int bits)
{
  const auto clamped = std::max(min_value, std::min(max_value, value));
  const auto span = max_value - min_value;
  const auto max_int = (1 << bits) - 1;
  return static_cast<int>((clamped - min_value) * static_cast<double>(max_int) / span);
}

// 将无符号整数按指定范围展开为浮点物理量。
double uintToFloat(int value, double min_value, double max_value, int bits)
{
  const auto span = max_value - min_value;
  const auto max_int = (1 << bits) - 1;
  return static_cast<double>(value) * span / static_cast<double>(max_int) + min_value;
}

// 获取 uint16_t 的高字节
std::uint8_t highByte(std::uint16_t value)
{
  return static_cast<std::uint8_t>((value >> 8) & 0xFF);
}

// 获取 uint16_t 的低字节
std::uint8_t lowByte(std::uint16_t value)
{
  return static_cast<std::uint8_t>(value & 0xFF);
}

// 根据高字节和低字节构造 uint16_t
std::uint16_t readU16(std::uint8_t high, std::uint8_t low)
{
  return static_cast<std::uint16_t>((static_cast<std::uint16_t>(high) << 8) | low);
}

// 根据高字节和低字节构造 int16_t
std::int16_t readI16(std::uint8_t high, std::uint8_t low)
{
  return static_cast<std::int16_t>(readU16(high, low));
}

// 根据大端字节序解析 float
float readBigEndianFloat(const std::uint8_t * data)
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

}  // namespace

// 返回当前协议实现名称。
std::string EncosProtocol::name() const
{
  return "encos";
}

// 编码 Encos 力位混合控制帧。
//
// 输入是统一关节空间命令，输出前完成方向、零位和减速比换算，再按 Encos 量化范围打包。
bool EncosProtocol::encodeMotion(
  const HardwareJointConfig & hardware_joint,
  const robot_interfaces::msg::JointCommand & command,
  CanfdFrame & frame,
  std::string & error) const
{
  // CANFD 直连模式下无需 EtherCAT passage，只需要一帧 CAN id=motor_id 的厂家控制帧。
  frame = CanfdFrame{};
  frame.id = static_cast<std::uint32_t>(hardware_joint.motor_id);
  if (!command.enable) {
    return true;
  }

  const auto direction = static_cast<double>(hardware_joint.direction);
  const auto ratio = hardware_joint.ratio;
  const auto position_rad = (command.position * direction + hardware_joint.offset * kDegToRad) * ratio;
  const auto velocity_radps = command.velocity * direction * ratio;
  const auto effort_nm = command.effort * direction * ratio;

  const auto & params = hardware_joint.encos;
  const auto kp_int = floatToUint(command.kp, params.kp_min, params.kp_max, 12);
  const auto kd_int = floatToUint(command.kd, params.kd_min, params.kd_max, 9);
  const auto pos_int = floatToUint(position_rad, params.pos_min, params.pos_max, 16);
  const auto spd_int = floatToUint(velocity_radps, params.spd_min, params.spd_max, 12);
  const auto tor_int = floatToUint(effort_nm, params.tor_min, params.tor_max, 12);

  frame.length = 8;
  frame.data[0] = static_cast<std::uint8_t>(0x00 | (kp_int >> 7));
  frame.data[1] = static_cast<std::uint8_t>(((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8));
  frame.data[2] = static_cast<std::uint8_t>(kd_int & 0xFF);
  frame.data[3] = static_cast<std::uint8_t>(pos_int >> 8);
  frame.data[4] = static_cast<std::uint8_t>(pos_int & 0xFF);
  frame.data[5] = static_cast<std::uint8_t>(spd_int >> 4);
  frame.data[6] = static_cast<std::uint8_t>(((spd_int & 0x0F) << 4) | (tor_int >> 8));
  frame.data[7] = static_cast<std::uint8_t>(tor_int & 0xFF);
  (void)error;
  return true;
}

// 直接编码 Encos CANFD 读取电机 ID 帧。
bool EncosProtocol::encodeQueryId(CanfdFrame & frame, std::string & error) const
{
  frame = CanfdFrame{};
  frame.id = 0x7FF;
  frame.length = 4;
  frame.data[0] = 0xFF;
  frame.data[1] = 0xFF;
  frame.data[2] = 0x00;
  frame.data[3] = 0x82;
  (void)error;
  return true;
}

// 直接编码 Encos CANFD 设置电机 ID 帧。
bool EncosProtocol::encodeModifyId(
  std::uint16_t motor_id,
  std::uint16_t new_motor_id,
  CanfdFrame & frame,
  std::string & error) const
{
  frame = CanfdFrame{};
  frame.id = 0x7FF;
  frame.length = 6;
  frame.data[0] = highByte(motor_id);
  frame.data[1] = lowByte(motor_id);
  frame.data[2] = 0x00;
  frame.data[3] = 0x04;
  frame.data[4] = highByte(new_motor_id);
  frame.data[5] = lowByte(new_motor_id);
  (void)error;
  return true;
}

// 直接编码 Encos CANFD 设置零点帧。
bool EncosProtocol::encodeSetZero(
  std::uint16_t motor_id,
  CanfdFrame & frame,
  std::string & error) const
{
  frame = CanfdFrame{};
  frame.id = 0x7FF;
  frame.length = 4;
  frame.data[0] = highByte(motor_id);
  frame.data[1] = lowByte(motor_id);
  frame.data[2] = 0x00;
  frame.data[3] = 0x03;
  (void)error;
  return true;
}

// 直接编码 Encos CANFD 参数读取帧。
bool EncosProtocol::encodeQueryParam(
  std::uint16_t motor_id,
  std::uint8_t param_code,
  CanfdFrame & frame,
  std::string & error) const
{
  frame = CanfdFrame{};
  frame.id = motor_id;
  frame.length = 2;
  frame.data[0] = 0xE0;
  frame.data[1] = param_code;
  (void)error;
  return true;
}

// 直接解码 Encos CANFD 力位混控反馈帧。
//
// 解析结果会完成方向、零位、减速比和 kt 换算，输出统一关节空间反馈。
bool EncosProtocol::decodeFeedback(
  const HardwareJointConfig & hardware_joint,
  const CanfdFrame & frame,
  EncosJointFeedback & feedback,
  std::string & error) const
{
  // 该解码逻辑与 EtherCAT PDO 里的单路 MotorMsg 反馈一致，只是输入来源变成 CanfdFrame。
  if (frame.length == 0) {
    error.clear();
    return false;
  }
  if (frame.id != static_cast<std::uint32_t>(hardware_joint.motor_id)) {
    error = "Encos CANFD feedback motor_id mismatch for joint '" + hardware_joint.name + "'";
    return false;
  }
  const auto ack_status = frame.data[0] >> 5;
  if (ack_status != 1 || frame.length != 8) {
    error = "Encos CANFD feedback does not contain torque-position status for joint '" +
      hardware_joint.name + "'";
    return false;
  }

  const auto pos_int = (static_cast<int>(frame.data[1]) << 8) | frame.data[2];
  const auto spd_int = (static_cast<int>(frame.data[3]) << 4) | ((frame.data[4] & 0xF0) >> 4);
  const auto cur_int = ((frame.data[4] & 0x0F) << 8) | frame.data[5];
  const auto & params = hardware_joint.encos;
  const auto motor_position_rad = uintToFloat(pos_int, params.pos_min, params.pos_max, 16);
  const auto motor_velocity_radps = uintToFloat(spd_int, params.spd_min, params.spd_max, 12);
  const auto motor_current_a = uintToFloat(cur_int, params.cur_min, params.cur_max, 12);
  const auto direction = static_cast<double>(hardware_joint.direction);
  const auto ratio = hardware_joint.ratio;

  feedback.position = (motor_position_rad / ratio - hardware_joint.offset * kDegToRad) * direction;
  feedback.velocity = motor_velocity_radps / ratio * direction;
  feedback.effort = motor_current_a * hardware_joint.encos.kt / ratio * direction;
  feedback.motor_error = frame.data[0] & 0x1F;
  feedback.motor_temp = static_cast<std::uint8_t>((frame.data[6] - 50) / 2);
  feedback.mos_temp = static_cast<std::uint8_t>((frame.data[7] - 50) / 2);
  return true;
}

// 直接解码 Encos CANFD 电机 ID 反馈。
bool EncosProtocol::decodeQueryId(
  const CanfdFrame & frame,
  EncosAdminResult & result,
  std::string & error) const
{
  if (frame.length == 0) {
    error = "Encos CANFD get motor id feedback frame is empty";
    return false;
  }
  result = EncosAdminResult{};
  if (frame.id != 0x7FFu || frame.data[2] != 0x01) {
    error = "Encos CANFD get motor id feedback is not a broadcast management response";
    return false;
  }
  if (frame.length == 5 && frame.data[0] == 0xFF && frame.data[1] == 0xFF) {
    result.success = true;
    result.operation = EncosAdminOperation::GetMotorId;
    result.motor_id = readU16(frame.data[3], frame.data[4]);
    result.feedback_code = 0x06;
    return true;
  }
  if (frame.length == 4 && frame.data[0] == 0x80 && frame.data[1] == 0x80) {
    result.success = false;
    result.operation = EncosAdminOperation::GetMotorId;
    result.feedback_code = 0x80;
    return true;
  }
  error = "Encos CANFD get motor id feedback format mismatch";
  return false;
}

// 直接解码 Encos CANFD 设置 ID 反馈。
bool EncosProtocol::decodeModifyId(
  std::uint16_t expected_motor_id,
  std::uint16_t requested_new_motor_id,
  const CanfdFrame & frame,
  EncosAdminResult & result,
  std::string & error) const
{
  if (frame.length == 0) {
    error = "Encos CANFD set motor id feedback frame is empty";
    return false;
  }
  if (frame.id != 0x7FFu || frame.data[2] != 0x01 || frame.length != 4) {
    error = "Encos CANFD set motor id feedback format mismatch";
    return false;
  }
  result = EncosAdminResult{};
  const auto motor_id = readU16(frame.data[0], frame.data[1]);
  if (expected_motor_id != 0 && motor_id != expected_motor_id) {
    error = "Encos CANFD set motor id feedback motor id mismatch";
    return false;
  }
  result.success = true;
  result.operation = EncosAdminOperation::SetMotorId;
  result.motor_id = motor_id;
  result.new_motor_id = requested_new_motor_id;
  result.feedback_code = frame.data[3];
  return true;
}

// 直接解码 Encos CANFD 设置零点反馈。
bool EncosProtocol::decodeSetZero(
  std::uint16_t expected_motor_id,
  const CanfdFrame & frame,
  EncosAdminResult & result,
  std::string & error) const
{
  if (frame.length == 0) {
    error = "Encos CANFD set motor zero feedback frame is empty";
    return false;
  }
  if (frame.id != 0x7FFu || frame.data[2] != 0x01 || frame.length != 4) {
    error = "Encos CANFD set motor zero feedback format mismatch";
    return false;
  }
  const auto motor_id = readU16(frame.data[0], frame.data[1]);
  if (expected_motor_id != 0 && motor_id != expected_motor_id) {
    error = "Encos CANFD set motor zero feedback motor id mismatch";
    return false;
  }
  result = EncosAdminResult{};
  result.success = true;
  result.operation = EncosAdminOperation::SetMotorZero;
  result.motor_id = motor_id;
  result.feedback_code = frame.data[3];
  return true;
}

// 直接解码 Encos CANFD 参数读取反馈。
bool EncosProtocol::decodeQueryParam(
  std::uint16_t expected_motor_id,
  std::uint8_t expected_param_code,
  const CanfdFrame & frame,
  EncosAdminResult & result,
  std::string & error) const
{
  if (frame.length == 0) {
    error = "Encos CANFD get motor parameter feedback frame is empty";
    return false;
  }
  if (frame.id != expected_motor_id) {
    error = "Encos CANFD get motor parameter feedback motor id mismatch";
    return false;
  }
  const auto ack_status = frame.data[0] >> 5;
  if (ack_status != 5 || frame.data[1] != expected_param_code) {
    error = "Encos CANFD get motor parameter feedback code mismatch";
    return false;
  }

  result = EncosAdminResult{};
  result.success = true;
  result.operation = EncosAdminOperation::GetMotorParam;
  result.motor_id = expected_motor_id;
  result.param_code = expected_param_code;
  result.feedback_code = ack_status;

  switch (expected_param_code) {
    case 1:
    case 2:
    case 3:
    case 4:
      if (frame.length != 6) {
        error = "Encos CANFD float parameter feedback length mismatch";
        return false;
      }
      result.value = readBigEndianFloat(&frame.data[2]);
      return true;
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 31:
      if (frame.length != 4) {
        error = "Encos CANFD uint16 parameter feedback length mismatch";
        return false;
      }
      result.value = static_cast<double>(readU16(frame.data[2], frame.data[3]));
      return true;
    case 22:
      if (frame.length != 4) {
        error = "Encos CANFD scaled parameter feedback length mismatch";
        return false;
      }
      result.value = static_cast<double>(readU16(frame.data[2], frame.data[3])) / 100.0;
      return true;
    case 23:
    case 24:
      if (frame.length != 6) {
        error = "Encos CANFD gain range feedback length mismatch";
        return false;
      }
      result.value_min = static_cast<double>(readU16(frame.data[2], frame.data[3]));
      result.value_max = static_cast<double>(readU16(frame.data[4], frame.data[5]));
      return true;
    case 25:
    case 26:
      if (frame.length != 6) {
        error = "Encos CANFD position/speed range feedback length mismatch";
        return false;
      }
      result.value_min = static_cast<double>(readI16(frame.data[2], frame.data[3])) / 100.0;
      result.value_max = static_cast<double>(readI16(frame.data[4], frame.data[5])) / 100.0;
      return true;
    case 27:
    case 28:
      if (frame.length != 6) {
        error = "Encos CANFD torque/current range feedback length mismatch";
        return false;
      }
      result.value_min = static_cast<double>(readI16(frame.data[2], frame.data[3])) / 10.0;
      result.value_max = static_cast<double>(readI16(frame.data[4], frame.data[5])) / 10.0;
      return true;
    case 32:
      if (frame.length != 6) {
        error = "Encos CANFD current loop feedback length mismatch";
        return false;
      }
      result.value_min = static_cast<double>(readU16(frame.data[2], frame.data[3])) / 10000.0;
      result.value_max = static_cast<double>(readU16(frame.data[4], frame.data[5])) / 10.0;
      return true;
    case 33:
    case 34:
      if (frame.length != 6) {
        error = "Encos CANFD loop feedback length mismatch";
        return false;
      }
      result.value_min = static_cast<double>(readU16(frame.data[2], frame.data[3])) / 100000.0;
      result.value_max = static_cast<double>(readU16(frame.data[4], frame.data[5])) / 100000.0;
      return true;
    case 37:
      if (frame.length != 3) {
        error = "Encos CANFD brake enable feedback length mismatch";
        return false;
      }
      result.value = frame.data[2];
      return true;
    default:
      if (frame.length >= 4) {
        result.value = static_cast<double>(readU16(frame.data[2], frame.data[3]));
        return true;
      }
      error = "Encos CANFD unsupported parameter feedback length";
      return false;
  }
}

}  // namespace robot_driver
