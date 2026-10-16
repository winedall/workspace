#ifndef ROBOT_DRIVER__PROTOCOL__ENCOS_PROTOCOL_HPP_
#define ROBOT_DRIVER__PROTOCOL__ENCOS_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "robot_driver/canfd/canfd_frame.hpp"
#include "robot_driver/hardware_config.hpp"
#include "robot_interfaces/msg/joint_command.hpp"

namespace robot_driver
{

// 协议层解码后的单关节反馈。
//
// 字段已经转换到机器人统一关节坐标系：position/velocity/effort 可直接填入
// robot_interfaces::msg::JointState。电机驱动底层故障码保留在 motor_error 中，
// 由 backend 再映射到统一 joint_error。
struct EncosJointFeedback
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  std::uint8_t motor_error{0};
  std::uint8_t motor_temp{0};
  std::uint8_t mos_temp{0};
};

// Encos 低频管理命令类型。
//
// 这些命令属于厂家私有维护语义，不是所有 EtherCAT 厂商都会支持，所以只放在
// EncosProtocol/EncosBackend 内部，不上提到通用 EthercatProtocol。
enum class EncosAdminOperation
{
  GetMotorId,
  SetMotorId,
  SetMotorZero,
  GetMotorParam,
};

// 一次 Encos 管理事务的解析结果。
//
// value 用于单值参数；value_min/value_max 用于厂家协议返回范围值的参数。
struct EncosAdminResult
{
  bool success{false};                                             // 厂家反馈是否表示本事务完成。
  std::string message;                                              // 错误或状态信息
  EncosAdminOperation operation{EncosAdminOperation::GetMotorId};   // 对应的维护命令类型
  std::uint16_t motor_id{0};                                       // 反馈中的电机 ID
  std::uint16_t new_motor_id{0};                                   // 设置 ID 时回传请求的新 ID
  std::uint8_t param_code{0};                                      // 参数读取结果对应的参数编号
  std::uint8_t feedback_code{0};                                   // 厂家反馈码或 ack 状态
  double value{0.0};                                               // 参数单值
  double value_min{0.0};                                           // 参数下限
  double value_max{0.0};                                           // 参数上限
};

// Encos 厂家协议编解码器。
//
// 负责 direction/offset/kt 换算，以及 Encos 力位混控 CAN 帧打包。它不打开网卡、
// 不判断系统状态，也不解释 EtherCAT PDO。EtherCAT 网关封装由 ethercat/gateway 层负责。
class EncosProtocol
{
public:
  // 返回协议名称，主要用于日志、诊断和后续 factory 扩展。
  std::string name() const;

  // 编码运动命令。
  bool encodeMotion(const HardwareJointConfig & hardware_joint, const robot_interfaces::msg::JointCommand & command, CanfdFrame & frame, std::string & error) const;
  // 编码读取电机 ID 帧。
  bool encodeQueryId(CanfdFrame & frame, std::string & error) const;
  // 编码设置电机 ID 帧。
  bool encodeModifyId(std::uint16_t motor_id, std::uint16_t new_motor_id, CanfdFrame & frame, std::string & error) const;
  // 编码设置零点帧。
  bool encodeSetZero(std::uint16_t motor_id, CanfdFrame & frame, std::string & error) const;
  // 编码参数读取帧。
  bool encodeQueryParam(std::uint16_t motor_id, std::uint8_t param_code, CanfdFrame & frame, std::string & error) const;

  // 解码反馈帧，并转换成统一关节空间反馈。
  bool decodeFeedback(const HardwareJointConfig & hardware_joint, const CanfdFrame & frame, EncosJointFeedback & feedback, std::string & error) const;
  // 解码电机 ID 反馈。
  bool decodeQueryId(const CanfdFrame & frame, EncosAdminResult & result, std::string & error) const;
  // 解码设置 ID 反馈。
  bool decodeModifyId(std::uint16_t expected_motor_id, std::uint16_t requested_new_motor_id, const CanfdFrame & frame, EncosAdminResult & result, std::string & error) const;
  // 解码设置零点反馈。
  bool decodeSetZero(std::uint16_t expected_motor_id, const CanfdFrame & frame, EncosAdminResult & result, std::string & error) const;
  // 解码参数读取反馈。
  bool decodeQueryParam(std::uint16_t expected_motor_id, std::uint8_t expected_param_code, const CanfdFrame & frame, EncosAdminResult & result, std::string & error) const;
};

}  // namespace robot_driver

#endif  // ROBOT_DRIVER__PROTOCOL__ENCOS_PROTOCOL_HPP_
