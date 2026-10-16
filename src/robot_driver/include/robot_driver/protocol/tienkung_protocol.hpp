#ifndef ROBOT_DRIVER__PROTOCOL__TIENKUNG_PROTOCOL_HPP_
#define ROBOT_DRIVER__PROTOCOL__TIENKUNG_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "robot_driver/canfd/canfd_frame.hpp"
#include "robot_driver/hardware_config.hpp"
#include "robot_interfaces/msg/joint_command.hpp"

namespace robot_driver
{

// 天工协议层解码后的单关节反馈。
//
// position/velocity/effort 已经换算到统一关节坐标系，可直接写入
// robot_interfaces::msg::JointState；mode、motor_error、温度和母线电压保留
// 厂家反馈原始含义，供 backend 继续做故障映射和诊断展示。
struct TienkungJointFeedback
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  std::uint16_t motor_error{0};
  std::uint8_t mode{0};
  std::int8_t motor_temp{0};
  std::int8_t mos_temp{0};
  double bus_voltage{0.0};
};

// 天工管理事务解析结果。
//
// 该结构承载天工低频配置/查询命令的协议返回值，和 TienkungProtocol 的
// encode/decode 接口同属协议层；backend 只负责组织事务流程和超时重试。
struct TienkungAdminResult
{
  bool success{false};
  std::string message;
  std::uint16_t motor_id{0};
  std::uint8_t feedback_code{0};
  double zero_position_rad{0.0};
  std::uint8_t new_motor_id{0};
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  std::uint16_t motor_error{0};
  float motor_temp{0.0f};
  float mos_temp{0.0f};
};

// 天工厂家 CAN-FD 协议编解码器。
//
// 本类只处理 CAN-FD 帧的厂家协议字段和单位换算，不访问 EtherCAT PDO，
// 也不管理 SocketCAN/IGH/SOEM 等通讯资源。EtherCAT 场景由
// TienkungEthercatGateway 负责把 CanfdFrame 放入 PDO 槽位。
class TienkungProtocol
{
public:
  // 返回协议名称，便于日志和测试识别。
  std::string name() const;

  // 把统一关节命令编码成天工 CANFD 运动控制帧。
  bool encodeMotion(const HardwareJointConfig & hardware_joint, const robot_interfaces::msg::JointCommand & command, std::uint8_t counter, CanfdFrame & frame, std::string & error) const;
  // 编码使能帧。
  bool encodeEnable(const HardwareJointConfig & hardware_joint, CanfdFrame & frame, std::string & error) const;
  // 编码失能帧。
  bool encodeDisable(const HardwareJointConfig & hardware_joint, CanfdFrame & frame, std::string & error) const;
  // 编码置零帧。
  bool encodeSetZero(const HardwareJointConfig & hardware_joint, CanfdFrame & frame, std::string & error) const;
  // 编码修改 ID 帧。
  bool encodeModifyId(const HardwareJointConfig & hardware_joint, std::uint8_t new_id, CanfdFrame & frame, std::string & error) const;
  // 编码广播查询 ID 帧。
  bool encodeQueryId(CanfdFrame & frame, std::string & error) const;
  // 编码广播重置 ID 帧。
  bool encodeResetId(CanfdFrame & frame, std::string & error) const;

  // 解码默认状态反馈帧，并转换成统一关节空间反馈。
  bool decodeFeedback(const HardwareJointConfig & hardware_joint, const CanfdFrame & frame, TienkungJointFeedback & feedback, std::string & error) const;
  // 解码启动/停止应答。
  bool decodeStartStop(const HardwareJointConfig & hardware_joint, const CanfdFrame & frame, bool & enabled, std::string & error) const;
  // 解码默认状态反馈帧，并填充管理事务结果。
  bool decodeQueryStatus(const HardwareJointConfig & hardware_joint, const CanfdFrame & frame, TienkungAdminResult & result, std::string & error) const;
  // 解码置零应答。
  bool decodeSetZero(const HardwareJointConfig & hardware_joint, const CanfdFrame & frame, TienkungAdminResult & result, std::string & error) const;
  // 解码修改 ID 应答。
  bool decodeModifyId(const HardwareJointConfig & hardware_joint, const CanfdFrame & frame, TienkungAdminResult & result, std::string & error) const;
  // 解码广播查询 ID 响应。
  bool decodeQueryId(const CanfdFrame & frame, TienkungAdminResult & result, std::string & error) const;
  // 解码广播重置 ID 响应。
  bool decodeResetId(const CanfdFrame & frame, TienkungAdminResult & result, std::string & error) const;
};

}  // namespace robot_driver

#endif
