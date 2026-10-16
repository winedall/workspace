#ifndef ROBOT_DRIVER__ETHERCAT_GATEWAY__CANFD_GATEWAY_HPP_
#define ROBOT_DRIVER__ETHERCAT_GATEWAY__CANFD_GATEWAY_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "robot_driver/canfd/canfd_frame.hpp"
#include "robot_driver/hardware_config.hpp"

namespace robot_driver
{

// 中性的 CANFD 网关槽位适配器。
//
// 文件已经位于 ethercat/gateway 目录下，因此类名保持简短。它只负责把通用
// CanfdFrame 放入 400 字节 PDO 中的 CAN-FD 槽位，或从槽位读回 CanfdFrame；
// 不理解 Encos/Tienkung 的命令字和数据含义。
class CanfdGateway
{
public:
  bool writeFrame(
    const HardwareJointConfig & joint,
    const CanfdFrame & frame,
    std::uint8_t * output_buffer,
    std::size_t output_size,
    std::string & error) const;

  bool readFrame(
    const HardwareJointConfig & joint,
    const std::uint8_t * input_buffer,
    std::size_t input_size,
    CanfdFrame & frame,
    std::string & error) const;

  bool clearFrame(
    const HardwareJointConfig & joint,
    std::uint8_t * output_buffer,
    std::size_t output_size,
    std::string & error) const;

  bool findFrame(
    std::uint8_t command,
    std::uint32_t expected_id,
    bool use_expected_id,
    const std::uint8_t * input_buffer,
    std::size_t input_size,
    CanfdFrame & frame,
    std::string & error) const;
};

}  // namespace robot_driver

#endif  // ROBOT_DRIVER__ETHERCAT_GATEWAY__CANFD_GATEWAY_HPP_
