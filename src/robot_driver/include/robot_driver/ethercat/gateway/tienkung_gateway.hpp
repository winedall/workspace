#ifndef ROBOT_DRIVER__ETHERCAT_GATEWAY__TIENKUNG_GATEWAY_HPP_
#define ROBOT_DRIVER__ETHERCAT_GATEWAY__TIENKUNG_GATEWAY_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "robot_driver/canfd/canfd_frame.hpp"
#include "robot_driver/hardware_config.hpp"

namespace robot_driver
{

// Tienkung EtherCAT 网关只封装 EtherCAT PDO 的槽位布局。
// 它不关心具体厂家命令含义，只在 CanfdFrame 和 PDO buffer 之间转换。
class TienkungEthercatGateway
{
public:
  // 按关节配置中的 bus/slot，将一帧 CAN-FD 报文写入对应 PDO 输出槽。
  bool writeFrame(
    const HardwareJointConfig & joint,
    const CanfdFrame & frame,
    std::uint8_t * output_buffer,
    std::size_t output_size,
    std::string & error) const;

  // 按关节配置中的 bus/slot，从对应 PDO 输入槽读取一帧 CAN-FD 报文。
  bool readFrame(
    const HardwareJointConfig & joint,
    const std::uint8_t * input_buffer,
    std::size_t input_size,
    CanfdFrame & frame,
    std::string & error) const;

  // 清空关节对应的 PDO 输出槽，避免旧命令在后续周期被重复发送。
  bool clearFrame(
    const HardwareJointConfig & joint,
    std::uint8_t * output_buffer,
    std::size_t output_size,
    std::string & error) const;

  // 写入广播命令槽。当前 Tienkung 网关约定广播帧使用 can_fd0[0]。
  bool writeBroadcastFrame(
    const CanfdFrame & frame,
    std::uint8_t * output_buffer,
    std::size_t output_size,
    std::string & error) const;

  // 在两个 CAN-FD 总线的输入槽中查找指定命令帧。
  // use_expected_id 为 true 时同时匹配 CAN id，适合查询类命令回包。
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

#endif  // ROBOT_DRIVER__ETHERCAT_GATEWAY__TIENKUNG_GATEWAY_HPP_
