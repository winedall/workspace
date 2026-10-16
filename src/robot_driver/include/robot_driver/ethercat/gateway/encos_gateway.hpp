#ifndef ROBOT_DRIVER__ETHERCAT_GATEWAY__ENCOS_GATEWAY_HPP_
#define ROBOT_DRIVER__ETHERCAT_GATEWAY__ENCOS_GATEWAY_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "robot_driver/canfd/canfd_frame.hpp"

namespace robot_driver
{

// Encos EtherCAT 网关只负责 CAN 帧与 EtherCAT PDO 数据区之间的搬运。
// 厂家命令的编码、反馈解析等协议语义由 EncosProtocol 处理。
class EncosEthercatGateway
{
public:
  // 单个 Encos EtherCAT 从站 PDO 中固定承载的 CAN 报文槽数量。
  static constexpr int kFramesPerSlave = 6;

  // 将一帧 CAN/CAN-FD 报文写入指定 passage 对应的 PDO 输出槽。
  // passage 使用配置文件中的 1 基编号，函数内部会转换为数组下标。
  bool writeFrame(
    int passage,
    const CanfdFrame & frame,
    std::uint8_t * output_buffer,
    std::size_t output_size,
    std::string & error) const;

  // 从指定 passage 对应的 PDO 输入槽读取一帧 CAN/CAN-FD 报文。
  // 这里只恢复报文 id、长度和数据区，不判断报文是否属于某个命令。
  bool readFrame(
    int passage,
    const std::uint8_t * input_buffer,
    std::size_t input_size,
    CanfdFrame & frame,
    std::string & error) const;

  // 清空指定 passage 对应的 PDO 输出槽，用于停止继续发送上一周期命令。
  bool clearFrame(
    int passage,
    std::uint8_t * output_buffer,
    std::size_t output_size,
    std::string & error) const;
};

}  // namespace robot_driver

#endif  // ROBOT_DRIVER__ETHERCAT_GATEWAY__ENCOS_GATEWAY_HPP_
