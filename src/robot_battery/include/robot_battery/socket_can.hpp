#ifndef ROBOT_BATTERY__SOCKET_CAN_HPP_
#define ROBOT_BATTERY__SOCKET_CAN_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace robot_battery
{

// CAN 数据帧的轻量封装，隐藏 Linux SocketCAN 的帧结构差异。
struct CanFrame
{
  uint32_t id{0};
  bool extended{false};
  bool fd{false};
  bool brs{false};
  std::size_t size{0};
  std::array<uint8_t, 64> data{};
};

// SocketCAN 设备封装，负责打开 CAN 口并收发原始帧。
class SocketCan
{
public:
  SocketCan() = default;
  ~SocketCan();

  SocketCan(const SocketCan &) = delete;
  SocketCan & operator=(const SocketCan &) = delete;

  // 打开指定 SocketCAN 接口，例如 can0。
  bool open(const std::string & device, std::string & error);

  // 关闭当前 CAN socket。
  void close();

  // 返回当前 socket 是否处于打开状态。
  bool isOpen() const;

  // 写入一帧 CAN 或 CANFD 数据。
  bool writeFrame(const CanFrame & frame, std::string & error);

  // 读取一帧 CAN 或 CANFD 数据；超时返回 false 且 error 为空。
  bool readFrame(CanFrame & frame, int timeout_ms, std::string & error);

private:
  int fd_{-1};
};

}  // namespace robot_battery

#endif  // ROBOT_BATTERY__SOCKET_CAN_HPP_
