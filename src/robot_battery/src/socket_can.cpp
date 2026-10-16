#include "robot_battery/socket_can.hpp"

#include <cerrno>
#include <cstring>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

namespace robot_battery
{

// 析构时关闭 socket，避免节点退出时泄漏文件描述符。
SocketCan::~SocketCan()
{
  close();
}

// 打开指定 SocketCAN 网络接口，并启用 CANFD 帧接收能力。
bool SocketCan::open(const std::string & device, std::string & error)
{
  close();

  fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    error = std::strerror(errno);
    return false;
  }

  ifreq ifr {};
  std::strncpy(ifr.ifr_name, device.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    error = std::strerror(errno);
    close();
    return false;
  }

  sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    error = std::strerror(errno);
    close();
    return false;
  }

  int enable_canfd = 1;
  if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0) {
    error = std::strerror(errno);
    close();
    return false;
  }

  error.clear();
  return true;
}

// 关闭已打开的 CAN socket。
void SocketCan::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

// 返回当前 CAN socket 是否可用。
bool SocketCan::isOpen() const
{
  return fd_ >= 0;
}

// 将轻量 CanFrame 转换为 Linux CAN/CANFD 帧并写入 socket。
bool SocketCan::writeFrame(const CanFrame & frame, std::string & error)
{
  if (fd_ < 0) {
    error = "CAN device is not open";
    return false;
  }

  canfd_frame raw {};
  raw.can_id = frame.id;
  if (frame.extended) {
    raw.can_id |= CAN_EFF_FLAG;
  }
  raw.len = static_cast<__u8>(frame.size > CANFD_MAX_DLEN ? CANFD_MAX_DLEN : frame.size);
  if (frame.fd) {
    raw.flags |= CANFD_FDF;
    if (frame.brs) {
      raw.flags |= CANFD_BRS;
    }
  } else if (raw.len > CAN_MAX_DLEN) {
    raw.len = CAN_MAX_DLEN;
  }

  std::memcpy(raw.data, frame.data.data(), raw.len);
  const auto write_size = frame.fd ? sizeof(canfd_frame) : sizeof(can_frame);
  const auto written = ::write(fd_, &raw, write_size);
  if (written != static_cast<ssize_t>(write_size)) {
    error = std::strerror(errno);
    return false;
  }

  error.clear();
  return true;
}

// 从 socket 读取一帧 CAN/CANFD 数据，超时时不视为错误。
bool SocketCan::readFrame(CanFrame & frame, int timeout_ms, std::string & error)
{
  if (fd_ < 0) {
    error = "CAN device is not open";
    return false;
  }

  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(fd_, &readfds);

  timeval timeout {};
  timeout.tv_sec = timeout_ms / 1000;
  timeout.tv_usec = (timeout_ms % 1000) * 1000;
  const int ret = select(fd_ + 1, &readfds, nullptr, nullptr, &timeout);
  if (ret == 0) {
    error.clear();
    return false;
  }
  if (ret < 0) {
    error = std::strerror(errno);
    return false;
  }

  canfd_frame raw {};
  const auto nbytes = ::read(fd_, &raw, sizeof(raw));
  if (nbytes < static_cast<ssize_t>(sizeof(can_frame))) {
    error = nbytes < 0 ? std::strerror(errno) : "incomplete CAN frame";
    return false;
  }

  frame = CanFrame{};
  frame.extended = (raw.can_id & CAN_EFF_FLAG) != 0;
  frame.id = raw.can_id & (frame.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
  frame.fd = (raw.flags & CANFD_FDF) != 0;
  frame.brs = (raw.flags & CANFD_BRS) != 0;
  frame.size = raw.len > frame.data.size() ? frame.data.size() : raw.len;
  std::memcpy(frame.data.data(), raw.data, frame.size);

  error.clear();
  return true;
}

}  // namespace robot_battery
