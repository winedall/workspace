#include "robot_driver/canfd/canfd_master.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>
#include <utility>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <rclcpp/rclcpp.hpp>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace robot_driver
{

namespace
{

static const auto kLogger = rclcpp::get_logger("canfd_master");
static rclcpp::Clock kThrottleClock(RCL_STEADY_TIME);

}  // namespace

// 析构时关闭所有 socket，保证异常退出或测试销毁时不会泄漏 fd。
CanfdMaster::~CanfdMaster()
{
  close();
}

// 打开 CANFD 总线配置中的所有 SocketCAN 接口。
//
// 该函数只负责打开 socket、启用 CANFD 帧、绑定接口；每个关节信箱由 configure() 建立。
bool CanfdMaster::open(const CanfdBusConfig & config)
{
  // open() 允许重复调用；先释放旧 socket，避免热重启或测试中 fd 泄漏。
  close();
  config_ = config;
  opened_ = false;
  configured_ = false;

  for (const auto & iface : config.interfaces) {
    BusRuntime bus;
    bus.bus = iface.bus;
    bus.name = iface.name;
    bus.fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (bus.fd < 0) {
      RCLCPP_ERROR(kLogger, "CANFD socket failed for %s: %s", iface.name.c_str(), std::strerror(errno));
      close();
      return false;
    }

    // SocketCAN 默认可能只接受 Classical CAN；显式打开 CANFD 帧支持。
    int enable_canfd = 1;
    (void)setsockopt(bus.fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd));

    sockaddr_can address{};
    address.can_family = AF_CAN;
    const int if_index = if_nametoindex(iface.name.c_str());
    if (if_index == 0) {
      RCLCPP_ERROR(kLogger, "CANFD interface not found: %s", iface.name.c_str());
      close();
      return false;
    }
    address.can_ifindex = if_index;
    if (bind(bus.fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
      RCLCPP_ERROR(kLogger, "CANFD bind failed for %s: %s", iface.name.c_str(), std::strerror(errno));
      close();
      return false;
    }

    // 在 bind() 之后设置发送缓冲区（内核会将其加倍，实际值可能翻倍）
    int sndbuf_size = 65536;  // 建议 64KB 起
    if (setsockopt(bus.fd, SOL_SOCKET, SO_SNDBUF, &sndbuf_size, sizeof(sndbuf_size)) < 0) {
      RCLCPP_ERROR(kLogger, "setsockopt SO_SNDBUF failed for %s: %s", iface.name.c_str(), std::strerror(errno));
      close();
      return false;
    }

    // 验证实际生效值（内核通常返回设置值的 2 倍）
    int actual_size;
    socklen_t len = sizeof(actual_size);
    getsockopt(bus.fd, SOL_SOCKET, SO_SNDBUF, &actual_size, &len);
    RCLCPP_INFO(kLogger, "SO_SNDBUF actual for %s is: %d bytes", iface.name.c_str(), actual_size);

    // 保存 bus 运行时状态
    buses_.push_back(bus);
  }

  opened_ = true;
  return true;
}

// 根据硬件关节映射建立输入/输出信箱。
//
// 信箱顺序与 joints 配置顺序一致，避免周期收发按 bus 数量组织导致反馈遗漏。
bool CanfdMaster::configure(const std::vector<HardwareJointConfig> & joints)
{
  std::lock_guard<std::mutex> lock(mailbox_mutex_);
  joints_.clear();
  joints_.reserve(joints.size());
  for (const auto & joint : joints) {
    const auto bus_index = findBusIndex(joint.canfd.bus);
    if (bus_index >= buses_.size()) {
      RCLCPP_ERROR(kLogger, "CANFD joint %s references unopened bus %d", joint.name.c_str(), joint.canfd.bus);
      configured_ = false;
      return false;
    }
    // 每个关节建立一对信箱；周期发送和接收都按该数组的关节数量推进。
    JointRuntime runtime;
    runtime.bus = joint.canfd.bus;
    runtime.motor_id = joint.motor_id;
    runtime.bus_index = bus_index;
    joints_.push_back(runtime);
  }
  configured_ = true;
  return true;
}

// 执行一次 CANFD 数据交换。
//
// 顺序参考 tport CANFD 后端：先批量发送所有非空输出信箱，再在有界窗口内
// 尽量吸收各 bus 已到达的反馈。CANFD 回包允许跨周期迟到；本函数不阻塞等待
// 本轮所有电机收齐，只把已收到的反馈按 bus 与 CAN id 写回对应输入信箱。
bool CanfdMaster::exchangeProcessData()
{
  if (!opened_ || !configured_) {
    RCLCPP_ERROR_THROTTLE(kLogger, kThrottleClock, 1000, "CANFD master is not ready");
    return false;
  }

  std::size_t sent_count = 0;
  for (auto & joint : joints_) {
    joint.awaiting = false;
    joint.received = false;
    const auto & frame = joint.output;
    if (frame.length == 0) {
      // 空帧表示该关节本周期没有待发数据。
      continue;
    }
    const auto & bus = buses_[joint.bus_index];
    canfd_frame raw{};
    raw.can_id = frame.id;
    raw.flags = CANFD_BRS;
    raw.len = std::min<std::uint8_t>(frame.length, kCanfdMaxDataLength);
    std::copy_n(frame.data.begin(), raw.len, raw.data);
    if (send(bus.fd, &raw, sizeof(raw), MSG_DONTWAIT) < 0) {
      RCLCPP_ERROR_THROTTLE(kLogger, kThrottleClock, 1000,
        "1 - CANFD send failed on %s: %s", bus.name.c_str(), std::strerror(errno));
      continue;
    //   return false;
    }
    joint.awaiting = true;
    ++sent_count;
  }
  if (sent_count == 0) {
    return true;
  }

  const int timeout_ms = std::max(0, static_cast<int>(std::ceil(config_.poll_timeout_ms)));
  const auto receive_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  std::vector<pollfd> pfds;
  pfds.reserve(buses_.size());
  for (const auto & bus : buses_) {
    pfds.push_back(pollfd{bus.fd, POLLIN, 0});
  }

  std::size_t received_count = 0;
  std::vector<std::size_t> received_by_bus(buses_.size(), 0);
  auto drain_bus = [&](std::size_t bus_index) {
    const auto & bus = buses_[bus_index];
    while (true) {
      canfd_frame raw{};
      if (recv(bus.fd, &raw, sizeof(raw), MSG_DONTWAIT) < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          break;
        }
        RCLCPP_WARN_THROTTLE(kLogger, kThrottleClock, 1000,
          "CANFD recv failed on %s: %s", bus.name.c_str(), std::strerror(errno));
        break;
      }

    //   std::this_thread::sleep_for(std::chrono::microseconds(50));

      // 反馈按接收 socket 的 bus 和 CAN id 回填，避免不同总线同 ID 的电机互相覆盖。
      auto * joint = findJoint(bus.bus, static_cast<int>(raw.can_id & CAN_SFF_MASK));
      if (joint == nullptr) {
        continue;
      }
      joint->input.id = raw.can_id & CAN_SFF_MASK;
      joint->input.length = std::min<std::uint8_t>(raw.len, kCanfdMaxDataLength);
      std::copy_n(raw.data, joint->input.length, joint->input.data.begin());

      // 只把本周期实际发送过的关节计入本轮诊断；迟到帧仍会刷新 input。
      if (joint->awaiting && !joint->received) {
        joint->received = true;
        ++received_count;
        ++received_by_bus[bus_index];
      }
    }
  };
  const auto bus_summary = [&]() {
    std::ostringstream stream;
    for (std::size_t index = 0; index < buses_.size() && index < received_by_bus.size(); ++index) {
      stream << " bus" << buses_[index].bus << "=" << received_by_bus[index];
    }
    return stream.str();
  };

  while (received_count < sent_count) {
    int wait_ms = 0;
    if (timeout_ms > 0) {
      const auto remaining = receive_deadline - std::chrono::steady_clock::now();
      if (remaining <= std::chrono::steady_clock::duration::zero()) {
        break;
      }
      const auto remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(remaining).count();
      wait_ms = static_cast<int>(std::max<long long>(1, remaining_ms));
    }

    const int ready = poll(pfds.data(), pfds.size(), wait_ms);
    if (ready < 0) {
      if (errno == EINTR) {
        continue;
      }
      RCLCPP_ERROR_THROTTLE(kLogger, kThrottleClock, 1000,
        "CANFD poll failed on timeout %d ms: %s", wait_ms, std::strerror(errno));
      break;
    }
    if (ready == 0) {
      break;
    }

    for (std::size_t bus_index = 0; bus_index < pfds.size(); ++bus_index) {
      if ((pfds[bus_index].revents & POLLIN) == 0) {
        continue;
      }
      drain_bus(bus_index);
    }
  }

  // 参考实现在窗口结束后再非阻塞收割一次，避免边界时刻到达的反馈滞留到下一轮。
  for (std::size_t bus_index = 0; bus_index < buses_.size(); ++bus_index) {
    drain_bus(bus_index);
  }
  if (received_count < sent_count) {
    const auto summary = bus_summary();
    RCLCPP_DEBUG_THROTTLE(kLogger, kThrottleClock, 1000,
      "CANFD cycle partial feedback after %d ms, received=%zu/%zu,%s",
      timeout_ms, received_count, sent_count, summary.c_str());
  }
  return true;
}

// 在指定 bus 上执行一帧广播/配置事务。
bool CanfdMaster::exchangeBusFrame(int bus_id, const CanfdFrame & request, CanfdFrame & response, int timeout_us)
{
  if (!opened_ || !configured_) {
    RCLCPP_ERROR_THROTTLE(kLogger, kThrottleClock, 1000, "CANFD master is not ready");
    return false;
  }

  const auto bus_iter = std::find_if(buses_.begin(), buses_.end(), [bus_id](const BusRuntime & bus) {
    return bus.bus == bus_id;
  });
  
  if (bus_iter == buses_.end()) {
    RCLCPP_ERROR(kLogger, "CANFD bus not configured: %d", bus_id);
    return false;
  }
  if (request.length == 0) {
    RCLCPP_ERROR(kLogger, "CANFD bus transaction request is empty");
    return false;
  }

  canfd_frame raw{};
  raw.can_id = request.id;
  raw.flags = CANFD_BRS;
  raw.len = std::min<std::uint8_t>(request.length, kCanfdMaxDataLength);
  std::copy_n(request.data.begin(), raw.len, raw.data);
  if (send(bus_iter->fd, &raw, sizeof(raw), MSG_DONTWAIT) < 0) {
    RCLCPP_ERROR_THROTTLE(kLogger, kThrottleClock, 1000,
      "2 - CANFD send failed on %s: %s", bus_iter->name.c_str(), std::strerror(errno));
    return false;
  }

  pollfd pfd{bus_iter->fd, POLLIN, 0};
  const int timeout_ms = std::max(0, (timeout_us + 999) / 1000);
  if (poll(&pfd, 1, timeout_ms) <= 0 || (pfd.revents & POLLIN) == 0) {
    RCLCPP_ERROR_THROTTLE(kLogger, kThrottleClock, 1000,
      "CANFD bus transaction timed out on %s",
      bus_iter->name.c_str());
    return false;
  }

  canfd_frame reply{};
  if (recv(bus_iter->fd, &reply, sizeof(reply), MSG_DONTWAIT) < 0) {
    RCLCPP_ERROR_THROTTLE(kLogger, kThrottleClock, 1000,
      "CANFD recv failed on %s: %s", bus_iter->name.c_str(), std::strerror(errno));
    return false;
  }

  response = CanfdFrame{};
  response.id = reply.can_id & CAN_SFF_MASK;
  response.length = std::min<std::uint8_t>(reply.len, kCanfdMaxDataLength);
  std::copy_n(reply.data, response.length, response.data.begin());
  return true;
}

// 查询逻辑 CANFD 总线号在 buses_ 中的下标。
//
// configure() 建立关节信箱时会把 bus_index 缓存下来，后续发送即可直接定位 socket。
// 未找到时返回 buses_.size()，由调用方统一判断为非法 bus。
std::size_t CanfdMaster::findBusIndex(int bus_id) const
{
  for (std::size_t index = 0; index < buses_.size(); ++index) {
    if (buses_[index].bus == bus_id) {
      return index;
    }
  }
  return buses_.size();
}

// 根据接收帧所属 bus 和 CAN id 查找对应关节信箱。
//
// CAN id 对应电机 motor_id；同一 bus 内 motor_id 已由配置校验保证唯一，因此线性查找
// 即可明确定位到唯一关节。未配置的回包返回 nullptr，由接收流程忽略。
CanfdMaster::JointRuntime * CanfdMaster::findJoint(int bus_id, int motor_id)
{
  for (auto & joint : joints_) {
    if (joint.bus == bus_id && joint.motor_id == motor_id) {
      return &joint;
    }
  }
  return nullptr;
}

// 返回指定关节输出信箱的原始字节地址。
//
// 该接口模拟 EthercatMaster 的 buffer 风格，方便 MixBackend 注入 fake master 做单元测试。
std::uint8_t * CanfdMaster::outputBuffer(std::size_t joint_index)
{
  if (joint_index >= joints_.size()) {
    return nullptr;
  }
  return reinterpret_cast<std::uint8_t *>(&joints_[joint_index].output);
}

// 返回指定关节输出信箱的字节数。
std::size_t CanfdMaster::outputBufferSize(std::size_t joint_index) const
{
  return joint_index >= joints_.size() ? 0 : sizeof(CanfdFrame);
}

// 设置指定关节输出信箱的当前帧。
bool CanfdMaster::setOutputFrame(std::size_t joint_index, const CanfdFrame & frame)
{
  std::lock_guard<std::mutex> lock(mailbox_mutex_);
  if (joint_index >= joints_.size()) {
    return false;
  }
  joints_[joint_index].output = frame;
  return true;
}

// 返回指定关节输入信箱的原始字节地址。
const std::uint8_t * CanfdMaster::inputBuffer(std::size_t joint_index) const
{
  if (joint_index >= joints_.size()) {
    return nullptr;
  }
  return reinterpret_cast<const std::uint8_t *>(&joints_[joint_index].input);
}

// 返回指定关节输入信箱的字节数。
std::size_t CanfdMaster::inputBufferSize(std::size_t joint_index) const
{
  return joint_index >= joints_.size() ? 0 : sizeof(CanfdFrame);
}

// 获取指定关节输入信箱的当前帧。
bool CanfdMaster::getInputFrame(std::size_t joint_index, CanfdFrame & frame) const
{
  std::lock_guard<std::mutex> lock(mailbox_mutex_);
  if (joint_index >= joints_.size()) {
    return false;
  }
  frame = joints_[joint_index].input;
  return true;
}

// 关闭所有已打开的 SocketCAN fd，并清理运行状态。
void CanfdMaster::close()
{
  std::lock_guard<std::mutex> lock(mailbox_mutex_);
  for (auto & bus : buses_) {
    if (bus.fd >= 0) {
      ::close(bus.fd);
      bus.fd = -1;
    }
  }
  buses_.clear();
  joints_.clear();
  opened_ = false;
  configured_ = false;
}

}  // namespace robot_driver
