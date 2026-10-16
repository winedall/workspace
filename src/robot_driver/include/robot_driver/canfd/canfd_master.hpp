#ifndef ROBOT_DRIVER__CANFD__CANFD_MASTER_HPP_
#define ROBOT_DRIVER__CANFD__CANFD_MASTER_HPP_

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "robot_driver/canfd/canfd_frame.hpp"
#include "robot_driver/hardware_config.hpp"

namespace robot_driver
{

// SocketCAN CANFD 主站封装。
//
// 接口命名刻意靠近 EthercatMaster：open/configure/exchangeProcessData/close。
// MixBackend 因此可以用和现有 EtherCAT backend 类似的生命周期来组织 CANFD 通讯。
class CanfdMaster
{
public:
  virtual ~CanfdMaster();

  // 打开配置中的所有 canX 接口。每一路 CANFD 对应一个 raw socket fd。
  virtual bool open(const CanfdBusConfig & config);
  // 根据硬件配置中的 joints 顺序建立每个关节的输入/输出信箱。
  virtual bool configure(const std::vector<HardwareJointConfig> & joints);
  // 执行一个 CANFD 周期：按关节数量发送输出信箱，并在有界窗口内吸收反馈。
  virtual bool exchangeProcessData();
  // 在指定 bus 上发送一帧，并接收该 bus 的任意一帧响应。
  //
  // 该接口用于查询电机 ID 这类广播配置命令；广播响应不能依赖配置里的 motor_id
  // 进行信箱匹配，因此独立于周期控制的关节输入信箱。
  virtual bool exchangeBusFrame(int bus, const CanfdFrame & request, CanfdFrame & response, int timeout_us);
  // 返回指定关节输出信箱的原始缓冲区，关节下标与 configure(joints) 的顺序一致。
  virtual std::uint8_t * outputBuffer(std::size_t joint_index);
  virtual std::size_t outputBufferSize(std::size_t joint_index) const;
  virtual bool setOutputFrame(std::size_t joint_index, const CanfdFrame & frame);
  // 返回指定关节输入信箱的原始缓冲区，关节下标与 configure(joints) 的顺序一致。
  virtual const std::uint8_t * inputBuffer(std::size_t joint_index) const;
  virtual std::size_t inputBufferSize(std::size_t joint_index) const;
  virtual bool getInputFrame(std::size_t joint_index, CanfdFrame & frame) const;
  // 关闭所有 socket 并清理运行态。
  virtual void close();

private:
  struct BusRuntime
  {
    // 配置中的逻辑总线号，用于和 joints[*].canfd.bus 关联。
    int bus{0};
    // Linux SocketCAN 接口名，例如 can0/can1。
    std::string name;
    // 该接口对应的 raw CAN socket fd。
    int fd{-1};
  };

  struct JointRuntime
  {
    // 该关节所在的逻辑 CANFD 总线号，用于回包匹配。
    int bus{0};
    // 该关节电机 ID，同一 bus 内由配置校验保证唯一。
    int motor_id{0};
    // buses_ 中的下标，发送时可直接找到对应 socket。
    std::size_t bus_index{0};
    // 该关节本周期是否已经成功发送输出帧，用于决定是否等待反馈。
    bool awaiting{false};
    // 该关节本周期是否已经收到反馈，用于按关节数量收齐。
    bool received{false};
    // 厂家 protocol 写入的待发送 CANFD 帧。
    CanfdFrame output;
    // exchangeProcessData() 收到并回填的反馈 CANFD 帧。
    CanfdFrame input;
  };

  // 查询逻辑 bus 在 buses_ 中的下标；不存在时返回 buses_.size()。
  std::size_t findBusIndex(int bus_id) const;
  // 根据接收 socket 所属 bus 和 CAN id 查找关节信箱。
  JointRuntime * findJoint(int bus_id, int motor_id);

  CanfdBusConfig config_;
  std::vector<BusRuntime> buses_;
  std::vector<JointRuntime> joints_;
  mutable std::mutex mailbox_mutex_;
  bool opened_{false};
  bool configured_{false};
};

}  // namespace robot_driver

#endif  // ROBOT_DRIVER__CANFD__CANFD_MASTER_HPP_
