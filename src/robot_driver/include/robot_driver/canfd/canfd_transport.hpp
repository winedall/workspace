#ifndef ROBOT_DRIVER__CANFD__CANFD_TRANSPORT_HPP_
#define ROBOT_DRIVER__CANFD__CANFD_TRANSPORT_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "robot_driver/canfd/canfd_frame.hpp"
#include "robot_driver/canfd/canfd_master.hpp"
#include "robot_driver/ethercat/ethercat_master.hpp"
#include "robot_driver/ethercat/gateway/canfd_gateway.hpp"
#include "robot_driver/hardware_config.hpp"

namespace robot_driver
{

// MixBackend 使用的 CANFD 帧传输接口。
//
// 厂家协议层只产生/消费 CanfdFrame；具体是直接走 SocketCAN，还是通过 EtherCAT
// 网关 PDO 转发，统一封装在这个接口后面。
class CanfdTransport
{
public:
  virtual ~CanfdTransport() = default;

  virtual bool open(const HardwareConfig & config) = 0;
  virtual bool configure(const HardwareConfig & config) = 0;
  // 按 hardware.joints 的顺序写入某个关节的待发送帧。
  //
  // SocketCAN 实现会写入 CanfdMaster 的关节信箱；EtherCAT gateway 实现会把同一帧
  // 写入 joint.gateway 指向的 PDO 槽位。
  virtual bool setOutputFrame(std::size_t joint_index, const CanfdFrame & frame) = 0;
  // 按 hardware.joints 的顺序读取某个关节最近一次反馈帧。
  virtual bool getInputFrame(std::size_t joint_index, CanfdFrame & frame) const = 0;
  // 执行一次底层 process data 交换。MixBackend 在交换后统一调用厂家协议解码反馈。
  virtual bool exchangeProcessData() = 0;
  // 执行一次低频管理帧事务。
  //
  // SocketCAN 路径按 joint.canfd.bus 发帧；gateway 路径按 joint.gateway 槽位发帧。
  // 该接口服务查询 ID、标零等配置命令，不参与高频运动控制周期。
  virtual bool exchangeBusFrame(
    const HardwareJointConfig & joint,
    const CanfdFrame & request,
    CanfdFrame & response,
    int timeout_us) = 0;
  virtual void close() = 0;
  virtual std::string lastError() const = 0;
};

// 直连 SocketCAN 的 CANFD transport，内部复用现有 CanfdMaster。
//
// 这个类是薄封装：保留 CanfdMaster 的 socket 打开、信箱匹配、收发等待等行为，
// 只把接口整理成 MixBackend 能和 gateway 路径共用的 CanfdTransport。
class SocketCanfdTransport : public CanfdTransport
{
public:
  SocketCanfdTransport();
  explicit SocketCanfdTransport(std::unique_ptr<CanfdMaster> master);
  ~SocketCanfdTransport() override = default;

  bool open(const HardwareConfig & config) override;
  bool configure(const HardwareConfig & config) override;
  bool setOutputFrame(std::size_t joint_index, const CanfdFrame & frame) override;
  bool getInputFrame(std::size_t joint_index, CanfdFrame & frame) const override;
  bool exchangeProcessData() override;
  bool exchangeBusFrame(
    const HardwareJointConfig & joint,
    const CanfdFrame & request,
    CanfdFrame & response,
    int timeout_us) override;
  void close() override;
  std::string lastError() const override;

private:
  std::unique_ptr<CanfdMaster> master_;
  std::string last_error_;
};

// EtherCAT 网关 CANFD transport，内部使用 IGH master 和中性网关槽位适配器。
//
// 它把 MixBackend 生成的 CanfdFrame 写入 IGH PDO 缓冲区中的 gateway 槽位；
// 厂家协议仍由 MixBackend 选择，transport 不解析 Encos/Tienkung 帧内容。
class GatewayCanfdTransport : public CanfdTransport
{
public:
  explicit GatewayCanfdTransport(std::unique_ptr<EthercatMaster> master);
  ~GatewayCanfdTransport() override = default;

  bool open(const HardwareConfig & config) override;
  bool configure(const HardwareConfig & config) override;
  bool setOutputFrame(std::size_t joint_index, const CanfdFrame & frame) override;
  bool getInputFrame(std::size_t joint_index, CanfdFrame & frame) const override;
  bool exchangeProcessData() override;
  bool exchangeBusFrame(
    const HardwareJointConfig & joint,
    const CanfdFrame & request,
    CanfdFrame & response,
    int timeout_us) override;
  void close() override;
  std::string lastError() const override;

private:
  const HardwareJointConfig * jointAt(std::size_t joint_index) const;
  std::unique_ptr<EthercatMaster> master_;
  CanfdGateway gateway_;
  std::vector<HardwareJointConfig> joints_;
  std::string last_error_;
};

}  // namespace robot_driver

#endif  // ROBOT_DRIVER__CANFD__CANFD_TRANSPORT_HPP_
