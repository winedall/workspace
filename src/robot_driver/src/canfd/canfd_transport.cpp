#include "robot_driver/canfd/canfd_transport.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

namespace robot_driver
{

// 默认构造真实 SocketCAN transport，内部创建 CanfdMaster。
SocketCanfdTransport::SocketCanfdTransport()
: SocketCanfdTransport(std::make_unique<CanfdMaster>())
{
}

// 注入式构造，供测试或特殊运行环境替换 CanfdMaster。
SocketCanfdTransport::SocketCanfdTransport(std::unique_ptr<CanfdMaster> master)
: master_(std::move(master))
{
}

// 打开 SocketCAN CANFD 接口。
//
// 该函数只处理硬件通讯通道，不解析 Encos/Tienkung 厂家协议。
bool SocketCanfdTransport::open(const HardwareConfig & config)
{
  // SocketCAN 路径只读取 hardware.canfd；混装协议字段仍由 MixBackend 处理。
  if (!master_) {
    last_error_ = "Socket CANFD master is not available";
    return false;
  }
  if (!master_->open(config.canfd)) {
    last_error_ = "Socket CANFD open failed";
    return false;
  }
  last_error_.clear();
  return true;
}

// 按硬件关节顺序创建 CanfdMaster 的输入/输出信箱。
bool SocketCanfdTransport::configure(const HardwareConfig & config)
{
  if (!master_) {
    last_error_ = "Socket CANFD master is not available";
    return false;
  }
  if (!master_->configure(config.joints)) {
    last_error_ = "Socket CANFD configure failed";
    return false;
  }
  last_error_.clear();
  return true;
}

// 把某个关节的待发送 CANFD 帧写入 SocketCAN 输出信箱。
bool SocketCanfdTransport::setOutputFrame(std::size_t joint_index, const CanfdFrame & frame)
{
  if (!master_ || !master_->setOutputFrame(joint_index, frame)) {
    last_error_ = "Socket CANFD output buffer unavailable";
    return false;
  }
  return true;
}

// 从 SocketCAN 输入信箱读取某个关节最近一次反馈帧。
bool SocketCanfdTransport::getInputFrame(std::size_t joint_index, CanfdFrame & frame) const
{
  return master_ && master_->getInputFrame(joint_index, frame);
}

// 执行一次 SocketCAN 周期收发。
bool SocketCanfdTransport::exchangeProcessData()
{
  if (!master_ || !master_->exchangeProcessData()) {
    last_error_ = "Socket CANFD process data exchange failed";
    return false;
  }
  return true;
}

// 在直连 CANFD 总线上执行一次低频管理帧事务。
bool SocketCanfdTransport::exchangeBusFrame(
  const HardwareJointConfig & joint,
  const CanfdFrame & request,
  CanfdFrame & response,
  int timeout_us)
{
  // 直连 CANFD 的管理事务使用配置里的逻辑 CAN bus，回包由 CanfdMaster 从该 bus 接收。
  if (!master_ || !master_->exchangeBusFrame(joint.canfd.bus, request, response, timeout_us)) {
    last_error_ = "Socket CANFD bus exchange failed";
    return false;
  }
  return true;
}

// 关闭 SocketCAN transport 并释放底层 socket。
void SocketCanfdTransport::close()
{
  if (master_) {
    master_->close();
  }
}

// 返回最近一次 SocketCAN transport 错误摘要。
std::string SocketCanfdTransport::lastError() const
{
  return last_error_;
}

// 构造 EtherCAT gateway transport，master 通常是 IghMaster。
GatewayCanfdTransport::GatewayCanfdTransport(std::unique_ptr<EthercatMaster> master)
: master_(std::move(master))
{
}

// 打开 EtherCAT master；CANFD 通道由后续 PDO 槽位承载。
bool GatewayCanfdTransport::open(const HardwareConfig & config)
{
  // Gateway 路径只打开 EtherCAT master；CANFD 总线被封装在从站 PDO 槽位中。
  if (!master_) {
    last_error_ = "Gateway EtherCAT master is not available";
    return false;
  }
  if (!master_->open(config.ethercat.network_interface)) {
    last_error_ = "Gateway EtherCAT open failed";
    return false;
  }
  last_error_.clear();
  return true;
}

// 配置 EtherCAT 从站并缓存关节到 gateway 槽位的映射。
bool GatewayCanfdTransport::configure(const HardwareConfig & config)
{
  if (!master_) {
    last_error_ = "Gateway EtherCAT master is not available";
    return false;
  }
  // 保存一份关节映射，后续 joint_index 可以稳定换算到 gateway.slave_id/bus/slot。
  joints_ = config.joints;
  if (!master_->configure(config.ethercat.slave_count)) {
    last_error_ = "Gateway EtherCAT configure failed";
    return false;
  }
  last_error_.clear();
  return true;
}

// 按 hardware.joints 下标查询关节配置，供 transport 信箱接口使用。
const HardwareJointConfig * GatewayCanfdTransport::jointAt(std::size_t joint_index) const
{
  if (joint_index >= joints_.size()) {
    return nullptr;
  }
  return &joints_[joint_index];
}

// 把某个关节的 CANFD 帧写入对应 EtherCAT 从站的输出 PDO 槽位。
bool GatewayCanfdTransport::setOutputFrame(std::size_t joint_index, const CanfdFrame & frame)
{
  const auto * joint = jointAt(joint_index);
  if (joint == nullptr || !master_) {
    last_error_ = "Gateway output joint index is invalid";
    return false;
  }
  // 每个 gateway 槽位落在某个 EtherCAT 从站的输出 PDO 中。
  auto * output = master_->outputBuffer(joint->gateway.slave_id);
  const auto size = master_->outputBufferSize(joint->gateway.slave_id);
  std::string error;
  if (!gateway_.writeFrame(*joint, frame, output, size, error)) {
    last_error_ = error;
    return false;
  }
  return true;
}

// 从对应 EtherCAT 从站输入 PDO 槽位读取某个关节的 CANFD 反馈帧。
bool GatewayCanfdTransport::getInputFrame(std::size_t joint_index, CanfdFrame & frame) const
{
  const auto * joint = jointAt(joint_index);
  if (joint == nullptr || !master_) {
    return false;
  }
  // 输入 PDO 与输出 PDO 使用相同的 gateway 槽位描述，读取后恢复成通用 CanfdFrame。
  const auto * input = master_->inputBuffer(joint->gateway.slave_id);
  const auto size = master_->inputBufferSize(joint->gateway.slave_id);
  std::string error;
  return gateway_.readFrame(*joint, input, size, frame, error);
}

// 执行一次 EtherCAT process data 交换，让 PDO 输出/输入同步。
bool GatewayCanfdTransport::exchangeProcessData()
{
  if (!master_ || !master_->exchangeProcessData()) {
    last_error_ = "Gateway EtherCAT process data exchange failed";
    return false;
  }
  return true;
}

// 通过指定关节的 gateway 槽位执行一次低频请求/响应事务。
bool GatewayCanfdTransport::exchangeBusFrame(
  const HardwareJointConfig & joint,
  const CanfdFrame & request,
  CanfdFrame & response,
  int timeout_us)
{
  (void)timeout_us;
  // EtherCAT gateway 没有 SocketCAN 那种“向某 bus 广播并接收任意回包”的 socket。
  // 这里按调用方指定的关节槽位执行一次请求/响应，适合按关节定位的低频维护命令。
  if (!master_) {
    last_error_ = "Gateway EtherCAT master is not available";
    return false;
  }
  auto * output = master_->outputBuffer(joint.gateway.slave_id);
  const auto output_size = master_->outputBufferSize(joint.gateway.slave_id);
  std::string error;
  if (!gateway_.writeFrame(joint, request, output, output_size, error)) {
    last_error_ = error;
    return false;
  }
  if (!master_->exchangeProcessData()) {
    last_error_ = "Gateway EtherCAT bus frame exchange failed";
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  const auto * input = master_->inputBuffer(joint.gateway.slave_id);
  const auto input_size = master_->inputBufferSize(joint.gateway.slave_id);
  if (!gateway_.readFrame(joint, input, input_size, response, error)) {
    last_error_ = error;
    return false;
  }
  return true;
}

// 关闭 EtherCAT gateway transport 并释放 master。
void GatewayCanfdTransport::close()
{
  if (master_) {
    master_->close();
  }
}

// 返回最近一次 gateway transport 错误；若本层无错误则透传 EtherCAT master 错误。
std::string GatewayCanfdTransport::lastError() const
{
  if (!last_error_.empty()) {
    return last_error_;
  }
  if (master_) {
    return master_->currentState().last_error;
  }
  return "";
}

}  // namespace robot_driver
