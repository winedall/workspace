#ifndef ROBOT_DRIVER__ETHERCAT__IGH_MASTER_HPP_
#define ROBOT_DRIVER__ETHERCAT__IGH_MASTER_HPP_

#include <memory>
#include <string>

#include "robot_driver/ethercat/ethercat_master.hpp"
#include "robot_driver/hardware_config.hpp"

namespace robot_driver
{

// IGH EtherCAT 主站封装。
//
// 这个类只负责主站生命周期、PDO 缓冲区和从站状态查询，不关心具体厂商
// 的电机协议。天工后端通过它把天工网关的 PDO 周期接入到统一驱动框架里。
class IghMaster : public EthercatMaster
{
public:
  // 依据 EtherCAT 总线配置创建主站封装。
  explicit IghMaster(const EthercatBusConfig & config);
  ~IghMaster() override;

  IghMaster(const IghMaster &) = delete;
  IghMaster & operator=(const IghMaster &) = delete;

  // 打开 IGH 主站并请求底层 master 资源。
  bool open(const std::string & network_interface) override;
  // 按预期从站数量完成 PDO 注册和从站配置。
  bool configure(int expected_slave_count) override;
  // 是否启用后台状态巡检线程。
  void setMonitorEnabled(bool enabled) override;
  // 执行一次周期性的收发交换。
  bool exchangeProcessData() override;
  // 轮询从站状态，用于故障检测和恢复判断。
  bool monitorSlaves() override;
  // 获取指定逻辑从站的输出缓冲区。
  //
  // 天工这条链路按 1 基从站号对外暴露，slave 1 对应第一路网关缓冲区。
  std::uint8_t * outputBuffer(int logical_slave_id) override;
  // 获取指定逻辑从站的输出缓冲区大小。
  std::size_t outputBufferSize(int logical_slave_id) const override;
  // 获取指定逻辑从站的输入缓冲区。
  const std::uint8_t * inputBuffer(int logical_slave_id) const override;
  // 获取指定逻辑从站的输入缓冲区大小。
  std::size_t inputBufferSize(int logical_slave_id) const override;
  // 关闭主站并释放底层资源。
  void close() override;
  // 返回当前 EtherCAT 运行状态快照。
  EthercatRuntimeState currentState() const override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_driver

#endif  // ROBOT_DRIVER__ETHERCAT__IGH_MASTER_HPP_
