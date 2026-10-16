#ifndef ROBOT_DRIVER__ETHERCAT__SOEM_MASTER_HPP_
#define ROBOT_DRIVER__ETHERCAT__SOEM_MASTER_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "robot_driver/ethercat/ethercat_master.hpp"

namespace robot_driver
{

// 基于 SOEM 的 EtherCAT master 实现。
//
// 本类只负责 EtherCAT 总线生命周期和 PDO 缓冲区访问，不编码任何厂家电机协议，
// 也不会主动 enable 电机。
//
// 注意：SOEM 使用全局 C API 和全局 ec_slave/ec_group 状态，因此该类当前按单实例使用。
// 如果未来需要多网卡或多机器人，应先在架构上隔离 SOEM 上下文，而不是直接复制实例。
class SoemMaster : public EthercatMaster
{
public:
  SoemMaster();
  ~SoemMaster() override;

  SoemMaster(const SoemMaster &) = delete;
  SoemMaster & operator=(const SoemMaster &) = delete;

  // 打开与 SOEM 的连接。
  bool open(const std::string & network_interface) override;
  // 配置与 SOEM 的连接。
  bool configure(int expected_slave_count) override;
  // 执行一次 ec_send_processdata/ec_receive_processdata。
  bool exchangeProcessData() override;
  // 检查并尝试恢复非 OP 从站。
  bool monitorSlaves() override;
  // 获取逻辑从站的 output PDO buffer。
  std::uint8_t * outputBuffer(int logical_slave_id) override;
  // 返回 output PDO buffer 大小，用于协议层在 reinterpret_cast 前做长度保护。
  std::size_t outputBufferSize(int logical_slave_id) const override;
  // 获取逻辑从站的 input PDO buffer。
  const std::uint8_t * inputBuffer(int logical_slave_id) const override;
  // 返回 input PDO buffer 大小，用于协议层在 reinterpret_cast 前做长度保护。
  std::size_t inputBufferSize(int logical_slave_id) const override;
  // 关闭与 SOEM 的连接。
  void close() override;
  // 返回当前运行状态。
  EthercatRuntimeState currentState() const override;

private:
  // 执行一次 SOEM 从站配置和 OP 状态迁移。configure() 负责重试和 socket 重建。
  bool configureOnce(int expected_slave_count);

  // 从 SOEM 全局状态刷新 state_。
  void refreshSlaveStates();

  // 启动/停止后台监控线程。线程只做总线状态检查和恢复，不编码任何厂家命令。
  void startMonitorThread();
  void stopMonitorThread();
  void monitorLoop();
  void setMonitorEnabled(bool enabled) override;

  // monitorSlaves() 的无锁内部实现。调用方必须已经持有 soem_mutex_。
  bool monitorSlavesLocked();
  // 更新错误文本并标记非 operational。
  void setError(const std::string & message);
  // 根据连续健康周期更新对外 operational 状态，避免一次瞬时抖动触发上层 fault。
  void updateStableOperational(bool cycle_healthy);

  // SOEM ec_config_map 需要一块 process data IO 内存。具体输出内容由协议层写入。
  std::array<char, 4096> io_map_{};
  // 当前运行状态。
  EthercatRuntimeState state_;
  // 是否已打开与 SOEM 的连接。
  bool opened_{false};
  // 最近一次打开的网卡名。configure 重试时需要完整重建 SOEM socket。
  std::string network_interface_;
  // 是否已经成功进入过 OP 生命周期。参考旧驱动的 inOP 语义。
  bool in_operational_lifecycle_{false};

  // WKC 容错计数。EtherCAT 周期允许短暂丢包，但连续异常应上报故障。
  int work_counter_error_count_{0};
  int work_counter_iteration_count_{0};
  int stable_operational_count_{0};

  // SOEM 使用全局状态，process data 周期和状态恢复不能并发调用 SOEM API。
  mutable std::mutex soem_mutex_;

  // 低频从站监控线程，对应旧驱动 ethercat_check_thread 的职责。
  std::atomic<bool> monitor_running_{false};
  // 是否允许启动后台监控线程。config_mode 下会关闭它，避免与 admin srv 竞争。
  std::atomic<bool> monitor_enabled_{true};
  std::thread monitor_thread_;
};

}  // namespace robot_driver

#endif  // ROBOT_DRIVER__ETHERCAT__SOEM_MASTER_HPP_
