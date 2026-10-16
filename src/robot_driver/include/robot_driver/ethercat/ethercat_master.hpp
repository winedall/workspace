#ifndef ROBOT_DRIVER__ETHERCAT__ETHERCAT_MASTER_HPP_
#define ROBOT_DRIVER__ETHERCAT__ETHERCAT_MASTER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
#include <set>

namespace robot_driver
{

// 单个 EtherCAT 从站的运行状态。
//
// 这里刻意只描述 EtherCAT 层状态，不绑定 Encos/Tienkung 的电机 PDO 格式。
// 厂商电机反馈解析应放在具体 backend 中，避免通用 master 接口被某一家协议污染。
struct EthercatSlaveState
{
  // SOEM 从站下标通常从 1 开始；0 表示 master 汇总状态，不作为实际从站。
  int slave_index{0};
  // 从站名称，来自 SOEM 的 ec_slave[i].name。
  std::string name;
  // 当前 AL 状态原始值，便于日志定位 SAFE_OP/OP/ERROR 等状态。
  int state{0};
  // AL status code 原始值，后续可进一步转换为可读文本。
  int al_status_code{0};
  // 是否处于 OPERATIONAL 且未 lost。
  bool online{false};
  // SOEM 标记的 lost 状态。
  bool lost{false};
};

// EtherCAT master 的运行状态快照。
//
// EncosBackend 用该结构生成 DriverStatus.status_message/fault_active。
// 该结构是内部状态，不对应 ROS 诊断 topic。
struct EthercatRuntimeState
{
  // 网卡是否成功打开。
  bool opened{false};
  // 从站是否完成配置和 IO map。
  bool configured{false};
  // 全部从站是否进入 OPERATIONAL。
  bool operational{false};
  // backend 根据配置推导的期望从站数量。
  int expected_slave_count{0};
  // SOEM 实际扫描到的从站数量。
  int detected_slave_count{0};
  // SOEM 根据 input/output WKC 推导的期望 workcounter。
  int expected_work_counter{0};
  // 最近一次 ec_receive_processdata 返回的实际 workcounter。
  int actual_work_counter{0};
  // 最近一次错误摘要。为空表示当前没有明确错误。
  std::string last_error;
  // 每个从站的总线级状态快照。
  std::vector<EthercatSlaveState> slaves;
};

// EtherCAT master 抽象接口。
//
// 该接口让 EncosBackend 可以通过 fake master 做单元测试；真实环境下由 SoemMaster
// 封装 SOEM 全局 API。这里不包含 Encos/Tienkung/Unitree 等厂商协议字段，只暴露
// 总线生命周期、process data 周期和从站 PDO 输出缓冲区。
class EthercatMaster
{
public:
  virtual ~EthercatMaster() = default;

  // 打开指定网卡。失败时返回 false，并在 currentState().last_error 中给出原因。
  virtual bool open(const std::string & network_interface) = 0;
  // 扫描并配置从站。expected_slave_count 为配置期望数量，0 表示只诊断不强制数量。
  virtual bool configure(int expected_slave_count) = 0;

  // 是否允许 master 启动后台监控线程。
  //
  // 默认实现为空。像 SoemMaster 这类带后台 monitor 的实现可以覆盖它；
  // 对于不需要后台监控线程的 master（例如 IghMaster），保持 no-op 即可。
  virtual void setMonitorEnabled(bool enabled)
  {
    (void)enabled;
  }

  // 执行一次 process data 交换：发送当前 output PDO，并接收最新 input PDO。
  virtual bool exchangeProcessData() = 0;

  // 检查并尽量恢复 EtherCAT 从站状态。
  //
  // 该接口对应旧驱动 ethercat_check_thread 的核心职责，但不绑定线程模型。
  // 调用方可以在周期 update 中低频或按故障触发调用。返回 true 表示当前总线已经恢复到可继续
  // process data 的状态；返回 false 表示仍存在未恢复的从站或总线错误。
  virtual bool monitorSlaves() = 0;

  // 返回逻辑从站的输出 PDO 缓冲区。logical_slave_id 使用硬件配置中的 0 基编号；
  // 实现层负责换算到 SOEM 的 ec_slave[1..N] 下标。
  virtual std::uint8_t * outputBuffer(int logical_slave_id) = 0;
  // 返回逻辑从站输出 PDO 缓冲区字节数。返回 0 表示不可写或没有输出 PDO。
  virtual std::size_t outputBufferSize(int logical_slave_id) const = 0;
  // 返回逻辑从站的输入 PDO 缓冲区。该缓冲区由最近一次 exchangeProcessData() 刷新。
  virtual const std::uint8_t * inputBuffer(int logical_slave_id) const = 0;
  // 返回逻辑从站输入 PDO 缓冲区字节数。返回 0 表示不可读或没有输入 PDO。
  virtual std::size_t inputBufferSize(int logical_slave_id) const = 0;
  // 关闭 master，释放 SOEM socket。
  virtual void close() = 0;

  // 返回最近一次总线运行状态快照。
  virtual EthercatRuntimeState currentState() const = 0;
};

}  // namespace robot_driver

#endif  // ROBOT_DRIVER__ETHERCAT__ETHERCAT_MASTER_HPP_
