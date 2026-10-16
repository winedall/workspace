#include "robot_driver/ethercat/soem_master.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
extern "C" {
#include "ethercat.h"
}
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

namespace robot_driver
{

namespace
{

constexpr int kConfigureAttempts = 3;
constexpr int kWorkCounterErrorPeriod = 1000;
constexpr int kWorkCounterErrorMax = 10;
constexpr int kStableOperationalCycles = 3;

static const auto kLogger = rclcpp::get_logger("soem_master");

}  // namespace

SoemMaster::SoemMaster() = default;

SoemMaster::~SoemMaster()
{
  close();
}

// 打开与 SOEM 的连接。
bool SoemMaster::open(const std::string & network_interface)
{
  // 重新 open 前先 close，避免重复调用时遗留旧 socket 或旧运行状态。
  close();
  std::lock_guard<std::mutex> lock(soem_mutex_);
  state_ = EthercatRuntimeState{};

  if (network_interface.empty()) {
    setError("EtherCAT network_interface is empty");
    return false;
  }
  network_interface_ = network_interface;

  // ec_init 会打开原始 socket 并绑定网卡；只有真实链路模式才会调用到这里。
  if (ec_init(network_interface.c_str()) == 0) {
    setError("EtherCAT open failed on interface '" + network_interface + "'");
    return false;
  }

  opened_ = true;
  state_.opened = true;
  state_.last_error.clear();
  return true;
}

// 配置与 SOEM 的连接。
bool SoemMaster::configure(int expected_slave_count)
{
  std::lock_guard<std::mutex> lock(soem_mutex_);
  state_.expected_slave_count = expected_slave_count;
  if (!opened_) {
    setError("EtherCAT configure requested before open");
    return false;
  }

  for (int attempt = 0; attempt < kConfigureAttempts; ++attempt) {
    if (configureOnce(expected_slave_count)) {
      state_.last_error.clear();
      if (monitor_enabled_.load()) {
        startMonitorThread();
      }
      return true;
    }

    if (attempt + 1 >= kConfigureAttempts) {
      break;
    }

    ec_close();
    opened_ = false;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (ec_init(network_interface_.c_str()) == 0) {
      setError(
        "EtherCAT reopen failed on interface '" + network_interface_ +
        "' during configure retry");
      continue;
    }
    opened_ = true;
    state_.opened = true;
  }

  return false;
}

void SoemMaster::setMonitorEnabled(bool enabled)
{
  monitor_enabled_.store(enabled);
  if (!enabled) {
    stopMonitorThread();
  }
}

// 一次配置尝试。
bool SoemMaster::configureOnce(int expected_slave_count)
{
  state_.configured = false;
  state_.operational = false;
  state_.expected_slave_count = expected_slave_count;
  state_.expected_work_counter = 0;
  state_.actual_work_counter = 0;
  state_.slaves.clear();
  in_operational_lifecycle_ = false;
  work_counter_error_count_ = 0;
  work_counter_iteration_count_ = 0;
  stable_operational_count_ = 0;

  const auto detected = ec_config_init(FALSE);
  state_.detected_slave_count = detected;
  if (detected <= 0) {
    setError("EtherCAT no slaves found");
    return false;
  }

  // 清除 ECT_COEDET_SDOCA 标志，避免 SOEM 使用 SDO Complete Access 读取 PDO 映射。
  // 部分 encos 从站不支持 Complete Access，开启该标志会得到无效映射（AL status 0x1E），
  // 导致从站无法进入 SAFE_OP/OP。
  for (int slave_idx = 0; slave_idx < ec_slavecount; ++slave_idx) {
    ec_slave[slave_idx + 1].CoEdetails &= ~ECT_COEDET_SDOCA;
  }

  if (expected_slave_count > 0 && detected != expected_slave_count) {
    // 从站数量不匹配通常意味着网线、供电、配置文件或 slave_id 规划有问题。
    // 这里直接失败，不允许在错误映射下继续进入运动控制。
    refreshSlaveStates();
    setError(
      "EtherCAT slave count mismatch: expected " + std::to_string(expected_slave_count) +
      ", detected " + std::to_string(detected));
    return false;
  }

  std::fill(io_map_.begin(), io_map_.end(), 0);
  // SOEM 会把 ec_slave[i].inputs/outputs 指向 io_map_ 内部区域；
  // 后续 outputBuffer() 返回的也是这块内存中的某段。
  ec_config_map(io_map_.data());
  ec_configdc();
  ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * std::max(detected, 1));

  state_.expected_work_counter = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

  // 为了读取输入 PDO，SOEM 需要让从站进入 OP。这里不填充任何运动输出，只发送零初始化 IOmap。
  // 运动命令只有 EncosBackend 通过 Protocol 显式写入 output buffer 后才会被带到总线上。
  ec_slave[0].state = EC_STATE_OPERATIONAL;
  ec_send_processdata();
  state_.actual_work_counter = ec_receive_processdata(EC_TIMEOUTRET);
  ec_writestate(0);

  int retries = 40;
  do {
    // 反复交换 process data，给从站进入 OPERATIONAL 留出时间。
    ec_send_processdata();
    state_.actual_work_counter = ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
  } while (retries-- > 0 && ec_slave[0].state != EC_STATE_OPERATIONAL);

  refreshSlaveStates();
  state_.configured = true;
  state_.operational = ec_slave[0].state == EC_STATE_OPERATIONAL;
  if (!state_.operational) {
    std::string details = "EtherCAT not all slaves reached OPERATIONAL (master_state=" + std::to_string(ec_slave[0].state) + ")";
    for (int i = 1; i <= ec_slavecount; ++i) {
      details += " slave" + std::to_string(i) + "={state=" + std::to_string(ec_slave[i].state) + " al_status=" + std::to_string(ec_slave[i].ALstatuscode) + "}";
    }
    setError(details);
    return false;
  }

  in_operational_lifecycle_ = true;
  stable_operational_count_ = kStableOperationalCycles;
  return true;
}

// 执行一次 ec_send_processdata/ec_receive_processdata。
bool SoemMaster::exchangeProcessData()
{
  std::lock_guard<std::mutex> lock(soem_mutex_);
  if (!state_.configured) {
    setError("EtherCAT process data exchange requested before configure");
    return false;
  }

  ec_send_processdata();
  state_.actual_work_counter = ec_receive_processdata(EC_TIMEOUTRET);
  if (state_.expected_work_counter > 0 && state_.actual_work_counter < state_.expected_work_counter)
  {
    ++work_counter_error_count_;
    ec_group[0].docheckstate = TRUE;
  }

  ++work_counter_iteration_count_;
  if (work_counter_iteration_count_ > kWorkCounterErrorPeriod) {
    work_counter_iteration_count_ = 0;
    work_counter_error_count_ = 0;
  }

  refreshSlaveStates();

  // WKC 是 EtherCAT 周期是否可信的核心状态。实际 WKC 低于期望值时，
  // 说明至少部分从站没有正确响应。参考旧驱动，只在连续异常超过阈值时上报故障。
  const bool wkc_healthy = state_.expected_work_counter <= 0 || state_.actual_work_counter >= state_.expected_work_counter;
  if (!wkc_healthy && work_counter_error_count_ > kWorkCounterErrorMax) {
    setError(
      "EtherCAT workcounter abnormal: expected " +
      std::to_string(state_.expected_work_counter) + ", actual " +
      std::to_string(state_.actual_work_counter));
    return false;
  }

  const bool state_operational = ec_slave[0].state == EC_STATE_OPERATIONAL;
  in_operational_lifecycle_ = in_operational_lifecycle_ || state_operational;
  updateStableOperational(state_operational && wkc_healthy);
  if (!state_operational && work_counter_error_count_ > kWorkCounterErrorMax) {
    setError("EtherCAT master is not OPERATIONAL");
    return false;
  }

  if (state_operational && wkc_healthy) {
    state_.last_error.clear();
  } else if (!wkc_healthy) {
    state_.last_error =
      "EtherCAT transient workcounter abnormal: expected " +
      std::to_string(state_.expected_work_counter) + ", actual " +
      std::to_string(state_.actual_work_counter);
  }
  return true;
}

// 检查并尝试恢复 EtherCAT 从站。
bool SoemMaster::monitorSlaves()
{
  std::lock_guard<std::mutex> lock(soem_mutex_);
  return monitorSlavesLocked();
}

// 检查并尝试恢复 EtherCAT 从站。
bool SoemMaster::monitorSlavesLocked()
{
  if (!state_.configured) {
    setError("EtherCAT slave monitor requested before configure");
    return false;
  }

  // 该函数保留旧驱动 ethercat_check_thread 的核心恢复路径。
  // 没有 WKC 异常或 docheckstate 标记时，不主动扰动 SOEM 状态机。
  if (in_operational_lifecycle_ && state_.actual_work_counter >= state_.expected_work_counter && !ec_group[0].docheckstate)
  {
    updateStableOperational(true);
    if (state_.operational) {
      state_.last_error.clear();
    }
    return true;
  }

  ec_readstate();
  bool all_operational = in_operational_lifecycle_;
  ec_group[0].docheckstate = FALSE;

  for (int slave = 1; slave <= ec_slavecount; ++slave) {
    if (ec_slave[slave].group != 0) {
      continue;
    }
    if (ec_slave[slave].state == EC_STATE_OPERATIONAL && !ec_slave[slave].islost) {
      continue;
    }

    all_operational = false;
    ec_group[0].docheckstate = TRUE;

    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
      // SAFE_OP + ERROR 需要先 ACK 错误，再尝试回到 OP。
      ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
      ec_writestate(slave);
    } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
      ec_slave[slave].state = EC_STATE_OPERATIONAL;
      ec_writestate(slave);
    } else if (ec_slave[slave].state > EC_STATE_NONE) {
      if (ec_reconfig_slave(slave, EC_TIMEOUTRET)) {
        ec_slave[slave].islost = FALSE;
      }
    }

    ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
    if (ec_slave[slave].state == EC_STATE_NONE || ec_slave[slave].islost) {
      if (ec_recover_slave(slave, EC_TIMEOUTRET)) {
        ec_slave[slave].islost = FALSE;
      }
    }
  }

  refreshSlaveStates();
  all_operational = ec_slave[0].state == EC_STATE_OPERATIONAL;
  for (const auto & slave : state_.slaves) {
    all_operational = all_operational && slave.online;
  }

  updateStableOperational(all_operational);
  if (!all_operational) {
    state_.last_error = "EtherCAT slave monitor recovery pending";
    return false;
  }

  state_.last_error = "EtherCAT slave monitor recovered";
  work_counter_error_count_ = 0;
  work_counter_iteration_count_ = 0;
  return true;
}

// 获取逻辑从站的 output PDO buffer。
std::uint8_t * SoemMaster::outputBuffer(int logical_slave_id)
{
  std::lock_guard<std::mutex> lock(soem_mutex_);
  if (!state_.configured) {
    setError("EtherCAT output buffer requested before configure");
    return nullptr;
  }
  if (logical_slave_id < 0 || logical_slave_id >= ec_slavecount) {
    // hw_encos.yaml 中的 slave_id 使用 0 基逻辑编号；SOEM 的 ec_slave 是 1 基数组。
    // 因此合法范围是 [0, ec_slavecount)，实际访问时再 +1。
    setError(
      "EtherCAT output slave_id out of range: " + std::to_string(logical_slave_id) +
      ", detected slaves=" + std::to_string(ec_slavecount));
    return nullptr;
  }
  auto * outputs = ec_slave[logical_slave_id + 1].outputs;
  if (outputs == nullptr) {
    setError("EtherCAT slave output buffer is null for slave_id " + std::to_string(logical_slave_id));
    return nullptr;
  }

  return outputs;
}

// 返回 output PDO buffer 大小，用于协议层在 reinterpret_cast 前做长度保护。
std::size_t SoemMaster::outputBufferSize(int logical_slave_id) const
{
  std::lock_guard<std::mutex> lock(soem_mutex_);
  // const 函数中不更新 state_，只返回可用于长度检查的 Obytes。
  // 调用方拿到 0 时应拒绝协议编码。
  if (!state_.configured || logical_slave_id < 0 || logical_slave_id >= ec_slavecount) {
    return 0;
  }
  return static_cast<std::size_t>(ec_slave[logical_slave_id + 1].Obytes);
}

// 获取逻辑从站的 input PDO buffer。
const std::uint8_t * SoemMaster::inputBuffer(int logical_slave_id) const
{
  std::lock_guard<std::mutex> lock(soem_mutex_);
  if (!state_.configured) {
    return nullptr;
  }
  if (logical_slave_id < 0 || logical_slave_id >= ec_slavecount) {
    return nullptr;
  }
  return ec_slave[logical_slave_id + 1].inputs;
}

// 返回 input PDO buffer 大小，用于协议层在 reinterpret_cast 前做长度保护。
std::size_t SoemMaster::inputBufferSize(int logical_slave_id) const
{
  std::lock_guard<std::mutex> lock(soem_mutex_);
  if (!state_.configured || logical_slave_id < 0 || logical_slave_id >= ec_slavecount) {
    return 0;
  }
  return static_cast<std::size_t>(ec_slave[logical_slave_id + 1].Ibytes);
}

// 关闭与 SOEM 的连接。
void SoemMaster::close()
{
  stopMonitorThread();
  std::lock_guard<std::mutex> lock(soem_mutex_);
  if (opened_) {
    // SOEM 的 ec_close 释放原始 socket。即使 configure 失败，也允许安全 close。
    ec_close();
  }
  opened_ = false;
  state_.opened = false;
  state_.configured = false;
  state_.operational = false;
  in_operational_lifecycle_ = false;
  work_counter_error_count_ = 0;
  work_counter_iteration_count_ = 0;
  stable_operational_count_ = 0;
}

// 返回当前运行状态。
EthercatRuntimeState SoemMaster::currentState() const
{
  std::lock_guard<std::mutex> lock(soem_mutex_);
  return state_;
}

// 启动 SOEM 监控线程。
void SoemMaster::startMonitorThread()
{
  if (monitor_running_.exchange(true)) {
    return;
  }
  monitor_thread_ = std::thread(&SoemMaster::monitorLoop, this);
}

// 停止 SOEM 监控线程。
void SoemMaster::stopMonitorThread()
{
  if (!monitor_running_.exchange(false)) {
    return;
  }
  if (monitor_thread_.joinable()) {
    monitor_thread_.join();
  }
}

// SOEM 监控线程循环。
void SoemMaster::monitorLoop()
{
  while (monitor_running_.load()) {
    {
      std::lock_guard<std::mutex> lock(soem_mutex_);
      if (state_.configured) {
        if (ec_group[0].docheckstate || work_counter_error_count_ > 0 || !state_.operational) {
          monitorSlavesLocked();
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

// 从 SOEM 全局状态刷新 state_。
void SoemMaster::refreshSlaveStates()
{
  // 读取 SOEM 全局 slave 状态，并转换为不含厂家协议信息的通用运行状态快照。
  state_.detected_slave_count = ec_slavecount;
  state_.slaves.clear();
  if (ec_slavecount <= 0) {
    return;
  }

  ec_readstate();
  state_.slaves.reserve(static_cast<std::size_t>(ec_slavecount));
  for (int slave = 1; slave <= ec_slavecount; ++slave) {
    EthercatSlaveState item;
    item.slave_index = slave;
    item.name = ec_slave[slave].name;
    item.state = ec_slave[slave].state;
    item.al_status_code = ec_slave[slave].ALstatuscode;
    item.lost = ec_slave[slave].islost;
    item.online = item.state == EC_STATE_OPERATIONAL && !item.lost;
    state_.slaves.push_back(item);
  }
}

// 更新错误文本并标记非 operational。
void SoemMaster::setError(const std::string & message)
{
  RCLCPP_ERROR(kLogger, "%s", message.c_str());
  state_.last_error = message;
  state_.operational = false;
  stable_operational_count_ = 0;
}

// 更新稳定 operational 状态。
void SoemMaster::updateStableOperational(bool cycle_healthy)
{
  if (!cycle_healthy) {
    stable_operational_count_ = 0;
    if (work_counter_error_count_ > kWorkCounterErrorMax) {
      state_.operational = false;
    }
    return;
  }

  if (stable_operational_count_ < kStableOperationalCycles) {
    ++stable_operational_count_;
  }
  if (stable_operational_count_ >= kStableOperationalCycles) {
    state_.operational = true;
  }
}

}  // namespace robot_driver
