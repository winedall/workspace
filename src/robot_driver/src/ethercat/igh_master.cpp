#include "robot_driver/ethercat/igh_master.hpp"

#include <cmath>
#include <algorithm>
#include <chrono>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#ifdef ROBOT_DRIVER_HAS_IGH
#include <ctime>
#include <ecrt.h>
#endif

namespace robot_driver
{

namespace
{

constexpr std::uint32_t kGatewayVendorId = 0x00123456;
constexpr std::uint32_t kGatewayProductCode = 0x26483052;
constexpr std::size_t kGatewayPdoSize = 400;
constexpr int kPdoEntryCount = 50;
constexpr int kWorkCounterErrorPeriod = 100;
constexpr int kWorkCounterErrorMax = 20;

const std::uint8_t kPdoSubindices[kPdoEntryCount] = {
  0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
  0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19,
  0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29,
  0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
  0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
  0x50
};

static const auto kLogger = rclcpp::get_logger("igh_master");

}  // namespace

struct IghMaster::Impl
{
  // 构造内部实现时，把 YAML 里的总线参数拷贝进来，并补齐天工网关默认值。
  explicit Impl(const EthercatBusConfig & bus_config)
  : config(bus_config)
  {
    if (config.cycle_time_ms <= 0.0) {
      config.cycle_time_ms = 2.0;
    }
  }

  EthercatBusConfig config;
  mutable std::mutex mutex;
  EthercatRuntimeState state;
  bool opened{false};
  bool configured{false};
  int expected_work_counter{0};
  int work_counter_error_count{0};
  int work_counter_iteration_count{0};
  std::vector<std::vector<std::uint8_t>> output_buffers;
  std::atomic<bool> monitor_enabled{true};
  std::atomic<bool> monitor_running{false};
  std::thread monitor_thread;

#ifdef ROBOT_DRIVER_HAS_IGH
  struct SlaveInfo
  {
    ec_slave_config_t * config{nullptr};
    unsigned int output_offset{0};
    unsigned int input_offset{0};
  };

  ec_master_t * master{nullptr};
  ec_domain_t * domain{nullptr};
  std::uint8_t * domain_data{nullptr};
  std::vector<SlaveInfo> slaves;
#endif

  // 记录最近一次主站错误，并将运行状态标成非 operational。
  void setError(const std::string & message)
  {
    RCLCPP_ERROR(kLogger, "%s", message.c_str());
    state.last_error = message;
    state.operational = false;
  }

#ifdef ROBOT_DRIVER_HAS_IGH
  // 按天工网关的 PDO 布局逐项注册输入/输出映射。
  //
  // 这里把注册逻辑集中起来，是为了让 configure() 只负责主流程，而不把低层
  // 映射细节散落到流程里。
  bool registerPdoEntries(SlaveInfo & slave)
  {
    // 天工网关每个从站暴露 50 个 PDO 子索引，这里按固定表逐项注册。
    for (int i = 0; i < kPdoEntryCount; ++i) {
      int offset = ecrt_slave_config_reg_pdo_entry(
        slave.config, 0x7000, kPdoSubindices[i], domain, nullptr);
      if (offset < 0) {
        setError(
          "IGH failed to register RxPDO entry 0x7000:0x" +
          std::to_string(kPdoSubindices[i]));
        return false;
      }
      if (i == 0) {
        slave.output_offset = static_cast<unsigned int>(offset);
      }

      offset = ecrt_slave_config_reg_pdo_entry(
        slave.config, 0x6000, kPdoSubindices[i], domain, nullptr);
      if (offset < 0) {
        setError(
          "IGH failed to register TxPDO entry 0x6000:0x" +
          std::to_string(kPdoSubindices[i]));
        return false;
      }
      if (i == 0) {
        slave.input_offset = static_cast<unsigned int>(offset);
      }
    }
    return true;
  }

  std::uint32_t cycleTimeNs() const
  {
    const auto ns = static_cast<long long>(std::llround(config.cycle_time_ms * 1'000'000.0));
    return ns > 0 ? static_cast<std::uint32_t>(ns) : 2'000'000u;
  }

  // 把当前所有从站的状态整理成一段可直接写入日志的摘要。
  //
  // 这比只打一个 "not all slaves reached OPERATIONAL" 更有诊断价值，能直接看出
  // 是哪个从站卡在 PRE-OP / SAFE-OP / OP 迁移上。
  std::string slaveStateSummary() const
  {
    std::string summary;
    for (std::size_t i = 0; i < slaves.size(); ++i) {
      if (!summary.empty()) {
        summary += "; ";
      }
      summary += "slave" + std::to_string(i + 1);
      if (slaves[i].config == nullptr) {
        summary += "{config=null}";
        continue;
      }

      ec_slave_config_state_t slave_state;
      std::memset(&slave_state, 0, sizeof(slave_state));
      ecrt_slave_config_state(slaves[i].config, &slave_state);

      summary += "{al=" + std::to_string(static_cast<unsigned int>(slave_state.al_state));
      summary += ", operational=" + std::string(slave_state.operational ? "true" : "false");
      summary += "}";
    }
    return summary;
  }

  // 刷新从站状态快照，供上层故障判断和状态上报使用。
  void refreshSlaveStates()
  {
    // 采集每个从站的 AL 状态，并把结果转换成统一的 EtherCATRuntimeState。
    state.slaves.clear();
    state.detected_slave_count = static_cast<int>(slaves.size());
    state.slaves.reserve(slaves.size());
    bool all_operational = configured && !slaves.empty();
    for (std::size_t i = 0; i < slaves.size(); ++i) {
      ec_slave_config_state_t slave_state;
      std::memset(&slave_state, 0, sizeof(slave_state));
      ecrt_slave_config_state(slaves[i].config, &slave_state);

      EthercatSlaveState item;
      item.slave_index = static_cast<int>(i + 1);
      item.name = "igh_slave_" + std::to_string(i + 1);
      item.state = static_cast<int>(slave_state.al_state);
      item.online = slave_state.operational;
      item.lost = false;
      state.slaves.push_back(item);
      all_operational = all_operational && item.online;
    }
    state.operational = all_operational;
  }

#endif

  // 根据当前从站状态更新状态摘要。
  //
  // 这相当于旧 ethercat_manager 里 ecat_check_loop 的巡检职责：定期读取 slave_config_state，
  // 让上层知道哪些从站不在 OP，而不是只依赖周期通信的 WKC 结果。
  bool checkSlaveStates()
  {
#ifndef ROBOT_DRIVER_HAS_IGH
    setError("IGH slave monitor requested but IGH support is not built");
    return false;
#else
    refreshSlaveStates();
    if (!state.configured || slaves.empty()) {
      return false;
    }

    bool all_operational = true;
    for (const auto & slave : state.slaves) {
      all_operational = all_operational && slave.online;
    }

    if (all_operational) {
      if (state.last_error == "IGH slave monitor recovery pending") {
        state.last_error.clear();
      }
      state.operational = true;
      return true;
    }

    state.operational = false;
    state.last_error = "IGH slave monitor recovery pending: " + slaveStateSummary();
    return false;
#endif
  }

  void startMonitorThread()
  {
    if (monitor_running.exchange(true)) {
      return;
    }
    monitor_thread = std::thread([this]() {
      while (monitor_running.load()) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          if (configured && monitor_enabled.load()) {
            (void)checkSlaveStates();
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    });
  }

  void stopMonitorThread()
  {
    if (!monitor_running.exchange(false)) {
      return;
    }
    if (monitor_thread.joinable()) {
      monitor_thread.join();
    }
  }
};

// 构造 IGH 主站封装，保留总线配置供后续 open/configure 使用。
IghMaster::IghMaster(const EthercatBusConfig & config)
: impl_(std::make_unique<Impl>(config))
{
}

// 析构时关闭主站并释放底层资源。
IghMaster::~IghMaster()
{
  close();
}

// 打开 IGH 主站并请求底层 master。
bool IghMaster::open(const std::string & network_interface)
{
  (void)network_interface;
  impl_->stopMonitorThread();
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->state = EthercatRuntimeState{};
  impl_->state.expected_slave_count = impl_->config.slave_count;

#ifndef ROBOT_DRIVER_HAS_IGH
  impl_->setError("IGH EtherCAT support is not built: ecrt.h/libethercat not found");
  return false;
#else
  if (impl_->master != nullptr) {
    ecrt_release_master(impl_->master);
    impl_->master = nullptr;
    impl_->domain = nullptr;
    impl_->domain_data = nullptr;
    impl_->slaves.clear();
    impl_->output_buffers.clear();
  }
  impl_->master = ecrt_request_master(0);
  if (impl_->master == nullptr) {
    // 只请求编号 0 的主站，因为当前系统只支持单主站天工网关。
    impl_->setError("IGH failed to request master 0");
    return false;
  }
  impl_->opened = true;
  impl_->state.opened = true;
  impl_->state.last_error.clear();
  return true;
#endif
}

// 按预期从站数量完成 PDO 注册，并等待从站进入 OPERATIONAL。
bool IghMaster::configure(int expected_slave_count)
{
  impl_->stopMonitorThread();
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->state.expected_slave_count = expected_slave_count;

#ifndef ROBOT_DRIVER_HAS_IGH
  impl_->setError("IGH EtherCAT support is not built: ecrt.h/libethercat not found");
  return false;
#else
  if (!impl_->opened || impl_->master == nullptr) {
    impl_->setError("IGH configure requested before open");
    return false;
  }
  impl_->domain = ecrt_master_create_domain(impl_->master);
  if (impl_->domain == nullptr) {
    impl_->setError("IGH failed to create process data domain");
    return false;
  }

  ec_master_state_t master_state;
  std::memset(&master_state, 0, sizeof(master_state));
  ecrt_master_state(impl_->master, &master_state);
  const auto detected = static_cast<int>(master_state.slaves_responding);
  impl_->state.detected_slave_count = detected;
  if (detected <= 0) {
    impl_->setError("IGH no slaves responding on bus");
    return false;
  }
  if (expected_slave_count > 0 && detected != expected_slave_count) {
    impl_->setError(
      "IGH slave count mismatch: expected " + std::to_string(expected_slave_count) +
      ", detected " + std::to_string(detected));
    return false;
  }

  // 先按检测到的从站数量分配输出缓冲区，再逐个绑定 vendor/product 对应的从站配置。
  impl_->slaves.clear();
  impl_->slaves.resize(static_cast<std::size_t>(detected));
  impl_->output_buffers.assign(
    static_cast<std::size_t>(detected),
    std::vector<std::uint8_t>(kGatewayPdoSize, 0));

  // 逐个从站注册 PDO，确保输入输出缓冲区和天工网关布局一致。
  for (int slave = 0; slave < detected; ++slave) {
    auto & info = impl_->slaves[static_cast<std::size_t>(slave)];
    info.config = ecrt_master_slave_config(
      impl_->master, 0, static_cast<std::uint16_t>(slave),
      kGatewayVendorId, kGatewayProductCode);
    if (info.config == nullptr) {
      impl_->setError("IGH failed to get slave config for position " + std::to_string(slave));
      return false;
    }
    // PDO 映射成功后，后续周期交换才能直接按缓冲区读写。
    if (!impl_->registerPdoEntries(info)) {
      return false;
    }
  }

  timespec now;
  clock_gettime(CLOCK_REALTIME, &now);
  ecrt_master_application_time(
    impl_->master,
    static_cast<std::uint64_t>(now.tv_sec) * 1000000000ull +
    static_cast<std::uint64_t>(now.tv_nsec));

  if (ecrt_master_activate(impl_->master) != 0) {
    impl_->setError("IGH failed to activate master");
    return false;
  }
  impl_->domain_data = ecrt_domain_data(impl_->domain);
  if (impl_->domain_data == nullptr) {
    impl_->setError("IGH failed to access process domain data");
    return false;
  }

  impl_->expected_work_counter = detected * 3;
  impl_->state.expected_work_counter = impl_->expected_work_counter;
  impl_->configured = true;
  impl_->state.configured = true;

  // 配置完成后先跑一段短轮询，等待所有从站进入 OPERATIONAL。
  {
    int retry = 0;
    const int max_retries = 1000;
    bool all_op = false;

    for (retry = 0; retry < max_retries; ++retry) {
      ecrt_master_receive(impl_->master);
      ecrt_domain_process(impl_->domain);

      if (retry % 10 == 0) {
        timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ecrt_master_application_time(
          impl_->master,
          static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ull +
          static_cast<std::uint64_t>(ts.tv_nsec));
        ecrt_master_sync_reference_clock(impl_->master);
      }

      all_op = true;
      for (std::size_t slave = 0; slave < impl_->slaves.size(); ++slave) {
        ec_slave_config_state_t slave_state;
        std::memset(&slave_state, 0, sizeof(slave_state));
        ecrt_slave_config_state(impl_->slaves[slave].config, &slave_state);
        if (!slave_state.operational) {
          all_op = false;
          break;
        }
      }

      if (all_op) {
        break;
      }

      if (retry % 20 == 0 && !impl_->slaves.empty()) {
        ec_slave_config_state_t slave_state;
        std::memset(&slave_state, 0, sizeof(slave_state));
        ecrt_slave_config_state(impl_->slaves[0].config, &slave_state);
        std::fprintf(
          stderr,
          "[robot_driver_node] IGH waiting OP: state=0x%02X retry=%d/%d\n",
          static_cast<unsigned int>(slave_state.al_state),
          retry,
          max_retries);
      }

      for (std::size_t slave = 0; slave < impl_->slaves.size(); ++slave) {
        std::memset(impl_->output_buffers[slave].data(), 0, kGatewayPdoSize);
        std::memset(
          impl_->domain_data + impl_->slaves[slave].output_offset,
          0,
          kGatewayPdoSize);
      }
      ecrt_domain_queue(impl_->domain);
      ecrt_master_send(impl_->master);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (all_op) {
      impl_->state.operational = true;
      impl_->state.last_error.clear();
      if (impl_->monitor_enabled.load()) {
        impl_->startMonitorThread();
      }
      return true;
    }
  }

  impl_->setError(
    "IGH not all slaves reached OPERATIONAL: " + impl_->slaveStateSummary());
  return false;
#endif
}

void IghMaster::setMonitorEnabled(bool enabled)
{
  impl_->monitor_enabled.store(enabled);
  if (!enabled) {
    impl_->stopMonitorThread();
    return;
  }

  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->configured && impl_->state.operational && !impl_->monitor_running.load()) {
    impl_->startMonitorThread();
  }
}

// 执行一次 EtherCAT 周期交换，并更新工作计数和从站状态。
bool IghMaster::exchangeProcessData()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);

#ifndef ROBOT_DRIVER_HAS_IGH
  impl_->setError("IGH EtherCAT support is not built: ecrt.h/libethercat not found");
  return false;
#else
  if (!impl_->configured || impl_->master == nullptr || impl_->domain == nullptr || impl_->domain_data == nullptr) {
    impl_->setError("IGH process data exchange requested before configure");
    return false;
  }

  // 一个周期内的标准顺序：接收主站数据 -> 处理 domain -> 拷贝输出 -> 发送。
  ecrt_master_receive(impl_->master);
  ecrt_domain_process(impl_->domain);

  ec_domain_state_t domain_state;
  std::memset(&domain_state, 0, sizeof(domain_state));
  ecrt_domain_state(impl_->domain, &domain_state);
  impl_->state.actual_work_counter = static_cast<int>(domain_state.working_counter);

  if (impl_->work_counter_iteration_count % 10 == 0) {
    timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    ecrt_master_application_time(
      impl_->master,
      static_cast<std::uint64_t>(now.tv_sec) * 1000000000ull +
      static_cast<std::uint64_t>(now.tv_nsec));
    ecrt_master_sync_reference_clock(impl_->master);
  }

  // 把上层写入的缓冲区拷到 domain，再统一交给 IGH 发送到底层总线。
  for (std::size_t slave = 0; slave < impl_->slaves.size(); ++slave) {
    std::memcpy(
      impl_->domain_data + impl_->slaves[slave].output_offset,
      impl_->output_buffers[slave].data(),
      kGatewayPdoSize);
  }

  ecrt_domain_queue(impl_->domain);
  ecrt_master_send(impl_->master);

  if (impl_->work_counter_iteration_count >= kWorkCounterErrorPeriod) {
    impl_->work_counter_iteration_count = 0;
    impl_->work_counter_error_count = 0;
  }
  ++impl_->work_counter_iteration_count;

  const bool wkc_healthy =
    impl_->expected_work_counter <= 0 ||
    impl_->state.actual_work_counter >= impl_->expected_work_counter;
  if (!wkc_healthy) {
    ++impl_->work_counter_error_count;
  }
  impl_->refreshSlaveStates();
  if (!wkc_healthy && impl_->work_counter_error_count > kWorkCounterErrorMax) {
    // impl_->setError(
    //   "IGH workcounter abnormal: expected " +
    //   std::to_string(impl_->expected_work_counter) + ", actual " +
    //   std::to_string(impl_->state.actual_work_counter));
    // return false;
  }
  if (wkc_healthy && impl_->state.operational) {
    impl_->state.last_error.clear();
  }
  return true;
#endif
}

// 轮询从站状态，用于故障检测和恢复判断。
bool IghMaster::monitorSlaves()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
#ifndef ROBOT_DRIVER_HAS_IGH
  impl_->setError("IGH EtherCAT support is not built: ecrt.h/libethercat not found");
  return false;
#else
  if (!impl_->configured) {
    impl_->setError("IGH slave monitor requested before configure");
    return false;
  }
  return impl_->checkSlaveStates();
#endif
}

// 获取指定逻辑从站的输出缓冲区。
std::uint8_t * IghMaster::outputBuffer(int logical_slave_id)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->configured || logical_slave_id < 1 ||
      logical_slave_id > static_cast<int>(impl_->output_buffers.size())) {
    impl_->setError("IGH output slave_id out of range: " + std::to_string(logical_slave_id));
    return nullptr;
  }
  return impl_->output_buffers[static_cast<std::size_t>(logical_slave_id - 1)].data();
}

// 获取指定逻辑从站的输出缓冲区大小。
std::size_t IghMaster::outputBufferSize(int logical_slave_id) const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->configured || logical_slave_id < 1 ||
      logical_slave_id > static_cast<int>(impl_->output_buffers.size())) {
    return 0;
  }
  return impl_->output_buffers[static_cast<std::size_t>(logical_slave_id - 1)].size();
}

// 获取指定逻辑从站的输入缓冲区。
const std::uint8_t * IghMaster::inputBuffer(int logical_slave_id) const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
#ifndef ROBOT_DRIVER_HAS_IGH
  (void)logical_slave_id;
  return nullptr;
#else
  if (!impl_->configured || logical_slave_id < 1 ||
      logical_slave_id > static_cast<int>(impl_->slaves.size()) ||
      impl_->domain_data == nullptr) {
    return nullptr;
  }
  return impl_->domain_data + impl_->slaves[static_cast<std::size_t>(logical_slave_id - 1)].input_offset;
#endif
}

// 获取指定逻辑从站的输入缓冲区大小。
std::size_t IghMaster::inputBufferSize(int logical_slave_id) const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->configured || logical_slave_id < 1 ||
      logical_slave_id > static_cast<int>(impl_->output_buffers.size())) {
    return 0;
  }
  return kGatewayPdoSize;
}

// 关闭主站并释放底层资源。
void IghMaster::close()
{
  impl_->stopMonitorThread();
  std::lock_guard<std::mutex> lock(impl_->mutex);
#ifdef ROBOT_DRIVER_HAS_IGH
  if (impl_->master != nullptr) {
    ecrt_release_master(impl_->master);
  }
  impl_->master = nullptr;
  impl_->domain = nullptr;
  impl_->domain_data = nullptr;
  impl_->slaves.clear();
#endif
  impl_->opened = false;
  impl_->configured = false;
  impl_->output_buffers.clear();
  impl_->state.opened = false;
  impl_->state.configured = false;
  impl_->state.operational = false;
  impl_->state.last_error.clear();
}

// 返回当前 EtherCAT 运行状态快照。
EthercatRuntimeState IghMaster::currentState() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->state;
}

}  // namespace robot_driver
