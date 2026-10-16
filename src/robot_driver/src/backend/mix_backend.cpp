#include "robot_driver/backend/mix_backend.hpp"

#include <algorithm>
#include <ctime>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <cstdio>

#include <rclcpp/rclcpp.hpp>

#include "robot_driver/ethercat/igh_master.hpp"

namespace robot_driver
{

namespace
{

static const auto kLogger = rclcpp::get_logger("mix_backend");
static rclcpp::Clock kThrottleClock(RCL_STEADY_TIME);

// 把 Encos 协议层管理结果转换成 mix service 统一返回结构。
MixAdminResult toMixResult(const HardwareJointConfig & hardware_joint, const EncosAdminResult & admin_result)
{
  MixAdminResult result;
  result.success = admin_result.success;
  result.message = admin_result.message;
  result.vendor = hardware_joint.vendor;
  result.motor_id = admin_result.motor_id != 0 ?
    admin_result.motor_id :
    static_cast<std::uint16_t>(hardware_joint.motor_id);
  result.new_motor_id = admin_result.new_motor_id;
  result.feedback_code = admin_result.feedback_code;
  result.position = admin_result.value;
  return result;
}

// 把 Tienkung 协议层管理结果转换成 mix service 统一返回结构。
MixAdminResult toMixResult(const HardwareJointConfig & hardware_joint, const TienkungAdminResult & admin_result)
{
  MixAdminResult result;
  result.success = admin_result.success;
  result.message = admin_result.message;
  result.vendor = hardware_joint.vendor;
  result.motor_id = admin_result.motor_id != 0 ?
    admin_result.motor_id :
    static_cast<std::uint16_t>(hardware_joint.motor_id);
  result.new_motor_id = admin_result.new_motor_id;
  result.feedback_code = admin_result.feedback_code;
  result.zero_position_rad = admin_result.zero_position_rad;
  result.position = admin_result.position;
  result.velocity = admin_result.velocity;
  result.effort = admin_result.effort;
  result.motor_error = admin_result.motor_error;
  result.motor_temp = admin_result.motor_temp;
  result.mos_temp = admin_result.mos_temp;
  return result;
}

}  // namespace

std::unique_ptr<CanfdTransport> createMixTransport(const HardwareConfig & hardware_config)
{
  if (hardware_config.bus_type == "ethercat") {
    return std::make_unique<GatewayCanfdTransport>(
      std::make_unique<IghMaster>(hardware_config.ethercat));
  }
  return std::make_unique<SocketCanfdTransport>();
}

// 正常运行构造函数：根据 bus_type 创建真实 transport 后委托到注入构造函数。
MixBackend::MixBackend(const BackendConfig & config)
: MixBackend(config, createMixTransport(config.hardware_config))
{
}

// 注入式构造函数：加载配置、初始化关节运行时状态，并打开 CANFD transport。
//
// 单元测试通过该入口注入 fake transport，从而验证编码/解码逻辑而不访问真实硬件。
MixBackend::MixBackend(const BackendConfig & config, std::unique_ptr<CanfdTransport> transport)
: HardwareBackend(config, "mix"),
  transport_(std::move(transport))
{
  if (!validateConfig()) {
    return;
  }

  // JointStates 的顺序必须跟 robot_joints.yaml 一致，而不是硬件配置文件顺序。
  initializeJoints();

  // 从当前 transport 的总线配置加载循环时间；未配置时由基类保持默认值。
  setCycleTimeMs(
    hardware_config_.bus_type == "ethercat" ?
    hardware_config_.ethercat.cycle_time_ms :
    hardware_config_.canfd.cycle_time_ms);
    
  // 初始化 CANFD 链路。
  initCanfd();

  // 初始化后，根据当前状态打印日志
  if (canfd_configured_ && !fault_active_.load()) {
    RCLCPP_INFO(kLogger, "%s", config_mode_ ? "mix backend is motor config" : "mix backend is motor motion");
  } else {
    RCLCPP_INFO(kLogger, "%s", "mix backend is fault");
  }
}

// 析构时停止后台线程并关闭 CANFD master。
MixBackend::~MixBackend()
{
  stopWorker();
  if (transport_) {
    transport_->close();
  }
}

// 校验 mix 后端所需的硬件配置。
bool MixBackend::validateConfig()
{
  // mix backend 是混装协议入口，底层允许 SocketCAN 直连或 IGH gateway 两种 transport。
  if (hardware_config_.vendor_type != "mix") {
    setFault("mix backend requires hardware.vendor_type to be 'mix'");
    return false;
  }
  if (hardware_config_.bus_type != "canfd" && hardware_config_.bus_type != "ethercat") {
    setFault("mix backend requires hardware.bus_type to be 'canfd' or 'ethercat'");
    return false;
  }
  for (const auto & hardware_joint : hardware_config_.joints) {
    // 硬件映射里的每个关节都必须存在于统一 robot_joints.yaml 中。
    if (!joints_config_.hasJoint(hardware_joint.name)) {
      setFault("mix hardware joint '" + hardware_joint.name + "' does not exist in robot_joints.yaml");
      return false;
    }
    if (hardware_joint.vendor != "encos" && hardware_joint.vendor != "tienkung") {
      setFault("mix joint '" + hardware_joint.name + "' vendor must be encos or tienkung");
      return false;
    }
  }
  return true;
}

// 初始化 CANFD transport。
//
// SocketCAN 路径打开 canfd.interfaces；EtherCAT 路径打开 IGH master 并按 gateway 槽位建立映射。
void MixBackend::initCanfd()
{
  if (!transport_) {
    recordCanfdFault("mix CANFD transport is not available");
    return;
  }
  if (!transport_->open(hardware_config_)) {
    recordCanfdFault("CANFD open failed");
    return;
  }
  if (!transport_->configure(hardware_config_)) {
    recordCanfdFault("CANFD configure failed");
    return;
  }
  fault_active_ = false;
  canfd_configured_ = true;
  status_message_ = "mix CANFD online";
}

// 记录带 CANFD transport 细节的故障，并退出已配置状态。
void MixBackend::recordCanfdFault(const std::string & message)
{
  FaultDetail detail;
  detail.source = "mix_backend";
  detail.transport = hardware_config_.bus_type;
  detail.message = message;
  if (transport_) {
    detail.transport_error = transport_->lastError();
  }
  recordFault(detail);
  canfd_configured_ = false;
}

// 编码单个关节命令。
//
// 根据关节配置中的 vendor 字段选择 EncosProtocol 或 TienkungProtocol。
bool MixBackend::encodeJointCommand(const robot_interfaces::msg::JointCommand & command)
{
  // 查询关节运行时数组下标。
  const auto * hardware_joint = hardware_config_.getJoint(command.joint_name);
  if (hardware_joint == nullptr) {
    setFault("mix hardware mapping missing for joint: " + command.joint_name);
    return false;
  }

  // CANFD 信箱按 hardware.joints 顺序建立，这里用关节名换算成信箱下标。
  const auto canfd_index = transportJointIndex(command.joint_name);
  // 编码并写入 CANFD 帧。
  CanfdFrame frame;
  std::string error;
  // Encos 和 Tienkung 的 CANFD 帧格式不同；mix backend 只做路由，不解释协议字段。
  if (hardware_joint->vendor == "encos") {
    if (!encos_protocol_.encodeMotion(*hardware_joint, command, frame, error)) {
      setFault(error);
      return false;
    }
    if (!transport_->setOutputFrame(canfd_index, frame)) {
      setFault("CANFD output buffer unavailable for joint: " + command.joint_name);
      return false;
    }
    return true;
  }
  else if (hardware_joint->vendor == "tienkung") {
    if (!tienkung_protocol_.encodeMotion(*hardware_joint, command, command_counter_++, frame, error)) {
      setFault(error);
      return false;
    }
    if (!transport_->setOutputFrame(canfd_index, frame)) {
      setFault("CANFD output buffer unavailable for joint: " + command.joint_name);
      return false;
    }
    return true;
  }
  setFault("unsupported mix joint vendor: " + hardware_joint->vendor);
  return false;
}

// 执行 mix 中天工关节的启动序列。
//
// 行为对齐 TienkungBackend：逐个天工关节执行断使能/使能，并等待 0x90 ACK。
bool MixBackend::runStartup()
{
  if (!runTienkungStartStop(0, "startup disable")) {
    return false;
  }

  // 等待 50ms 确保断使能生效
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  if (!runTienkungStartStop(1, "startup enable")) {
    return false;
  }
  return true;
}

// 按单关节事务依次执行 mix 中天工关节的启停状态切换。
bool MixBackend::runTienkungStartStop(std::uint8_t state, const std::string & action)
{
  if (!canfd_configured_ || !transport_) {
    return false;
  }
  for (auto & joint : joints_) {
    const auto * hardware_joint = hardware_config_.getJoint(joint.name);
    if (hardware_joint == nullptr || hardware_joint->vendor != "tienkung") {
      continue;
    }
    if (!jointDefaultEnabled(joint.name)) {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      joint.enabled = false;
      continue;
    }
    if (!syncTienkungStartStop(joint, state, action)) {
      return false;
    }
  }
  return true;
}

// 对 mix 中单个天工关节执行一次启停事务：写指令、交换数据、等待 ACK。
bool MixBackend::syncTienkungStartStop(
  JointRuntime & joint,
  std::uint8_t state,
  const std::string & action)
{
  if (!jointDefaultEnabled(joint.name)) {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    joint.enabled = false;
    return true;
  }

  const auto * hardware_joint = hardware_config_.getJoint(joint.name);
  if (hardware_joint == nullptr) {
    setFault("mix hardware mapping missing while sending Tienkung start/stop for joint: " + joint.name);
    return false;
  }

  CanfdFrame frame;
  std::string error;
  const bool ok = state != 0 ? tienkung_protocol_.encodeEnable(*hardware_joint, frame, error) :
                               tienkung_protocol_.encodeDisable(*hardware_joint, frame, error);
  if (!ok) {
    setFault(error.empty() ? "Tienkung start/stop encode failed for joint: " + joint.name : error);
    return false;
  }
  if (!transport_->setOutputFrame(transportJointIndex(hardware_joint->name), frame)) {
    setFault("CANFD output frame unavailable while sending Tienkung start/stop frame: " + joint.name);
    return false;
  }

  return waitTienkungStartStop(joint, *hardware_joint, state, 10, action);
}

// 等待 mix 中单个天工关节返回指定启停应答。
bool MixBackend::waitTienkungStartStop(
  JointRuntime & joint,
  const HardwareJointConfig & hardware_joint,
  std::uint8_t expected_state,
  int max_cycles,
  const std::string & action)
{
  std::string last_error;
  for (int cycle = 0; cycle < max_cycles; ++cycle) {
    if (!transport_->exchangeProcessData()) {
      recordCanfdFault("CANFD exchange failed while waiting for Tienkung " + action + ": " + hardware_joint.name);
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    const auto canfd_index = transportJointIndex(hardware_joint.name);
    CanfdFrame frame;
    if (!transport_->getInputFrame(canfd_index, frame)) {
      setFault("CANFD input frame unavailable while checking Tienkung start/stop ack: " + hardware_joint.name);
      return false;
    }
    bool enabled = false;
    std::string error;
    if (tienkung_protocol_.decodeStartStop(hardware_joint, frame, enabled, error)) {
      if ((expected_state != 0) != enabled) {
        continue;
      }
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      joint.enabled = enabled;
      return true;
    }
    if (!error.empty()) {
      last_error = error;
    }
  }

  setFault(
    "Tienkung timeout waiting for mix " + action + ": " + hardware_joint.name +
    (last_error.empty() ? "" : ", last_error=" + last_error));
  return false;
}

// 判断 Mix CANFD 链路是否已经可用于统一控制周期。
bool MixBackend::transportReady() const
{
  return canfd_configured_ && transport_;
}

// 执行一次 CANFD process data 交换。
//
// 该函数只负责通讯层收发和必要等待，反馈解析统一由 readJointFeedback() 完成。
bool MixBackend::exchangeProcessData()
{
  if (!transport_->exchangeProcessData()) {
    recordCanfdFault("CANFD process data exchange failed");
    return false;
  }
  return true;
}

// 返回 Mix CANFD 是否已经完成 open/configure。
bool MixBackend::configured() const
{
  return canfd_configured_;
}

// 按关节名查找 CANFD 信箱下标。
//
// configure() 使用 hardware_config_.joints 顺序建信箱，因此这里保持同一个顺序。
std::size_t MixBackend::transportJointIndex(const std::string & joint_name) const
{
  for (std::size_t index = 0; index < hardware_config_.joints.size(); ++index) {
    if (hardware_config_.joints[index].name == joint_name) {
      return index;
    }
  }
  return hardware_config_.joints.size();
}

// 查询管理事务输出信箱。
bool MixBackend::setAdminOutputFrame(
  const HardwareJointConfig & hardware_joint,
  const CanfdFrame & frame,
  MixAdminResult & result)
{
  const auto canfd_index = transportJointIndex(hardware_joint.name);
  if (!transport_ || !transport_->setOutputFrame(canfd_index, frame)) {
    result.message = "CANFD output frame unavailable for joint: " + hardware_joint.name;
    return false;
  }
  return true;
}

// 查询管理事务输入信箱。
bool MixBackend::getAdminInputFrame(const HardwareJointConfig & hardware_joint, CanfdFrame & frame) const
{
  const auto canfd_index = transportJointIndex(hardware_joint.name);
  return transport_ && transport_->getInputFrame(canfd_index, frame);
}


// 按关节名查询当前状态。
MixAdminResult MixBackend::queryStatus(const std::string & joint_name, double timeout_sec)
{
  std::lock_guard<std::mutex> admin_lock(admin_mutex_);
  MixAdminResult result;

  if (!config_mode_) {
    result.message = "mix admin services only available in config_mode";
    return result;
  }
  if (!canfd_configured_ || !transport_ || fault_active_.load()) {
    result.message = "mix CANFD backend not ready";
    return result;
  }
  const auto * hardware_joint = hardware_config_.getJoint(joint_name);
  if (hardware_joint == nullptr) {
    result.message = "mix hardware mapping missing for joint: " + joint_name;
    return result;
  }
  result.vendor = hardware_joint->vendor;
  result.motor_id = static_cast<std::uint16_t>(hardware_joint->motor_id);

  std::string error;
  CanfdFrame frame;
  if (hardware_joint->vendor == "tienkung") {
    robot_interfaces::msg::JointCommand command;
    command.joint_name = joint_name;
    command.enable = true;
    command.position = 0.0F;
    command.velocity = 0.0F;
    command.effort = 0.0F;
    command.kp = 0.0F;
    command.kd = 0.0F;
    if (!tienkung_protocol_.encodeMotion(*hardware_joint, command, 0, frame, error)) {
      result.message = "mix tienkung encode query status fallback mixed command failed: " + error;
      return result;
    }
  } else {
    if (!encos_protocol_.encodeQueryParam(hardware_joint->motor_id, 1, frame, error)) {
      result.message = "mix encos encode query position failed: " + error;
      return result;
    }
  }
  if (!setAdminOutputFrame(*hardware_joint, frame, result)) {
    return result;
  }

  const auto safe_timeout = std::max(timeout_sec, 0.001);
  const int max_cycles = std::max(1, static_cast<int>(safe_timeout * 1000.0 / 2.0));
  for (int cycle = 0; cycle < max_cycles; ++cycle) {
    if (!transport_->exchangeProcessData()) {
      result.message = "mix CANFD exchange failed while querying status";
      return result;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    CanfdFrame input_frame;
    if (!getAdminInputFrame(*hardware_joint, input_frame) || input_frame.length == 0) {
      continue;
    }

    if (hardware_joint->vendor == "tienkung") {
      TienkungAdminResult tienkung_result;
      if (tienkung_protocol_.decodeQueryStatus(*hardware_joint, input_frame, tienkung_result, error)) {
        result = toMixResult(*hardware_joint, tienkung_result);
        result.message = "mix tienkung query status completed";
        return result;
      }
    } else {
      EncosAdminResult encos_result;
      if (encos_protocol_.decodeQueryParam(
        static_cast<std::uint16_t>(hardware_joint->motor_id), 1, input_frame, encos_result, error))
      {
        result = toMixResult(*hardware_joint, encos_result);
        result.message = result.success ? "mix encos query status completed" : encos_result.message;
        return result;
      }
    }
  }

  result.message = "mix query status timed out";
  return result;
}

// 按关节名设置电机零点。
MixAdminResult MixBackend::setZero(const std::string & joint_name, double timeout_sec)
{
  std::lock_guard<std::mutex> admin_lock(admin_mutex_);
  MixAdminResult result;

  if (!config_mode_) {
    result.message = "mix admin services only available in config_mode";
    return result;
  }
  if (!canfd_configured_ || !transport_ || fault_active_.load()) {
    result.message = "mix CANFD backend not ready";
    return result;
  }
  const auto * hardware_joint = hardware_config_.getJoint(joint_name);
  if (hardware_joint == nullptr) {
    result.message = "mix hardware mapping missing for joint: " + joint_name;
    return result;
  }

  if (hardware_joint->vendor == "encos") {
    return setEncosJointZero(*hardware_joint, timeout_sec);
  }
  if (hardware_joint->vendor == "tienkung") {
    return setTienkungJointZero(*hardware_joint, timeout_sec);
  }

  result.message = "unsupported mix joint vendor: " + hardware_joint->vendor;
  return result;
}

MixAdminResult MixBackend::setEncosJointZero(
  const HardwareJointConfig & hardware_joint,
  double timeout_sec)
{
  MixAdminResult result;
  result.vendor = hardware_joint.vendor;
  result.motor_id = static_cast<std::uint16_t>(hardware_joint.motor_id);

  std::string error;
  CanfdFrame request;
  if (!encos_protocol_.encodeSetZero(hardware_joint.motor_id, request, error)) {
    result.message = "mix encos encode set zero failed: " + error;
    return result;
  }

  const auto safe_timeout = std::max(timeout_sec, 0.001);
  const auto timeout_us = std::max(1, static_cast<int>(safe_timeout * 1000000.0));
  CanfdFrame response;
  if (!transport_->exchangeBusFrame(hardware_joint, request, response, timeout_us)) {
    result.message = "mix CANFD bus exchange failed while setting zero";
    return result;
  }

  EncosAdminResult encos_result;
  if (encos_protocol_.decodeSetZero(
    static_cast<std::uint16_t>(hardware_joint.motor_id), response, encos_result, error))
  {
    result = toMixResult(hardware_joint, encos_result);
    result.message = result.success ? "mix encos set zero completed" : encos_result.message;
    return result;
  }

  result.message = "mix encos set zero decode failed: " + error;
  return result;
}

MixAdminResult MixBackend::setTienkungJointZero(
  const HardwareJointConfig & hardware_joint,
  double timeout_sec)
{
  MixAdminResult result;
  result.vendor = hardware_joint.vendor;
  result.motor_id = static_cast<std::uint16_t>(hardware_joint.motor_id);

  std::string error;
  CanfdFrame frame;
  if (!tienkung_protocol_.encodeSetZero(hardware_joint, frame, error)) {
    result.message = "mix tienkung encode set zero failed: " + error;
    return result;
  }
  if (!setAdminOutputFrame(hardware_joint, frame, result)) {
    return result;
  }

  const auto safe_timeout = std::max(timeout_sec, 0.001);
  const int max_cycles = std::max(1, static_cast<int>(safe_timeout * 1000.0 / 2.0));
  for (int cycle = 0; cycle < max_cycles; ++cycle) {
    if (!transport_->exchangeProcessData()) {
      result.message = "mix CANFD exchange failed while setting zero";
      return result;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    CanfdFrame input_frame;
    if (!getAdminInputFrame(hardware_joint, input_frame) || input_frame.length == 0) {
      continue;
    }

    TienkungAdminResult tienkung_result;
    if (tienkung_protocol_.decodeSetZero(hardware_joint, input_frame, tienkung_result, error)) {
      result = toMixResult(hardware_joint, tienkung_result);
      result.message = result.success ? "mix tienkung set zero completed" : "mix tienkung set zero failed";
      return result;
    }
  }

  result.message = "mix set zero timed out";
  return result;
}

// 按关节名查询电机 ID。
MixAdminResult MixBackend::queryID(const std::string & joint_name, double timeout_sec)
{
  std::lock_guard<std::mutex> admin_lock(admin_mutex_);
  MixAdminResult result;

  if (!config_mode_) {
    result.message = "mix admin services only available in config_mode";
    return result;
  }
  if (!canfd_configured_ || !transport_ || fault_active_.load()) {
    result.message = "mix CANFD backend not ready";
    return result;
  }
  const auto * hardware_joint = hardware_config_.getJoint(joint_name);
  if (hardware_joint == nullptr) {
    result.message = "mix hardware mapping missing for joint: " + joint_name;
    return result;
  }
  result.vendor = hardware_joint->vendor;
  result.motor_id = static_cast<std::uint16_t>(hardware_joint->motor_id);

  std::string error;
  CanfdFrame request;
  if (hardware_joint->vendor == "tienkung") {
    if (!tienkung_protocol_.encodeQueryId(request, error)) {
      result.message = "mix tienkung encode query id failed: " + error;
      return result;
    }
  } else if (!encos_protocol_.encodeQueryId(request, error)) {
    result.message = "mix encos encode query id failed: " + error;
    return result;
  }

  const auto safe_timeout = std::max(timeout_sec, 0.001);
  const auto timeout_us = std::max(1, static_cast<int>(safe_timeout * 1000000.0));
  CanfdFrame response;
  if (!transport_->exchangeBusFrame(*hardware_joint, request, response, timeout_us)) {
    result.message = "mix CANFD bus exchange failed while querying id";
    return result;
  }

  if (hardware_joint->vendor == "tienkung") {
    TienkungAdminResult tienkung_result;
    if (tienkung_protocol_.decodeQueryId(response, tienkung_result, error)) {
      result = toMixResult(*hardware_joint, tienkung_result);
      result.message = "mix tienkung query id completed";
      return result;
    }
  } else {
    EncosAdminResult encos_result;
    if (encos_protocol_.decodeQueryId(response, encos_result, error)) {
      result = toMixResult(*hardware_joint, encos_result);
      result.message = result.success ? "mix encos query id completed" : encos_result.message;
      return result;
    }
  }
  result.message = "mix query id decode failed: " + error;
  return result;
}

// 按关节名修改电机 ID。
MixAdminResult MixBackend::modifyID(
  const std::string & joint_name,
  std::uint16_t new_motor_id,
  double timeout_sec)
{
  std::lock_guard<std::mutex> admin_lock(admin_mutex_);
  MixAdminResult result;

  if (!config_mode_) {
    result.message = "mix admin services only available in config_mode";
    return result;
  }
  if (!canfd_configured_ || !transport_ || fault_active_.load()) {
    result.message = "mix CANFD backend not ready";
    return result;
  }
  const auto * hardware_joint = hardware_config_.getJoint(joint_name);
  if (hardware_joint == nullptr) {
    result.message = "mix hardware mapping missing for joint: " + joint_name;
    return result;
  }
  result.vendor = hardware_joint->vendor;
  result.motor_id = static_cast<std::uint16_t>(hardware_joint->motor_id);
  result.new_motor_id = new_motor_id;

  std::string error;
  CanfdFrame frame;
  if (hardware_joint->vendor == "tienkung") {
    if (new_motor_id > 255) {
      result.message = "tienkung new motor id must be <= 255";
      return result;
    }
    if (!tienkung_protocol_.encodeModifyId(*hardware_joint, static_cast<std::uint8_t>(new_motor_id), frame, error)) {
      result.message = "mix tienkung encode modify id failed: " + error;
      return result;
    }
  } else {
    if (!encos_protocol_.encodeModifyId(hardware_joint->motor_id, new_motor_id, frame, error)) {
      result.message = "mix encos encode modify id failed: " + error;
      return result;
    }
  }
  if (!setAdminOutputFrame(*hardware_joint, frame, result)) {
    return result;
  }

  const auto safe_timeout = std::max(timeout_sec, 0.001);
  const int max_cycles = std::max(1, static_cast<int>(safe_timeout * 1000.0 / 2.0));
  for (int cycle = 0; cycle < max_cycles; ++cycle) {
    if (!transport_->exchangeProcessData()) {
      result.message = "mix CANFD exchange failed while modifying id";
      return result;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    CanfdFrame input_frame;
    if (!getAdminInputFrame(*hardware_joint, input_frame) || input_frame.length == 0) {
      continue;
    }
    if (hardware_joint->vendor == "tienkung") {
      TienkungAdminResult tienkung_result;
      if (tienkung_protocol_.decodeModifyId(*hardware_joint, input_frame, tienkung_result, error)) {
        result = toMixResult(*hardware_joint, tienkung_result);
        result.message = "mix tienkung modify id completed";
        return result;
      }
    } else {
      EncosAdminResult encos_result;
      if (encos_protocol_.decodeModifyId(
        static_cast<std::uint16_t>(hardware_joint->motor_id), new_motor_id, input_frame, encos_result, error))
      {
        result = toMixResult(*hardware_joint, encos_result);
        result.message = result.success ? "mix encos modify id completed" : encos_result.message;
        return result;
      }
    }
  }
  result.message = "mix modify id timed out";
  return result;
}

// 读取所有关节反馈。
//
// 每个关节从 transport 输入信箱读取反馈，再按 vendor 调用对应 protocol 解码。
void MixBackend::readJointFeedback()
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    auto & joint = joints_[index];
    const auto * hardware_joint = hardware_config_.getJoint(joint.name);
    if (hardware_joint == nullptr) {
      continue;
    }
    CanfdFrame frame;
    if (!transport_->getInputFrame(transportJointIndex(joint.name), frame)) {
      continue;
    }
    std::string error;
    if (hardware_joint->vendor == "encos") {
      EncosJointFeedback feedback;
      if (!encos_protocol_.decodeFeedback(*hardware_joint, frame, feedback, error)) {
        continue;
      }
      joint.position = feedback.position;
      joint.velocity = feedback.velocity;
      joint.effort = feedback.effort;
      joint.motor_error = feedback.motor_error;
      joint.motor_temp = static_cast<float>(feedback.motor_temp);
      joint.mos_temp = static_cast<float>(feedback.mos_temp);
    } else {
      TienkungJointFeedback feedback;
      if (!tienkung_protocol_.decodeFeedback(*hardware_joint, frame, feedback, error)) {
        continue;
      }
      joint.position = feedback.position;
      joint.velocity = feedback.velocity;
      joint.effort = feedback.effort;
      joint.motor_error = feedback.motor_error;
      joint.motor_temp = static_cast<float>(feedback.motor_temp);
      joint.mos_temp = static_cast<float>(feedback.mos_temp);
    }
    joint.enabled = true;
    joint.joint_error = joint.motor_error == 0 ?
      robot_interfaces::msg::JointState::JOINT_ERR_NONE :
      robot_interfaces::msg::JointState::JOINT_ERR_MOTOR_WARN;
    joint.error_message = joint.joint_error == robot_interfaces::msg::JointState::JOINT_ERR_NONE ?
      "" : "mix " + hardware_joint->vendor + " motor error code: " + std::to_string(joint.motor_error);
    if (joint.joint_error == robot_interfaces::msg::JointState::JOINT_ERR_MOTOR_WARN) {
      const int bus = hardware_config_.bus_type == "ethercat" ? hardware_joint->gateway.bus : hardware_joint->canfd.bus;
      RCLCPP_ERROR_THROTTLE(kLogger, kThrottleClock, 1000,
        "mix joint motor warning: vendor=%s joint=%s bus=%d motor_id=%d motor_error=%u message=\"%s\"",
        hardware_joint->vendor.c_str(),
        joint.name.c_str(),
        bus,
        hardware_joint->motor_id,
        static_cast<unsigned int>(joint.motor_error),
        joint.error_message.c_str());
    }
  }
}

}  // namespace robot_driver
