#include "robot_driver/backend/encos_backend.hpp"

#include <algorithm>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <time.h>

#include "robot_driver/ethercat/soem_master.hpp"
#include "robot_driver/protocol/encos_protocol.hpp"

namespace robot_driver
{

namespace
{

static const auto kLogger = rclcpp::get_logger("encos_backend");
static rclcpp::Clock kThrottleClock(RCL_STEADY_TIME);

// 根据 Encos 电机错误码返回可读告警信息。
std::string motorWarningName(std::uint8_t motor_error)
{
  std::string name;
  switch (motor_error) {
    case 1:
      name = "over-temperature";
      break;
    case 2:
      name = "over-current";
      break;
    case 3:
      name = "over-voltage";
      break;
    case 4:
      name = "under-voltage";
      break;
    default:
      name = "unknown";
      break;
  }
  return "encos motor warning: " + name + " (code=" + std::to_string(static_cast<int>(motor_error)) + ")";
}

}  // namespace

EncosBackend::EncosBackend(const BackendConfig & config)
: EncosBackend(config, std::make_unique<SoemMaster>())
{
}

EncosBackend::EncosBackend(
  const BackendConfig & config,
  std::unique_ptr<EthercatMaster> ethercat_master)
: HardwareBackend(config, "encos"),
  ethercat_master_(std::move(ethercat_master)),
  protocol_(std::make_unique<EncosProtocol>())
{
  // 先做静态配置校验，再初始化 EtherCAT，避免真实链路打开后才暴露 YAML 错误。
  if (!validateConfig()) {
    return;
  }

  // 初始化关节运行时反馈状态
  initializeJoints();

  // 从硬件配置读取 EtherCAT 周期（ms），默认 2ms。
  setCycleTimeMs(hardware_config_.ethercat.cycle_time_ms);

  // 初始化 EtherCAT
  initEthercat();

  // 初始化后，根据当前状态打印日志
  if (ethercat_configured_ && !fault_active_.load()) {
    RCLCPP_INFO(kLogger, "%s", config_mode_ ? "encos backend is motor config" : "encos backend is motor motion");
  } else {
    RCLCPP_INFO(kLogger, "%s", "encos backend is fault");
  }
}

EncosBackend::~EncosBackend()
{
  // 析构时必须先停止 worker 线程，再释放 SOEM 资源。
  stopWorker();

  // 关闭 EtherCAT 主通道
  if (ethercat_master_) {
    ethercat_master_->close();
  }
}

// 校验 Encos 硬件配置是否合法。
bool EncosBackend::validateConfig()
{
  if (!hardware_config_.vendor_type.empty() && hardware_config_.vendor_type != "encos") {
    setFault("encos backend requires hardware.vendor_type to be 'encos'");
    return false;
  }
  if (hardware_config_.bus_type != "ethercat") {
    setFault("encos backend requires hardware.bus_type to be 'ethercat'");
    return false;
  }
  for (const auto & hardware_joint : hardware_config_.joints) {
    if (!joints_config_.hasJoint(hardware_joint.name)) {
      setFault("encos hardware joint '" + hardware_joint.name + "' does not exist in robot_joints.yaml");
      return false;
    }
    if (hardware_joint.encos.slave_id < 0) {
      setFault("encos joint '" + hardware_joint.name + "' slave_id must be >= 0");
      return false;
    }
    if (hardware_joint.encos.passage <= 0) {
      setFault("encos joint '" + hardware_joint.name + "' passage must be > 0");
      return false;
    }
    if (hardware_joint.encos.kt <= 0.0) {
      setFault("encos joint '" + hardware_joint.name + "' kt must be > 0");
      return false;
    }
  }
  return true;
}

/*****************************************************************************/
// 初始化 EtherCAT。
void EncosBackend::initEthercat()
{
  // 验证网络接口配置
  if (hardware_config_.ethercat.network_interface.empty()) {
    setFault("encos EtherCAT requires hardware.ethercat.network_interface");
    ethercat_configured_ = false;
    return;
  }

  // 验证 EtherCAT 主站是否可用
  if (!ethercat_master_) {
    setFault("encos EtherCAT requested but master is not available");
    ethercat_configured_ = false;
    return;
  }

  // 打开 EtherCAT 主站
  if (!ethercat_master_->open(hardware_config_.ethercat.network_interface)) {
    ethercat_configured_ = false;
    setFault("EtherCAT open failed");
    return;
  }

  // 配置 EtherCAT 主站
  if (!ethercat_master_->configure(hardware_config_.ethercat.slave_count)) {
    ethercat_configured_ = false;
    setFault("EtherCAT configure failed");
    return;
  }

  // 初始化成功
  fault_active_ = false;
  ethercat_configured_ = true;
  status_message_ = "encos motion write ready";
}

// 判断 Encos EtherCAT 链路是否已经可用于统一控制周期。
bool EncosBackend::transportReady() const
{
  return ethercat_configured_ && ethercat_master_ && protocol_;
}

// 执行一次 SOEM process data 交换。
//
// 该函数是 HardwareBackend::runCycle() 的通讯钩子，失败时要求 Encos 进入可重连故障。
bool EncosBackend::exchangeProcessData()
{
  if (!ethercat_master_->exchangeProcessData()) {
    setFault("EtherCAT process data exchange failed during send command");
    return false;
  }
  setStatusMessage("encos EtherCAT online: motion write enabled");
  return true;
}

// 返回 Encos EtherCAT 是否已经完成 open/configure。
bool EncosBackend::configured() const
{
  return ethercat_configured_;
}

// 从输入缓冲区读取所有关节反馈，并刷新统一关节状态。
void EncosBackend::readJointFeedback()
{
  if (!ethercat_master_ || !protocol_) {
    return;
  }

  // 加锁保护反馈缓存
  std::lock_guard<std::mutex> lock(feedback_mutex_);

  // 刷新所有关节的反馈
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    auto & joint = joints_[index];
    const auto * hardware_joint = hardware_config_.getJoint(joint.name);
    if (hardware_joint == nullptr) {
      continue;
    }

    const auto * input_buffer = ethercat_master_->inputBuffer(hardware_joint->encos.slave_id);
    const auto input_size = ethercat_master_->inputBufferSize(hardware_joint->encos.slave_id);
    if (input_buffer == nullptr || input_size == 0) {
      continue;
    }

    EncosJointFeedback feedback;
    std::string error;
    CanfdFrame frame;
    if (!gateway_.readFrame(hardware_joint->encos.passage, input_buffer, input_size, frame, error)) {
      continue;
    }
    if (!protocol_->decodeFeedback(*hardware_joint, frame, feedback, error)) {
      continue;
    }

    // 更新关节状态
    joint.position = feedback.position;
    joint.velocity = feedback.velocity;
    joint.effort = feedback.effort;
    joint.motor_temp = static_cast<float>(feedback.motor_temp);
    joint.mos_temp = static_cast<float>(feedback.mos_temp);
    joint.enabled = true;
    joint.motor_error = feedback.motor_error;
    if (feedback.motor_error != 0) {
      joint.motor_fault_count = std::min(
        joint.motor_fault_count + 1,
        std::max(1, backend_config_.motor_fault_confirm_frames));
    } else {
      joint.motor_fault_count = 0;
    }

    if (feedback.motor_error == 0 ||
      joint.motor_fault_count < std::max(1, backend_config_.motor_fault_confirm_frames))
    {
      joint.joint_error = robot_interfaces::msg::JointState::JOINT_ERR_NONE;
    } else {
      joint.joint_error = robot_interfaces::msg::JointState::JOINT_ERR_MOTOR_WARN;
    }
    if (joint.joint_error == robot_interfaces::msg::JointState::JOINT_ERR_NONE) {
      joint.error_message.clear();
    } else if (joint.joint_error == robot_interfaces::msg::JointState::JOINT_ERR_MOTOR_WARN) {
      joint.error_message = motorWarningName(feedback.motor_error);
      RCLCPP_ERROR_THROTTLE(kLogger, kThrottleClock, 1000,
        "encos joint motor warning: joint=%s motor_id=%d slave_id=%d passage=%d motor_error=%u message=\"%s\"",
        joint.name.c_str(),
        hardware_joint->motor_id,
        hardware_joint->encos.slave_id,
        hardware_joint->encos.passage,
        static_cast<unsigned int>(feedback.motor_error),
        joint.error_message.c_str());
    } else {
      joint.error_message = "encos motor error code: " + std::to_string(static_cast<int>(feedback.motor_error));
    }

    // 高层校验：仅在硬件无故障（或仅为电机告警）时才覆盖，避免掩盖真实硬件错误
    if (joint.joint_error == robot_interfaces::msg::JointState::JOINT_ERR_NONE ||
        joint.joint_error == robot_interfaces::msg::JointState::JOINT_ERR_MOTOR_WARN)
    {
      const auto * joint_config = joints_config_.getJoint(joint.name);

      // INVALID_ANGLE_STATE: 反馈位置超过 YAML 定义的限位
      if (joint_config != nullptr &&
          (joint.position < joint_config->min_pos || joint.position > joint_config->max_pos))
      {
        ++joint.invalid_state_count;
        if (joint.invalid_state_count >= 3) {
          joint.joint_error = robot_interfaces::msg::JointState::JOINT_ERR_INVALID_ANGLE_STATE;
          joint.error_message = "feedback position " + std::to_string(joint.position) +
            " exceeds limits [" + std::to_string(joint_config->min_pos) +
            ", " + std::to_string(joint_config->max_pos) + "]";
        }
      } else {
        joint.invalid_state_count = 0;
      }

      // ZERO_ABSENCE: 反馈位置为绝对零值，表明通讯异常
      if (joint.position == 0.0) {
        ++joint.zero_absence_count;
        if (joint.zero_absence_count >= 3) {
          if (joint.joint_error == robot_interfaces::msg::JointState::JOINT_ERR_NONE) {
            joint.joint_error = robot_interfaces::msg::JointState::JOINT_ERR_ZERO_ABSENCE;
            joint.error_message = "position is absolute zero, possible communication loss";
          }
        }
      } else {
        joint.zero_absence_count = 0;
      }
    }
  }
}

// 编码关节命令。
bool EncosBackend::encodeJointCommand(const robot_interfaces::msg::JointCommand & command)
{
  const auto * hardware_joint = hardware_config_.getJoint(command.joint_name);
  if (hardware_joint == nullptr) {
    setFault("encos hardware mapping missing for joint: " + command.joint_name);
    return false;
  }

  auto * output_buffer = ethercat_master_->outputBuffer(hardware_joint->encos.slave_id);
  const auto output_size = ethercat_master_->outputBufferSize(hardware_joint->encos.slave_id);
  if (output_buffer == nullptr || output_size == 0) {
    setFault("EtherCAT output buffer unavailable for joint: " + command.joint_name);
    return false;
  }

  std::string error;
  CanfdFrame frame;
  if (!protocol_->encodeMotion(*hardware_joint, command, frame, error)) {
    setFault(error);
    return false;
  }
  if (!gateway_.writeFrame(hardware_joint->encos.passage, frame, output_buffer, output_size, error)) {
    setFault(error);
    return false;
  }
  return true;
}

/*****************************************************************************/
// 按关节名获取电机 ID。
EncosAdminResult EncosBackend::getMotorId(const std::string & joint_name, double timeout_sec)
{
  std::lock_guard<std::mutex> admin_lock(admin_mutex_);
  EncosAdminResult failure;
  failure.operation = EncosAdminOperation::GetMotorId;
  if (!config_mode_) {
    failure.message = "admin services only available in config_mode";
    return failure;
  }
  if (!ethercat_configured_ || !ethercat_master_ || !protocol_ || fault_active_.load()) {
    failure.message = "encos backend not ready";
    return failure;
  }

  const auto * hardware_joint = hardware_config_.getJoint(joint_name);
  if (hardware_joint == nullptr) {
    failure.success = false;
    failure.message = "encos hardware mapping missing for joint: " + joint_name;
    return failure;
  }

  const auto slave_id = hardware_joint->encos.slave_id;
  const auto passage = hardware_joint->encos.passage;
  auto * output_buffer = ethercat_master_->outputBuffer(slave_id);
  const auto output_size = ethercat_master_->outputBufferSize(slave_id);
  if (output_buffer == nullptr || output_size == 0) {
    failure.message = "output buffer unavailable";
    return failure;
  }

  std::string error;
  CanfdFrame request;
  if (!protocol_->encodeQueryId(request, error) ||
    !gateway_.writeFrame(passage, request, output_buffer, output_size, error))
  {
    failure.message = "encos get motor id request failed: " + error;
    return failure;
  }

  const auto safe_timeout = std::max(timeout_sec, 0.001);
  const int max_cycles = std::max(1, static_cast<int>(safe_timeout * 1000.0 / 2.0));
  for (int cycle = 0; cycle < max_cycles; ++cycle) {
    if (!ethercat_master_->exchangeProcessData()) {
      failure.message = "EtherCAT exchange failed while getting motor id";
      return failure;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const auto * input_buffer = ethercat_master_->inputBuffer(slave_id);
    const auto input_size = ethercat_master_->inputBufferSize(slave_id);
    if (input_buffer == nullptr || input_size == 0) {
      continue;
    }
    CanfdFrame response;
    EncosAdminResult result;
    if (gateway_.readFrame(passage, input_buffer, input_size, response, error) &&
      protocol_->decodeQueryId(response, result, error))
    {
      return result;
    }
  }
  failure.message = "timeout";
  return failure;
}

// 按关节名设置电机 ID。
EncosAdminResult EncosBackend::setMotorId(
  const std::string & joint_name,
  std::uint16_t new_motor_id,
  double timeout_sec)
{
  std::lock_guard<std::mutex> admin_lock(admin_mutex_);
  EncosAdminResult failure;
  failure.operation = EncosAdminOperation::SetMotorId;
  failure.new_motor_id = new_motor_id;
  if (!config_mode_) {
    failure.message = "admin services only available in config_mode";
    return failure;
  }
  if (!ethercat_configured_ || !ethercat_master_ || !protocol_ || fault_active_.load()) {
    failure.message = "encos backend not ready";
    return failure;
  }

  const auto * hardware_joint = hardware_config_.getJoint(joint_name);
  if (hardware_joint == nullptr) {
    failure.success = false;
    failure.message = "encos hardware mapping missing for joint: " + joint_name;
    return failure;
  }
  const auto slave_id = hardware_joint->encos.slave_id;
  const auto passage = hardware_joint->encos.passage;
  const auto motor_id = hardware_joint->motor_id;
  failure.motor_id = motor_id;

  auto * output_buffer = ethercat_master_->outputBuffer(slave_id);
  const auto output_size = ethercat_master_->outputBufferSize(slave_id);
  if (output_buffer == nullptr || output_size == 0) {
    failure.message = "output buffer unavailable";
    return failure;
  }

  std::string error;
  CanfdFrame request;
  if (!protocol_->encodeModifyId(motor_id, new_motor_id, request, error) ||
    !gateway_.writeFrame(passage, request, output_buffer, output_size, error))
  {
    failure.message = "encos set motor id request failed: " + error;
    return failure;
  }

  const auto safe_timeout = std::max(timeout_sec, 0.001);
  const int max_cycles = std::max(1, static_cast<int>(safe_timeout * 1000.0 / 2.0));
  for (int cycle = 0; cycle < max_cycles; ++cycle) {
    if (!ethercat_master_->exchangeProcessData()) {
      failure.message = "EtherCAT exchange failed while setting motor id";
      return failure;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const auto * input_buffer = ethercat_master_->inputBuffer(slave_id);
    const auto input_size = ethercat_master_->inputBufferSize(slave_id);
    if (input_buffer == nullptr || input_size == 0) {
      continue;
    }
    CanfdFrame response;
    EncosAdminResult result;
    if (gateway_.readFrame(passage, input_buffer, input_size, response, error) &&
      protocol_->decodeModifyId(motor_id, new_motor_id, response, result, error))
    {
      return result;
    }
  }
  failure.message = "timeout";
  return failure;
}

// 按关节名设置电机零点。
EncosAdminResult EncosBackend::setMotorZero(const std::string & joint_name, double timeout_sec)
{
  std::lock_guard<std::mutex> admin_lock(admin_mutex_);
  EncosAdminResult failure;
  failure.operation = EncosAdminOperation::SetMotorZero;
  if (!config_mode_) {
    failure.message = "admin services only available in config_mode";
    return failure;
  }
  if (!ethercat_configured_ || !ethercat_master_ || !protocol_ || fault_active_.load()) {
    failure.message = "encos backend not ready";
    return failure;
  }

  const auto * hardware_joint = hardware_config_.getJoint(joint_name);
  if (hardware_joint == nullptr) {
    failure.success = false;
    failure.message = "encos hardware mapping missing for joint: " + joint_name;
    return failure;
  }
  const auto slave_id = hardware_joint->encos.slave_id;
  const auto passage = hardware_joint->encos.passage;
  const auto motor_id = hardware_joint->motor_id;
  failure.motor_id = motor_id;

  auto * output_buffer = ethercat_master_->outputBuffer(slave_id);
  const auto output_size = ethercat_master_->outputBufferSize(slave_id);
  if (output_buffer == nullptr || output_size == 0) {
    failure.message = "output buffer unavailable";
    return failure;
  }

  std::string error;
  CanfdFrame request;
  if (!protocol_->encodeSetZero(motor_id, request, error) ||
    !gateway_.writeFrame(passage, request, output_buffer, output_size, error))
  {
    failure.message = "encos set motor zero request failed: " + error;
    return failure;
  }

  const auto safe_timeout = std::max(timeout_sec, 0.001);
  const int max_cycles = std::max(1, static_cast<int>(safe_timeout * 1000.0 / 2.0));
  for (int cycle = 0; cycle < max_cycles; ++cycle) {
    if (!ethercat_master_->exchangeProcessData()) {
      failure.message = "EtherCAT exchange failed while setting motor zero";
      return failure;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const auto * input_buffer = ethercat_master_->inputBuffer(slave_id);
    const auto input_size = ethercat_master_->inputBufferSize(slave_id);
    if (input_buffer == nullptr || input_size == 0) {
      continue;
    }
    CanfdFrame response;
    EncosAdminResult result;
    if (gateway_.readFrame(passage, input_buffer, input_size, response, error) &&
      protocol_->decodeSetZero(motor_id, response, result, error))
    {
      return result;
    }
  }
  failure.message = "timeout";
  return failure;
}

// 按关节名获取电机参数。
EncosAdminResult EncosBackend::getMotorParam(
  const std::string & joint_name,
  std::uint8_t param_code,
  double timeout_sec)
{
  std::lock_guard<std::mutex> admin_lock(admin_mutex_);
  EncosAdminResult failure;
  failure.operation = EncosAdminOperation::GetMotorParam;
  failure.param_code = param_code;
  if (!config_mode_) {
    failure.message = "admin services only available in config_mode";
    return failure;
  }
  if (!ethercat_configured_ || !ethercat_master_ || !protocol_ || fault_active_.load()) {
    failure.message = "encos backend not ready";
    return failure;
  }

  const auto * hardware_joint = hardware_config_.getJoint(joint_name);
  if (hardware_joint == nullptr) {
    failure.success = false;
    failure.message = "encos hardware mapping missing for joint: " + joint_name;
    return failure;
  }
  const auto slave_id = hardware_joint->encos.slave_id;
  const auto passage = hardware_joint->encos.passage;
  const auto motor_id = hardware_joint->motor_id;
  failure.motor_id = motor_id;

  auto * output_buffer = ethercat_master_->outputBuffer(slave_id);
  const auto output_size = ethercat_master_->outputBufferSize(slave_id);
  if (output_buffer == nullptr || output_size == 0) {
    failure.message = "output buffer unavailable";
    return failure;
  }

  std::string error;
  CanfdFrame request;
  if (!protocol_->encodeQueryParam(motor_id, param_code, request, error) ||
    !gateway_.writeFrame(passage, request, output_buffer, output_size, error))
  {
    failure.message = "encos get motor parameter request failed: " + error;
    return failure;
  }

  const auto safe_timeout = std::max(timeout_sec, 0.001);
  const int max_cycles = std::max(1, static_cast<int>(safe_timeout * 1000.0 / 2.0));
  for (int cycle = 0; cycle < max_cycles; ++cycle) {
    if (!ethercat_master_->exchangeProcessData()) {
      failure.message = "EtherCAT exchange failed while getting motor parameter";
      return failure;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const auto * input_buffer = ethercat_master_->inputBuffer(slave_id);
    const auto input_size = ethercat_master_->inputBufferSize(slave_id);
    if (input_buffer == nullptr || input_size == 0) {
      continue;
    }
    CanfdFrame response;
    EncosAdminResult result;
    if (gateway_.readFrame(passage, input_buffer, input_size, response, error) &&
      protocol_->decodeQueryParam(motor_id, param_code, response, result, error))
    {
      return result;
    }
  }
  failure.message = "timeout";
  return failure;
}

}  // namespace robot_driver
