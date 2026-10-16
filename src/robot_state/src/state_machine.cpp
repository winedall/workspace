#include "robot_state/state_machine.hpp"

#include <algorithm>
#include <sstream>
#include <string>

namespace robot_state
{

namespace
{
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
}  // namespace

StateMachine::StateMachine(
  const std::string & robot_joints_config_path,
  bool sim_mode)
: robot_joints_config_path_(robot_joints_config_path), sim_mode_(sim_mode)
{
}

// 处理参数加载完成，更新状态机，节点从 BOOTING 进入 INIT。
void StateMachine::onParametersLoaded()
{
  if (state_ == SystemState::BOOTING) {
    setState(SystemState::INIT);
  }
}

// 加载机器人关节配置，更新状态机，节点从 INIT 进入 IDLE。
void StateMachine::loadConfiguration()
{
  joints_config_ = robot_common::loadRobotJointsConfig(robot_joints_config_path_);
  config_loaded_ = true;
  if (state_ == SystemState::INIT) {
    setState(SystemState::IDLE);
  }
}

// 处理关节状态消息，回调只更新快照，不直接做状态跳转，避免订阅回调里发生复杂状态竞争。
void StateMachine::onJointStates(
  const robot_interfaces::msg::JointStates & msg)
{
  last_joint_states_ = msg;
  have_joint_states_ = true;

  // 首次收到足够比例的有效非零数据前，不触发自检，避免 EtherCAT 首个周期尚未完成时
  // 读到全零或少量零散初始值导致误判为通讯异常。
  if (!data_settled_) {
    size_t enabled_joint_count = 0;
    size_t non_zero_joint_count = 0;
    for (const auto & state : msg.states) {
      const auto * cfg = joints_config_.getJoint(state.joint_name);
      if (cfg == nullptr || !cfg->enable_default) {
        continue;
      }
      ++enabled_joint_count;
      if (state.position != 0.0f) {
        ++non_zero_joint_count;
      }
    }
    const size_t required_non_zero_count = (enabled_joint_count + 2) / 3;
    data_settled_ = enabled_joint_count > 0 && non_zero_joint_count >= required_non_zero_count;
  } else if (settled_frame_count_ < kSettleFrameCount) {
    ++settled_frame_count_;
  }
}

// 处理驱动状态消息，回调只更新快照，不直接做状态跳转，避免订阅回调里发生复杂状态竞争。
void StateMachine::onDriverStatus(
  const robot_interfaces::msg::DriverStatus & msg)
{
  last_driver_status_ = msg;
  have_driver_status_ = true;
}

// 处理 RL 管理状态消息，回调只更新快照，不直接做状态跳转，避免订阅回调里发生复杂状态竞争。
void StateMachine::onRlManagerStatus(
  const robot_interfaces::msg::RlManagerStatus & msg)
{
  last_rl_manager_status_ = msg;
  have_rl_manager_status_ = true;
  active_rl_model_ = msg.active_model;
}

// 评估当前状态，根据输入更新状态机状态。
void StateMachine::evaluate()
{
  // BOOTING/INIT 不根据外部输入自动跳转，必须由明确流程推进。
  if (state_ == SystemState::BOOTING || state_ == SystemState::INIT) {
    return;
  }

  // 开机自检：IDLE 状态下先等待关节数据稳定和 RL 管理状态到达，再执行一次性检查。
  // 避免启动初期 RL status 尚未发布时，把“未收到状态”误判为 IMU 异常。
  if (state_ == SystemState::IDLE && self_check_state_ == SelfCheckState::PENDING &&
    have_joint_states_ && have_rl_manager_status_ && data_settled_ &&
    settled_frame_count_ >= kSettleFrameCount)
  {
    std::string fail_reason;
    if (runSelfCheck(fail_reason)) {
      self_check_state_ = SelfCheckState::PASSED;
    } else {
      self_check_state_ = SelfCheckState::FAILED;
      enterFault("self-check: " + fail_reason);
      return;
    }
  }

  // 驱动故障会使系统进入 FAULT，等待 reset_fault 服务确认恢复。
  if (last_driver_status_.fault_active ||
    last_driver_status_.status == robot_interfaces::msg::DriverStatus::STATUS_FAULT)
  {
    enterFault(
      "driver status fault: status=" + std::to_string(last_driver_status_.status) +
      ", fault_active=" + (last_driver_status_.fault_active ? "true" : "false"));
  }
  // 关节状态异常，进入 FAULT。
  if (!jointsHealthy()) {
    std::string detail;
    for (const auto & j : last_joint_states_.states) {
      if (j.joint_error != robot_interfaces::msg::JointState::JOINT_ERR_NONE &&
        j.joint_error != robot_interfaces::msg::JointState::JOINT_ERR_MOTOR_WARN)
      {
        if (!detail.empty()) detail += ", ";
        detail += j.joint_name + "(error=" + std::to_string(j.joint_error) + ")";
      }
    }
    enterFault("joint state error: " + detail);
  }

  if (state_ == SystemState::RL_ACTIVE) {
    // RL 管理状态推理错误，进入 FAULT。
    if (last_rl_manager_status_.inference_error) {
      enterFault("rl manager inference error: " + last_rl_manager_status_.error_message);
    }
    // RL 管理状态未满足 active 运行条件，按具体原因进入 FAULT。
    else if (!rlManagerReady()) {
      if (!last_rl_manager_status_.online || !last_rl_manager_status_.model_loaded ||
        !last_rl_manager_status_.model_ready)
      {
        enterFault(
          "rl manager not ready: online=" + std::to_string(last_rl_manager_status_.online) +
          ", model_loaded=" + std::to_string(last_rl_manager_status_.model_loaded) +
          ", model_ready=" + std::to_string(last_rl_manager_status_.model_ready));
      } else if (!imuInputReady()) {
        ++imu_fault_frame_count_;
        if (imu_fault_frame_count_ >= kImuFaultConfirmFrameCount) {
          enterFault(
            "imu data timeout: imu_ready=" + std::to_string(last_rl_manager_status_.imu_ready) +
            ", imu_age_sec=" + std::to_string(last_rl_manager_status_.imu_age_sec));
        }
      } else if (!last_rl_manager_status_.missing_inputs.empty()) {
        std::string missing;
        for (const auto & input : last_rl_manager_status_.missing_inputs) {
          if (!missing.empty()) missing += ", ";
          missing += input;
        }
        enterFault("rl manager inputs missing: " + missing);
      } else {
        enterFault(
          "rl manager not ready: online=" + std::to_string(last_rl_manager_status_.online) +
          ", model_loaded=" + std::to_string(last_rl_manager_status_.model_loaded) +
          ", model_ready=" + std::to_string(last_rl_manager_status_.model_ready));
      }
    } else {
      imu_fault_frame_count_ = 0;
    }
  } else {
    imu_fault_frame_count_ = 0;
  }

  // FAULT 是人工或明确服务恢复状态，不允许自动回到 READY。
  if (state_ == SystemState::FAULT) {
    return;
  }

  // DO_READY 期间等待 robot_control 汇报准备动作插值完成。
  if (state_ == SystemState::DO_READY && preparation_done_) {
    setState(SystemState::READY);
    return;
  }

  // DAMP 是受控阻尼输出状态，不自动切回 READY 或进入 RL_ACTIVE。
  if (state_ == SystemState::DAMP) {
    return;
  }

  // RL_STANDBY 只有在 RL 管理节点明确汇报模型 ready 后，才进入 RL_ACTIVE。
  if (state_ == SystemState::RL_STANDBY && readyConditionsMet() && rlManagerReady()) {
    setState(SystemState::RL_ACTIVE);
    return;
  }
}

// 处理意图意图。
bool StateMachine::handleIntent(
  const robot_interfaces::msg::RobotIntent & intent)
{
  // 故障期间拒绝处理意图，避免绕过安全状态。
  if (state_ == SystemState::FAULT) {
    if (intent.intent_type == "status" && intent.command == "damp") {
      return true;
    }
    return false;
  }

  // config/getup 只保存姿势配置到 rl_manager_node，不触发状态跳转。
  // 新起身配置会等机器人下次 damp → idle → do_ready 重新准备时生效，
  // 避免在站立姿态下直接切换目标导致摔倒。
  if (intent.intent_type == "config" && intent.command == "getup") {
    if (state_ == SystemState::READY || state_ == SystemState::DO_READY) {
      return true;
    }
    return false;
  }

  // config（非 getup）、motion、model 均为运行参数配置，不参与状态机切换。
  if (intent.intent_type == "config" || intent.intent_type == "motion" || intent.intent_type == "model")
  {
    return false;
  }

  // reset 类意图，清除故障。
  if (intent.intent_type == "reset") {
    if (intent.command == "reset_fault") {
      return resetFault("RobotIntent reset_fault");
    }
    return false;
  }

  // mode 类意图，切换控制模式。
  if (intent.intent_type == "mode") {
    if (intent.command != "switch_mode" ||
      (intent.action != "rl" && intent.action != "rule"))
    {
      return false;
    }
    return setMode(intent.action, "RobotIntent switch_mode");
  }

  // status 类意图，切换系统状态。
  if (intent.intent_type != "status") {
    return false;
  }

   // 切换阻尼状态。
  if (intent.command == "damp") {
    setState(SystemState::DAMP);
    return true;
  }
   // 切换零力矩状态。
  if (intent.command == "reinit" && state_ == SystemState::DAMP) {
    setState(SystemState::IDLE);
    return true;
  }
   // 切换准备状态。
  if (intent.command == "ready" && state_ == SystemState::IDLE && readyConditionsMet()) {
    preparation_done_ = false;
    setState(SystemState::DO_READY);
    return true;
  }
   // 切换行走状态。
  if (intent.command == "walk" && state_ == SystemState::READY && readyConditionsMet()) {
    setState(SystemState::RL_STANDBY);
    return true;
  }
  
  return false;
}

// 切换系统状态。
bool StateMachine::setMode(
    const std::string & target_mode,
    const std::string & reason)
{
  (void)reason;

  // 故障期间拒绝模式切换，避免绕过安全状态。
  if (state_ == SystemState::FAULT) {
    return false;
  }
  if (target_mode == "damp") {
    setState(SystemState::DAMP);
    return true;
  }
  if (target_mode == "idle" && state_ == SystemState::DAMP) {
    setState(SystemState::IDLE);
    return true;
  }
  if (target_mode == "ready" && state_ == SystemState::IDLE && readyConditionsMet()) {
    preparation_done_ = false;
    setState(SystemState::DO_READY);
    return true;
  }
  if (target_mode == "rl" && state_ == SystemState::READY) {
    setState(SystemState::RL_STANDBY);
    return true;
  }
  if (target_mode == "rl_active" && state_ == SystemState::RL_STANDBY && rlManagerReady()) {
    setState(SystemState::RL_ACTIVE);
    return true;
  }
  if (target_mode == "rule" && state_ == SystemState::READY) {
    setState(SystemState::RULE_STANDBY);
    return true;
  }
  if (target_mode == "rule_active" && state_ == SystemState::RULE_STANDBY) {
    setState(SystemState::RULE_ACTIVE);
    return true;
  }
  return false;
}

// 进入故障 状态。
void StateMachine::enterFault(const std::string & reason)
{
  fault_active_ = true;
  // 已在 FAULT 中时保留原始故障原因，避免后续超时等次要故障覆盖根因。
  if (state_ != SystemState::FAULT) {
    fault_reason_ = reason;
  }
  setState(SystemState::FAULT);
}

// 重置故障。
bool StateMachine::resetFault(const std::string & reason)
{
  (void)reason;
  // 故障复位必须在 FAULT 状态下执行，且当前输入已恢复健康。
  if (state_ != SystemState::FAULT) {
    return false;
  }
  if (!driverHealthy() || !jointsHealthy()) {
    return false;
  }
  fault_active_ = false;
  fault_reason_.clear();
  self_check_state_ = SelfCheckState::PENDING;
  setState(SystemState::IDLE);
  return true;
}

// 获取当前系统状态。
SystemState StateMachine::state() const
{
  return state_;
}

// 获取是否正在故障。
bool StateMachine::faultActive() const
{
  return fault_active_;
}

// 获取故障原因。
std::string StateMachine::faultReason() const
{
  return fault_reason_;
}

// 获取开机自检状态。
SelfCheckState StateMachine::selfCheckState() const
{
  return self_check_state_;
}
// 返回所有关节位置字符串，每行 "关节名 = rad (deg°)"。
std::string StateMachine::jointPositionsString() const
{
  std::ostringstream oss;
  oss.setf(std::ios::fixed, std::ios::floatfield);
  oss.precision(3);
  bool first = true;
  for (const auto & joint : last_joint_states_.states) {
    if (!first) {
      oss << "\n";
    }
    first = false;
    float deg = static_cast<float>(joint.position * kRadToDeg);
    oss << joint.joint_name << " = " << joint.position << " (" << deg << "°)";
  }
  return oss.str();
}

// 将当前系统状态转换为消息。
robot_interfaces::msg::RobotState StateMachine::toMessage() const
{
  // 该消息是系统状态对外唯一权威发布内容。
  robot_interfaces::msg::RobotState msg;
  msg.state = static_cast<std::uint8_t>(state_);
  msg.state_name = stateName(state_);
  msg.active_mode = active_mode_;
  msg.active_control_source = active_control_source_;
  msg.active_rl_model = active_rl_model_;
  msg.fault_active = fault_active_;
  msg.fault_reason = fault_reason_;
  return msg;
}

// 检查驱动是否健康。
bool StateMachine::driverHealthy() const
{
  // 驱动必须在线，且未主动报告故障。
  return have_driver_status_ &&
    last_driver_status_.online &&
    !last_driver_status_.fault_active &&
    last_driver_status_.status != robot_interfaces::msg::DriverStatus::STATUS_FAULT;
}

// 检查 RL 管理节点是否准备就绪。
bool StateMachine::rlManagerReady() const
{
  if (!have_rl_manager_status_) {
    return false;
  }
  return last_rl_manager_status_.online &&
    last_rl_manager_status_.model_loaded &&
    last_rl_manager_status_.model_ready &&
    !last_rl_manager_status_.inference_error &&
    last_rl_manager_status_.missing_inputs.empty() &&
    imuInputReady();
}

// 检查 imu 数据是否准备就绪
bool StateMachine::imuInputReady() const
{
  if (!have_rl_manager_status_) {
    return false;
  }
  if (!last_rl_manager_status_.imu_ready) {
    return false;
  }
  return std::find(
    last_rl_manager_status_.missing_inputs.begin(),
    last_rl_manager_status_.missing_inputs.end(),
    "imu") == last_rl_manager_status_.missing_inputs.end();
}

// 检查关节是否健康。
bool StateMachine::jointsHealthy() const
{
  if (!have_joint_states_) {
    // 尚未收到关节状态时不直接判 fault；是否可进入 READY 由 fresh 检查另行限制。
    return true;
  }
  for (const auto & joint : last_joint_states_.states) {
    // 温度告警先允许继续，由上层后续降额/限速策略处理；其他错误码进入 FAULT。
    if (joint.joint_error == robot_interfaces::msg::JointState::JOINT_ERR_NONE ||
      joint.joint_error == robot_interfaces::msg::JointState::JOINT_ERR_MOTOR_WARN)
    {
      continue;
    }
    return false;
  }
  return true;
}

// 执行开机自检：验证每个 enable_default=true 关节的位置读数是否有效。
//
// 自检包含两层检查：
//   1) position == 0.0f → 表示通讯异常（真实编码器读数即使零点也不会是精确零值）
//   2) position 超出 YAML 配置的 [min_pos, max_pos] 范围
//
// 失败时通过 fail_reason 输出具体原因；调用方负责在失败时进入 FAULT。
bool StateMachine::runSelfCheck(std::string & fail_reason) const
{
  if (!have_rl_manager_status_) {
    fail_reason = "rl manager status not received";
    return false;
  }

  if (!imuInputReady()) {
    fail_reason = "imu self-check failed: imu_ready=" +
      std::to_string(last_rl_manager_status_.imu_ready) +
      ", imu_age_sec=" + std::to_string(last_rl_manager_status_.imu_age_sec);
    return false;
  }

  for (const auto & joint : last_joint_states_.states) {
    const auto * config = joints_config_.getJoint(joint.joint_name);
    if (config == nullptr) {
      continue;
    }
    // 跳过默认不使能的关节（如 head），它们可能确实没有上电。
    if (!config->enable_default) {
      continue;
    }
    // 精确零值表示通讯异常：即使关节在零点位置，真实编码器读数也有小数值。
    // 仿真环境中头部等关节可能物理上处于零位，跳过此检查。
    if (!sim_mode_ && joint.position == 0.0f) {
      fail_reason = "joint '" + joint.joint_name + "' position is 0.0 (communication error)";
      return false;
    }
    // 位置超出配置的物理限位。
    if (joint.position < config->min_pos || joint.position > config->max_pos) {
      fail_reason = "joint '" + joint.joint_name + "' position " +
        std::to_string(joint.position) + " exceeds limits [" +
        std::to_string(config->min_pos) + ", " + std::to_string(config->max_pos) + "]";
      return false;
    }
  }
  return true;
}

// 检查是否满足进入 READY 状态的条件。
bool StateMachine::readyConditionsMet() const
{
  // READY 需要配置已加载、自检通过、无故障、驱动在线、关节状态健康。
  return config_loaded_ &&
    !fault_active_ &&
    self_check_state_ == SelfCheckState::PASSED &&
    driverHealthy() &&
    jointsHealthy();
}

// 设置状态机状态，不触发状态转换。
void StateMachine::setState(SystemState state)
{
  state_ = state;
}

// 获取状态机状态名称。
std::string StateMachine::stateName(SystemState state) const
{
  // 字符串使用小写下划线，便于 topic、日志和外部 API 统一处理。
  switch (state) {
    case SystemState::BOOTING:
      return "booting";
    case SystemState::INIT:
      return "init";
    case SystemState::IDLE:
      return "idle";
    case SystemState::DO_READY:
      return "do_ready";
    case SystemState::READY:
      return "ready";
    case SystemState::RL_STANDBY:
      return "rl_standby";
    case SystemState::RL_ACTIVE:
      return "rl_active";
    case SystemState::RULE_STANDBY:
      return "rule_standby";
    case SystemState::RULE_ACTIVE:
      return "rule_active";
    case SystemState::TRANSITIONING:
      return "transitioning";
    case SystemState::FAULT:
      return "fault";
    case SystemState::DAMP:
      return "damp";
  }
  return "unknown";
}

// 标记状态机准备完成。
void StateMachine::onPreparationDone()
{
  preparation_done_ = true;
}

}  // namespace robot_state
