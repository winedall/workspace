#include "robot_router/command_router.hpp"

#include <algorithm>
#include <string>
#include <unordered_set>

namespace robot_router
{

CommandRouter::CommandRouter(const RouterConfig & config)
: config_(config),
  joints_config_(robot_common::loadRobotJointsConfig(config.robot_joints_config))
{
  // 构造时给状态缓存一个保守默认值：系统仍在启动、驱动未知。
  // 在收到 state_manager 和 driver 的真实状态前，不应默认放行运动命令。
  system_state_.state = robot_interfaces::msg::RobotState::STATE_BOOTING;
  driver_status_.online = false;
  driver_status_.status = robot_interfaces::msg::DriverStatus::STATUS_UNKNOWN;
}

// 更新系统状态缓存。
void CommandRouter::updateSystemState(const robot_interfaces::msg::RobotState & state)
{
  system_state_ = state;
}

// 更新驱动状态缓存。
void CommandRouter::updateDriverStatus(const robot_interfaces::msg::DriverStatus & status)
{
  driver_status_ = status;
  have_driver_status_ = true;
}

// 评估意图，判断是否通过准入。
RouteDecision CommandRouter::evaluateIntent(
  const robot_interfaces::msg::RobotIntent & intent)
{
  if (isSystemBlockingMotion() && !isManualFaultDampIntent(intent) &&
    !isManualFaultResetIntent(intent))
  {
    return {false, "system state blocks intent"};
  }
  if (isSystemBlockingMotion() && isMotionIntent(intent)) {
    return {false, "system state blocks motion intent"};
  }

  // DO_READY 期间拒绝意图切换，允许阻尼（damp）打断准备动作。
  if (system_state_.state == robot_interfaces::msg::RobotState::STATE_DO_READY &&
    intent.intent_type == "model")
  {
    return {false, "model switch rejected during DO_READY"};
  }

  // 优先级只允许高优先级覆盖低优先级；数值越小优先级越高。
  // 门 D 前评估：app 模式下，即使遥控器刚占用了激活意图（例如按过菜单/阻尼），
  // 也不应凭残留的 teleop 优先级阻塞 APP 接管控制权。
  const int candidate_priority = intentPriority(intent);
  const int current_priority = have_active_intent_ ? intentPriority(active_intent_) : -1;
  if (have_active_intent_ &&
    !(app_mode_ && intent.source == "app") &&
    candidate_priority > current_priority)
  {
    return {false, "lower priority than active intent"};
  }

  // 门 D：app 模式下，teleop 仅允许安全与模式切换白名单。
  if (app_mode_ && intent.source == "teleop") {
    if (!isTeleopAllowedInAppMode(intent)) {
      return {false, "teleop rejected: app mode active"};
    }
  }

  // 通过准入后更新激活意图，并由节点层发布到 active_intent topic。
  active_intent_ = intent;
  have_active_intent_ = true;
  return {true, "intent accepted"};
}

// 评估关节命令，判断是否通过准入。
RouteDecision CommandRouter::evaluateJointCommands(
  const robot_interfaces::msg::JointCommands & commands)
{
  // 空命令帧没有可执行语义，直接拒绝，避免下游误认为已经产生安全停止命令。
  if (commands.commands.empty()) {
    return {false, "empty joint command frame"};
  }
  // FAULT 是全局安全阻断条件，只允许人工 damp 后的阻尼命令作为安全卸力通道。
  if (isSystemBlockingMotion() && !isFaultDampingCommand(commands)) {
    return {false, "system state blocks joint commands"};
  }
  // 默认要求驱动明确在线且无故障；未收到驱动状态时保持拒绝。
  if (!driverAllowsMotion()) {
    return {false, "driver unavailable"};
  }
  // sequence 属于命令源自己的序号域，按 source 分桶拒绝明显乱序或重复的帧。
  const auto sequence_key = commands.source;
  const auto last_sequence_it = last_accepted_sequence_by_source_.find(sequence_key);
  if (last_sequence_it != last_accepted_sequence_by_source_.end() &&
    commands.sequence <= last_sequence_it->second)
  {
    return {false, "joint command sequence is not newer"};
  }
  // 来源必须匹配当前激活控制源或当前模式允许的候选源。
  if (!sourceAllowedForJointCommands(commands)) {
    return {false, "joint command source not allowed"};
  }

  // 同一帧内逐个检查关节白名单和重复关节，防止非法名称进入控制层。
  std::unordered_set<std::string> names;
  for (const auto & command : commands.commands) {
    if (!joints_config_.hasJoint(command.joint_name)) {
      return {false, "unknown joint name"};
    }
    if (!names.insert(command.joint_name).second) {
      return {false, "duplicate joint name"};
    }
  }

  // 只有完全通过准入的帧才更新该来源的输入 sequence，避免被拒绝的旧帧污染时序状态。
  last_accepted_sequence_by_source_[sequence_key] = commands.sequence;
  return {true, "joint commands accepted"};
}

// 获取当前激活意图。
const robot_interfaces::msg::RobotIntent & CommandRouter::activeIntent() const
{
  return active_intent_;
}

// 构造 command_router accepted 输出帧。
robot_interfaces::msg::JointCommands CommandRouter::makeAcceptedJointCommands(
  const robot_interfaces::msg::JointCommands & commands)
{
  auto accepted = commands;
  accepted.parent_sequence = commands.sequence;
  accepted.parent_source = commands.source;
  accepted.sequence = ++accepted_output_sequence_;
  accepted.source = "command_router";
  accepted.stage = "accepted";
  return accepted;
}

// 设置控制模式。
void CommandRouter::setControlMode(ControlMode mode)
{
  control_mode_ = mode;
  if (control_mode_ != ControlMode::MIX) {
    cached_external_commands_.reset();
  }
}

// 设置 app 模式开关。
void CommandRouter::setAppMode(bool enabled)
{
  app_mode_ = enabled;
}

// 返回当前是否处于 app 模式。
bool CommandRouter::appMode() const
{
  return app_mode_;
}

// 清空当前激活意图，让后续意图（如 APP）能够接管控制权。
void CommandRouter::clearActiveIntent()
{
  active_intent_ = robot_interfaces::msg::RobotIntent();
  have_active_intent_ = false;
}

// 判断 teleop 意图在 app 模式下是否属于放行白名单。
bool CommandRouter::isTeleopAllowedInAppMode(
  const robot_interfaces::msg::RobotIntent & intent) const
{
  // config 类仅放行模式切换（mode 菜单：rl/mix/spec/mocap/app）；
  // getup/speed/nav 等配置交给 APP 控制，不放行。
  if (intent.intent_type == "config" && intent.command == "mode") {
    return true;
  }
  // status/damp：阻尼。
  if (intent.intent_type == "status" && intent.command == "damp") {
    return true;
  }
  // model/motion_stop：急停保护。
  if (intent.intent_type == "model" && intent.command == "motion_stop") {
    return true;
  }
  // reset、switch_model、finish_loop 等其他意图均不放行。
  return false;
}

// 缓存外部控制指令。
void CommandRouter::cacheExternalCommands(
  const robot_interfaces::msg::JointCommands & commands)
{
  if (control_mode_ != ControlMode::MIX) {
    return;
  }
  cached_external_commands_ = commands;
}

// 将 RL 指令与缓存的外部指令合并。
robot_interfaces::msg::JointCommands CommandRouter::merge(
  const robot_interfaces::msg::JointCommands & rl_cmds) const
{
  // 非 mix 模式或无外部指令时，直接返回 RL 指令。
  if (control_mode_ != ControlMode::MIX || !cached_external_commands_.has_value()) {
    return rl_cmds;
  }

  auto merged = rl_cmds;
  const auto & ext = cached_external_commands_.value();

  // 构建外部指令的 joint_name → JointCommand 查找表。
  std::unordered_map<std::string, const robot_interfaces::msg::JointCommand *> ext_map;
  for (const auto & cmd : ext.commands) {
    ext_map[cmd.joint_name] = &cmd;
  }

  // 按关节名覆盖，enable 取 AND 运算（RL 和外部都允许才使能）。
  for (auto & cmd : merged.commands) {
    auto it = ext_map.find(cmd.joint_name);
    if (it != ext_map.end()) {
      cmd.position = it->second->position;
      cmd.kp = it->second->kp;
      cmd.kd = it->second->kd;
      cmd.enable = cmd.enable && it->second->enable;
    }
  }

  // 合并后的指令来源仍标记为 rl_manager，以通过来源准入检查。
  return merged;
}

// 返回当前缓存的外部指令中涉及的关节名称集合。
std::unordered_set<std::string> CommandRouter::externalJointNames() const
{
  std::unordered_set<std::string> names;
  if (control_mode_ != ControlMode::MIX) {
    return names;
  }
  if (cached_external_commands_.has_value()) {
    for (const auto & cmd : cached_external_commands_.value().commands) {
      names.insert(cmd.joint_name);
    }
  }
  return names;
}

// 判断系统是否阻断运动意图。
bool CommandRouter::isSystemBlockingMotion() const
{
  return system_state_.state == robot_interfaces::msg::RobotState::STATE_FAULT ||
    system_state_.fault_active;
}

// 判断当前是否请求手动 damp 命令。
bool CommandRouter::isManualFaultDampIntent(const robot_interfaces::msg::RobotIntent & intent) const
{
  return intent.intent_type == "status" && intent.command == "damp";
}

// 判断当前是否请求故障复位。
bool CommandRouter::isManualFaultResetIntent(const robot_interfaces::msg::RobotIntent & intent) const
{
  return intent.intent_type == "reset" && intent.command == "reset_fault";
}

// 判断当前是否请求故障 damp 命令。
bool CommandRouter::isFaultDampingCommand(const robot_interfaces::msg::JointCommands & commands) const
{
  return isSystemBlockingMotion() &&
    have_active_intent_ &&
    isManualFaultDampIntent(active_intent_) &&
    commands.source == "rl_manager" &&
    commands.type == robot_interfaces::msg::JointCommands::TYPE_DAMPING;
}

// 判断驱动是否允许运动命令。
bool CommandRouter::driverAllowsMotion() const
{
  if (!have_driver_status_) {
    return false;
  }
  return driver_status_.online &&
    !driver_status_.fault_active &&
    (driver_status_.status == robot_interfaces::msg::DriverStatus::STATUS_ONLINE ||
    driver_status_.status == robot_interfaces::msg::DriverStatus::STATUS_DEGRADED);
}

// 判断意图是否为运动意图。
bool CommandRouter::isMotionIntent(const robot_interfaces::msg::RobotIntent & intent) const
{
  // 这里保持保守白名单，后续新增动作时可扩展命令名或直接依赖 intent_type=motion。
  return intent.intent_type == "motion";
}

// 判断关节命令来源是否允许。
bool CommandRouter::sourceAllowedForJointCommands(
  const robot_interfaces::msg::JointCommands & commands) const
{
  if (isFaultDampingCommand(commands)) {
    return true;
  }
  // 如果已有激活意图，优先要求关节命令来源与激活意图来源一致。
  if (have_active_intent_ && !active_intent_.source.empty() && commands.source == active_intent_.source) {
    return true;
  }
  // RL 模式、DO_READY/READY 准备阶段和 DAMP 阻尼阶段允许 rl_manager 作为候选执行源。
  if ((system_state_.state == robot_interfaces::msg::RobotState::STATE_DAMP ||
    system_state_.state == robot_interfaces::msg::RobotState::STATE_IDLE ||
    system_state_.state == robot_interfaces::msg::RobotState::STATE_DO_READY ||
    system_state_.state == robot_interfaces::msg::RobotState::STATE_READY ||
    system_state_.state == robot_interfaces::msg::RobotState::STATE_RL_STANDBY ||
    system_state_.state == robot_interfaces::msg::RobotState::STATE_RL_ACTIVE) &&
    commands.source == "rl_manager")
  {
    return true;
  }
  return false;
}

// 计算意图优先级。
int CommandRouter::intentPriority(const robot_interfaces::msg::RobotIntent & intent) const
{
  const int requested_priority = static_cast<int>(intent.priority);
  if (intent.source == "teleop") {
    return config_.teleop_priority;
  }
  if (intent.source == "app") {
    return std::clamp(
      requested_priority, config_.app_priority_min, config_.app_priority_max);
  }
  if (intent.source == "api" || intent.source == "http" || intent.source == "mqtt" ||
    intent.source == "cloud")
  {
    return std::clamp(
      requested_priority, config_.api_priority_min, config_.api_priority_max);
  }
  return config_.unknown_priority;
}

}  // namespace robot_router
