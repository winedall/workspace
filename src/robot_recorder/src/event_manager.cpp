#include "robot_recorder/event_manager.hpp"

#include <utility>
#include <string>

namespace robot_recorder
{

// 创建事件管理器，并设置同类事件的冷却时间。
EventManager::EventManager(double cooldown_sec)
: cooldown_sec_(cooldown_sec)
{
}

// 判断 RobotState 是否表示系统处于故障状态。
bool EventManager::isFault(const robot_interfaces::msg::RobotState & msg)
{
  return msg.fault_active ||
    msg.state == robot_interfaces::msg::RobotState::STATE_FAULT;
}

// 判断单个关节状态是否属于需要记录的关节告警。
bool EventManager::isJointWarning(const robot_interfaces::msg::JointState & msg)
{
  return msg.joint_error == robot_interfaces::msg::JointState::JOINT_ERR_MOTOR_WARN;
}

// 生成关节告警去重 key，用于识别同一关节的同一类持续告警。
std::string EventManager::warningKey(const robot_interfaces::msg::JointState & msg)
{
  return msg.joint_name + ":" + std::to_string(static_cast<int>(msg.joint_error)) +
    ":" + std::to_string(static_cast<int>(msg.motor_error));
}

// 处理系统状态消息，在正常到故障的上升沿创建 system_fault 事件。
bool EventManager::onRobotState(
  const robot_interfaces::msg::RobotState & msg,
  int64_t now_ms,
  uint32_t joint_states_seq,
  uint32_t joint_commands_seq,
  const std::string & driver_msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  bool current = isFault(msg);

  if (current && !previous_fault_) {
    const std::string trigger_key = "system_fault";
    if (event_id_ > 0) {
      int64_t dt = now_ms - last_trigger_time_ms_;
      if (trigger_key == last_trigger_key_ && dt < static_cast<int64_t>(cooldown_sec_ * 1000.0)) {
        previous_fault_ = current;
        return false;
      }
    }

    ++event_id_;
    EventSession session;
    session.id = event_id_;
    session.event_type = "system_fault";
    session.severity = "error";
    session.trigger_time_ms = now_ms;
    session.trigger_state = msg.state;
    session.trigger_state_name = msg.state_name;
    session.trigger_fault_reason = driver_msg.empty() ? msg.fault_reason : driver_msg;
    session.trigger_fault_active = msg.fault_active;
    session.joint_states_sequence = joint_states_seq;
    session.joint_commands_sequence = joint_commands_seq;

    pending_events_.push_back(std::move(session));
    previous_fault_ = current;
    last_trigger_time_ms_ = now_ms;
    last_trigger_key_ = trigger_key;
    return true;
  }

  previous_fault_ = current;
  return false;
}

// 处理关节状态消息，在关节 warning 首次出现时创建 joint_warning 事件。
bool EventManager::onJointStates(
  const robot_interfaces::msg::JointStates & msg,
  int64_t now_ms)
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::set<std::string> current_warning_keys;
  const robot_interfaces::msg::JointState * triggered_joint = nullptr;
  for (const auto & joint : msg.states) {
    if (!isJointWarning(joint)) {
      continue;
    }

    const auto key = warningKey(joint);
    current_warning_keys.insert(key);
    if (triggered_joint == nullptr && previous_warning_keys_.count(key) == 0) {
      triggered_joint = &joint;
    }
  }

  previous_warning_keys_ = std::move(current_warning_keys);

  if (triggered_joint == nullptr) {
    return false;
  }

  const std::string trigger_key = "joint_warning:" + warningKey(*triggered_joint);
  if (event_id_ > 0) {
    int64_t dt = now_ms - last_trigger_time_ms_;
    if (trigger_key == last_trigger_key_ && dt < static_cast<int64_t>(cooldown_sec_ * 1000.0)) {
      return false;
    }
  }

  ++event_id_;
  EventSession session;
  session.id = event_id_;
  session.event_type = "joint_warning";
  session.severity = "warning";
  session.trigger_time_ms = now_ms;
  session.joint_name = triggered_joint->joint_name;
  session.joint_error = triggered_joint->joint_error;
  session.motor_error = triggered_joint->motor_error;
  session.error_message = triggered_joint->error_message;
  session.joint_states_sequence = msg.sequence;
  session.joint_commands_sequence = msg.last_command_sequence;

  pending_events_.push_back(std::move(session));
  last_trigger_time_ms_ = now_ms;
  last_trigger_key_ = trigger_key;
  return true;
}

// 返回当前是否存在尚未被 recorder 写入的事件。
bool EventManager::hasPendingEvent() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return !pending_events_.empty();
}

// 返回当前事件序号。
int EventManager::currentEventId() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return event_id_;
}

// 取出最早的待写事件。
EventSession EventManager::takePendingEvent()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (pending_events_.empty()) {
    return EventSession{};
  }
  auto session = std::move(pending_events_.front());
  pending_events_.pop_front();
  return session;
}

// 清空事件管理器的故障、告警去重和当前会话状态。
void EventManager::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  previous_fault_ = false;
  previous_warning_keys_.clear();
  event_id_ = 0;
  last_trigger_time_ms_ = 0;
  last_trigger_key_.clear();
  pending_events_.clear();
}

// 返回上一帧系统状态是否处于故障。
bool EventManager::previousFault() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return previous_fault_;
}

}  // namespace robot_recorder
