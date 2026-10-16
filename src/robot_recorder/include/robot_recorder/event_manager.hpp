#ifndef ROBOT_RECORDER__EVENT_MANAGER_HPP_
#define ROBOT_RECORDER__EVENT_MANAGER_HPP_

#include <cstdint>
#include <deque>
#include <mutex>
#include <set>
#include <string>

#include "robot_interfaces/msg/joint_states.hpp"
#include "robot_interfaces/msg/robot_state.hpp"

namespace robot_recorder
{

/// 事件会话：一次故障或告警触发对应的元数据。
///
/// recorder 只在事件边沿写入这份元数据；高频关节状态和命令细节仍由
/// *_joint_states.csv / *_commands.csv 持续记录。这里保存 sequence，方便按事件
/// 快速回查触发时刻附近的详细数据。
struct EventSession
{
  int id{0};                                 // 事件序号（自增）
  std::string event_type{"system_fault"};    // 事件类型：system_fault / joint_warning
  std::string severity{"error"};             // 事件级别：warning / error
  int64_t trigger_time_ms{0};                // 触发时间戳（ms）
  uint8_t trigger_state{0};                  // 触发时刻的系统状态
  std::string trigger_state_name;            // 触发时刻的状态名称
  std::string trigger_fault_reason;          // 触发时刻的故障原因
  bool trigger_fault_active{false};          // 触发时刻故障标志
  std::string joint_name;                    // 关节事件对应的关节名
  uint8_t joint_error{0};                    // 关节事件对应的 joint_error
  uint8_t motor_error{0};                    // 关节事件对应的原始 motor_error
  std::string error_message;                 // 关节事件对应的错误说明
  uint32_t joint_states_sequence{0};         // 触发时刻最近的 joint_states 序号
  uint32_t joint_commands_sequence{0};       // 触发时刻最近的 joint_commands 序号
};

/// 故障/告警事件管理器。
///
/// 负责两类低频事件的边沿检测和元数据采集：
/// 1. system_fault：state 从正常切到 fault。
/// 2. joint_warning：control/hardware_states 中关节首次出现电机告警。
///
/// EventManager 不直接写文件，只维护待写事件队列；RecorderNode 负责把事件写入
/// events.csv。这样事件检测逻辑可以独立单元测试。
class EventManager
{
public:
  /// cooldown_sec: 防重触冷却时间（秒）
  explicit EventManager(double cooldown_sec);

  /// 处理 state 消息，执行系统故障上升沿检测。
  ///
  /// joint_states_seq / joint_commands_seq 是 recorder 当前缓存的最新序号，
  /// 用于在 events.csv 中定位故障触发时刻附近的详细状态和命令。
  /// 返回 true 表示检测到从“正常→故障”的上升沿，新事件已创建。
  bool onRobotState(
    const robot_interfaces::msg::RobotState & msg,
    int64_t now_ms,
    uint32_t joint_states_seq,
    uint32_t joint_commands_seq,
    const std::string & driver_msg = "");

  /// 处理 control/hardware_states 消息，检测关节 warning 上升沿。
  ///
  /// 同一个 joint_name + joint_error + motor_error 组合持续存在时只触发一次；
  /// 告警清除后再次出现才会重新触发。返回 true 表示新建了 joint_warning 事件。
  bool onJointStates(
    const robot_interfaces::msg::JointStates & msg,
    int64_t now_ms);

  /// 是否存在待写事件。
  bool hasPendingEvent() const;

  /// 当前事件序号
  int currentEventId() const;

  /// 取出最早的待写事件。
  EventSession takePendingEvent();

  /// 重置事件管理器所有状态
  void reset();

  /// 前一次状态是否为故障
  bool previousFault() const;

private:
  /// 判断 RobotState 消息是否表示系统故障状态。
  static bool isFault(const robot_interfaces::msg::RobotState & msg);

  /// 判断单个关节状态是否属于需要记录事件的电机告警。
  static bool isJointWarning(const robot_interfaces::msg::JointState & msg);

  /// 生成关节告警去重 key；同一 key 持续存在时不会重复触发事件。
  static std::string warningKey(const robot_interfaces::msg::JointState & msg);

  mutable std::mutex mutex_;
  bool previous_fault_{false};               // 上一次系统故障标志（用于 fault 上升沿检测）
  std::set<std::string> previous_warning_keys_; // 上一帧处于 warning 的关节/错误组合
  int event_id_{0};                          // 事件序号计数器
  double cooldown_sec_;                      // 防重复触发冷却时间（秒）
  int64_t last_trigger_time_ms_{0};          // 上次触发时间（ms）
  std::string last_trigger_key_;             // 上次触发事件的去重 key
  std::deque<EventSession> pending_events_;  // 待写事件队列
};

}  // namespace robot_recorder
#endif  // ROBOT_RECORDER__EVENT_MANAGER_HPP_
