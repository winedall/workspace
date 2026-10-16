#ifndef ROBOT_STATE__ROBOT_STATE_NODE_HPP_
#define ROBOT_STATE__ROBOT_STATE_NODE_HPP_

#include <memory>
#include <set>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "robot_common/audio_client.hpp"
#include "robot_interfaces/msg/driver_status.hpp"
#include "robot_interfaces/msg/joint_states.hpp"
#include "robot_interfaces/msg/robot_intent.hpp"
#include "robot_interfaces/msg/rl_manager_status.hpp"
#include "robot_interfaces/msg/robot_state.hpp"
#include "robot_interfaces/srv/reset_fault.hpp"
#include "robot_interfaces/srv/set_robot_mode.hpp"
#include "robot_state/state_machine.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace robot_state
{

// ROS 2 状态机节点。
//
// 节点负责参数声明、订阅输入、提供服务和周期发布 state；具体状态转换逻辑
// 放在 StateMachine 中，避免 ROS 回调细节污染状态机核心逻辑。
class RobotStateNode : public rclcpp::Node
{
public:
  explicit RobotStateNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 输入回调：只更新时间戳和状态快照，真正状态推进在 publishState() 中周期执行。
  void handleJointStates(const robot_interfaces::msg::JointStates::SharedPtr msg);
  void handleDriverStatus(const robot_interfaces::msg::DriverStatus::SharedPtr msg);
  void handleRlManagerStatus(const robot_interfaces::msg::RlManagerStatus::SharedPtr msg);
  void handleActiveIntent(const robot_interfaces::msg::RobotIntent::SharedPtr msg);
  void handlePreparationProgress(const std_msgs::msg::Float32::SharedPtr msg);

  // 定时推进状态机并发布 RobotState。
  void publishState();

  // 检测状态变化并播报语音提示。
  void announceStateChange(const robot_interfaces::msg::RobotState & msg);
  void announceJointAlarms(const robot_interfaces::msg::JointStates & msg);
  // 发布一条提示音播报请求。
  void playTip(const std::string & key, uint8_t priority = 200, bool interrupt = false);
  // 异步请求 monitor 重启音频后端；不影响状态机 reset_fault 结果。
  void requestAudioRecover();
  // 根据关节状态生成提示音 key；返回空字符串表示无需播报。
  static std::string jointAlarmTip(const robot_interfaces::msg::JointState & msg);

  // 服务回调：对外提供模式切换和故障复位。
  void handleSetMode(
    const std::shared_ptr<robot_interfaces::srv::SetRobotMode::Request> request,
    std::shared_ptr<robot_interfaces::srv::SetRobotMode::Response> response);
  void handleResetFault(
    const std::shared_ptr<robot_interfaces::srv::ResetFault::Request> request,
    std::shared_ptr<robot_interfaces::srv::ResetFault::Response> response);

  // ROS 参数缓存。
  std::string robot_joints_config_;
  double state_publish_frequency_hz_{20.0};
  bool enable_mujoco_sim_{false};

  // 状态机核心与 ROS 通信对象。
  std::unique_ptr<StateMachine> state_machine_;
  rclcpp::Subscription<robot_interfaces::msg::JointStates>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<robot_interfaces::msg::DriverStatus>::SharedPtr driver_status_sub_;
  rclcpp::Subscription<robot_interfaces::msg::RlManagerStatus>::SharedPtr rl_manager_status_sub_;
  rclcpp::Subscription<robot_interfaces::msg::RobotIntent>::SharedPtr active_intent_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr preparation_progress_sub_;
  rclcpp::Publisher<robot_interfaces::msg::RobotState>::SharedPtr state_pub_;
  rclcpp::Service<robot_interfaces::srv::SetRobotMode>::SharedPtr set_mode_srv_;
  rclcpp::Service<robot_interfaces::srv::ResetFault>::SharedPtr reset_fault_srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr audio_recover_clt_;
  
  // 定时器，用于周期发布状态。
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // 音频客户端，用于播放提示音。
  std::unique_ptr<robot_common::AudioClient> audio_client_;

  // 状态变化跟踪，用于语音播报。
  uint8_t prev_state_{255};
  bool prev_fault_active_{false};
  std::string prev_fault_reason_;
  bool self_check_announced_{false};
  std::set<std::string> active_joint_alarm_keys_;
};

}  // namespace robot_state

#endif  // ROBOT_STATE__ROBOT_STATE_NODE_HPP_
