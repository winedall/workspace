#include "robot_state/robot_state_node.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <memory>
#include <set>
#include <string>

#include "robot_common/audio_tips.hpp"
#include "rclcpp/qos.hpp"

namespace robot_state
{

RobotStateNode::RobotStateNode(const rclcpp::NodeOptions & options)
: Node("robot_state_node", options)
{
  // 声明参数，支持 launch/YAML 覆盖。
  robot_joints_config_ = declare_parameter<std::string>("robot_joints_config", "");
  state_publish_frequency_hz_ = declare_parameter<double>("state_publish_frequency_hz", 20.0);
  enable_mujoco_sim_ = declare_parameter<bool>("enable_mujoco_sim", false);

  // 状态机初始化时，会自动进入 BOOTING 状态，等待配置加载完成。
  state_machine_ = std::make_unique<StateMachine>(robot_joints_config_, enable_mujoco_sim_);
  // 显式推进 BOOTING -> INIT -> IDLE。若配置加载失败，节点启动会失败。
  state_machine_->onParametersLoaded();
  state_machine_->loadConfiguration();

  // 提示音播报客户端，供状态切换时发出提示。
  audio_client_ = std::make_unique<robot_common::AudioClient>(*this);

  // 订阅关节状态，用于检测关节故障。
  joint_states_sub_ = create_subscription<robot_interfaces::msg::JointStates>(
    "control/hardware_states",
    rclcpp::SensorDataQoS(),
    std::bind(&RobotStateNode::handleJointStates, this, std::placeholders::_1));

  // 订阅驱动状态，用于检测驱动故障。
  driver_status_sub_ = create_subscription<robot_interfaces::msg::DriverStatus>(
    "driver/status",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&RobotStateNode::handleDriverStatus, this, std::placeholders::_1));

  // 订阅RL管理器状态，用于检测RL管理器故障。
  rl_manager_status_sub_ = create_subscription<robot_interfaces::msg::RlManagerStatus>(
    "rl_manager/status",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&RobotStateNode::handleRlManagerStatus, this, std::placeholders::_1));

  // 订阅当前意图，用于检测意图故障。
  active_intent_sub_ = create_subscription<robot_interfaces::msg::RobotIntent>(
    "router/active_intent",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&RobotStateNode::handleActiveIntent, this, std::placeholders::_1));

  // 订阅准备进度，用于检测准备进度故障。
  preparation_progress_sub_ = create_subscription<std_msgs::msg::Float32>(
    "control/preparation_progress",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&RobotStateNode::handlePreparationProgress, this, std::placeholders::_1));
    
  // 发布系统状态，用于订阅者获取当前状态。
  state_pub_ = create_publisher<robot_interfaces::msg::RobotState>(
    "state", rclcpp::SystemDefaultsQoS());

  // 状态机控制服务。服务只改变系统状态，不直接控制硬件。
  set_mode_srv_ = create_service<robot_interfaces::srv::SetRobotMode>(
    "state/set_mode",
    std::bind(&RobotStateNode::handleSetMode, this, std::placeholders::_1, std::placeholders::_2));
      
  // 重置故障服务，用于重置系统状态。
  reset_fault_srv_ = create_service<robot_interfaces::srv::ResetFault>(
    "state/reset_fault",
    std::bind(&RobotStateNode::handleResetFault, this, std::placeholders::_1, std::placeholders::_2));

  // 提示音恢复服务，用于请求提示音恢复。
  audio_recover_clt_ = create_client<std_srvs::srv::Trigger>("monitor/audio_recover");
      
  // 定时器，用于定期发布系统状态。
  const auto period = std::chrono::duration<double>(1.0 / std::max(state_publish_frequency_hz_, 1.0));
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&RobotStateNode::publishState, this));
}

// 处理关节状态消息，更新状态机。
void RobotStateNode::handleJointStates(
  const robot_interfaces::msg::JointStates::SharedPtr msg)
{
  state_machine_->onJointStates(*msg);
  announceJointAlarms(*msg);
}

// 处理驱动状态消息，更新状态机。
void RobotStateNode::handleDriverStatus(
  const robot_interfaces::msg::DriverStatus::SharedPtr msg)
{
  state_machine_->onDriverStatus(*msg);
}

// 处理RL管理器状态消息，更新状态机。
void RobotStateNode::handleRlManagerStatus(
  const robot_interfaces::msg::RlManagerStatus::SharedPtr msg)
{
  state_machine_->onRlManagerStatus(*msg);
}

// 处理当前意图消息，更新状态机。
void RobotStateNode::handleActiveIntent(
  const robot_interfaces::msg::RobotIntent::SharedPtr msg)
{
  if (msg->intent_type == "reset" && msg->command == "reset_fault") {
    requestAudioRecover();
  }

  // 只消费 command_router 已准入的 active_intent。config/motion/model 等非状态机意图
  // 会由 StateMachine 忽略，避免遥控菜单配置影响系统状态。
  const auto accepted = state_machine_->handleIntent(*msg);
  
  // motion 类型是行走模式下持续刷新的速度命令，不重复打印。
  if (msg->intent_type != "motion") {
    RCLCPP_INFO(
      get_logger(),
      "Intent received source=%s type=%s command=%s action=%s accepted=%d",
      msg->source.c_str(),
      msg->intent_type.c_str(),
      msg->command.c_str(),
      msg->action.c_str(),
      accepted);
  }
}

// 发布系统状态，更新状态机并发布最新状态。
void RobotStateNode::publishState()
{
  const auto stamp = now();
  state_machine_->evaluate();
  auto msg = state_machine_->toMessage();
  msg.header.stamp = stamp;
  msg.header.frame_id = "base_link";
  state_pub_->publish(msg);
  announceStateChange(msg);
}

// 发布一条提示音播报请求。
void RobotStateNode::playTip(const std::string & key, uint8_t priority, bool interrupt)
{
  if (audio_client_) {
    audio_client_->playTip(key, priority, interrupt);
  }
}

// 生成关节告警提示音 key，用于播报语音提示。
std::string RobotStateNode::jointAlarmTip(const robot_interfaces::msg::JointState & msg)
{
  if (msg.joint_error == robot_interfaces::msg::JointState::JOINT_ERR_NONE) {
    return "";
  }

  if (msg.joint_error == robot_interfaces::msg::JointState::JOINT_ERR_MOTOR_WARN) {
    std::string reason;
    switch (msg.motor_error) {
      case 1:
        reason = robot_common::audio_tips::MOTOR_OVER_TEMP_WARN;
        break;
      case 2:
        reason = robot_common::audio_tips::MOTOR_OVER_CURRENT_WARN;
        break;
      case 3:
        reason = robot_common::audio_tips::MOTOR_OVER_VOLTAGE_WARN;
        break;
      case 4:
        reason = robot_common::audio_tips::MOTOR_UNDER_VOLTAGE_WARN;
        break;
      default:
        reason = robot_common::audio_tips::MOTOR_WARN;
        break;
    }
    return reason;
  }

  return robot_common::audio_tips::JOINT_STATE_ERROR;
}

// 检测关节告警并播报语音提示。
void RobotStateNode::announceJointAlarms(const robot_interfaces::msg::JointStates & msg)
{
  std::set<std::string> current_keys;
  for (const auto & joint : msg.states) {
    const auto tip = jointAlarmTip(joint);
    if (tip.empty()) {
      continue;
    }

    current_keys.insert(tip);
    if (active_joint_alarm_keys_.count(tip) == 0) {
      const auto priority = joint.joint_error == robot_interfaces::msg::JointState::JOINT_ERR_MOTOR_WARN ? 180 : 200;
      playTip(tip, priority);
    }
  }
  active_joint_alarm_keys_ = std::move(current_keys);
}

// 检测状态变化并播报语音提示。
void RobotStateNode::announceStateChange(const robot_interfaces::msg::RobotState & msg)
{
  if (msg.fault_active &&
    (!prev_fault_active_ || msg.fault_reason != prev_fault_reason_))
  {
    RCLCPP_ERROR(
      get_logger(),
      "System fault active: state=%s fault_reason=\"%s\"",
      msg.state_name.c_str(),
      msg.fault_reason.c_str());
  }

  // 自检结果播报（只播报一次，不重复）。
  const bool self_check_just_done = state_machine_->selfCheckState() != SelfCheckState::PENDING && !self_check_announced_;
  if (self_check_just_done) {
    self_check_announced_ = true;
    const auto pos_str = state_machine_->jointPositionsString();
    RCLCPP_INFO(get_logger(), "Self-check %s:\n%s",
      state_machine_->selfCheckState() == SelfCheckState::PASSED ? "PASSED" : "FAILED",
      pos_str.c_str());
    if (state_machine_->selfCheckState() == SelfCheckState::PASSED) {
      playTip(robot_common::audio_tips::SELF_CHECK_OK, 100);
    } else {
      playTip(robot_common::audio_tips::SELF_CHECK_FAILED, 100);
    }
  }
  // 状态机主状态切换播报（仅首次进入该状态时播报）。
  if (msg.state != prev_state_) {
    prev_state_ = msg.state;
    switch (msg.state) {
      case robot_interfaces::msg::RobotState::STATE_IDLE:
        playTip(robot_common::audio_tips::STATE_IDLE, 100);
        break;  
      case robot_interfaces::msg::RobotState::STATE_DO_READY:
        playTip(robot_common::audio_tips::STATE_DO_READY, 150);
        break;  
      case robot_interfaces::msg::RobotState::STATE_READY:
        playTip(robot_common::audio_tips::STATE_READY, 150);
        break;
      case robot_interfaces::msg::RobotState::STATE_RL_ACTIVE:
        playTip(robot_common::audio_tips::STATE_RL_ACTIVE, 150);
        break;
      case robot_interfaces::msg::RobotState::STATE_DAMP:
        playTip(robot_common::audio_tips::STATE_DAMP, 150);
        break;
      case robot_interfaces::msg::RobotState::STATE_FAULT:
        if (msg.fault_reason.find("imu") != std::string::npos) {
          playTip(robot_common::audio_tips::IMU_DATA_ERROR, 200);
        } else {
          playTip(robot_common::audio_tips::STATE_FAULT, 200);
        }
        break;
      default:
        break;
    }
    prev_fault_active_ = msg.fault_active;
    prev_fault_reason_ = msg.fault_reason;
    return;
  }

  // 故障恢复播报：从 fault_active=true 变 false 时。
  if (prev_fault_active_ && !msg.fault_active) {
    playTip(robot_common::audio_tips::FAULT_RECOVERED, 200);
  }
  prev_fault_active_ = msg.fault_active;
  prev_fault_reason_ = msg.fault_reason;
}

// 处理设置模式服务，更新状态机。
void RobotStateNode::handleSetMode(
  const std::shared_ptr<robot_interfaces::srv::SetRobotMode::Request> request,
  std::shared_ptr<robot_interfaces::srv::SetRobotMode::Response> response)
{
  response->success = state_machine_->setMode(request->target_mode, request->reason);
  response->message = response->success ? "mode accepted" : "mode rejected";
}

// 处理重置故障服务，更新状态机。
void RobotStateNode::handleResetFault(
  const std::shared_ptr<robot_interfaces::srv::ResetFault::Request> request,
  std::shared_ptr<robot_interfaces::srv::ResetFault::Response> response)
{
  requestAudioRecover();
  response->success = state_machine_->resetFault(request->reason);
  response->message = response->success ? "fault reset" : "fault reset rejected";
}

// 请求提示音恢复服务。
void RobotStateNode::requestAudioRecover()
{
  if (!audio_recover_clt_) {
    return;
  }
  if (!audio_recover_clt_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "audio recover service not ready: %s",
      "monitor/audio_recover");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  RCLCPP_INFO(get_logger(), "requesting audio recover service: %s", "monitor/audio_recover");
  audio_recover_clt_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      try {
        const auto response = future.get();
        RCLCPP_INFO(
          get_logger(),
          "audio recover service response success=%d message=%s",
          response->success,
          response->message.c_str());
      } catch (const std::exception & error) {
        RCLCPP_WARN(get_logger(), "audio recover service request failed: %s", error.what());
      }
    });
}

// 处理准备进度消息，更新状态机。
void RobotStateNode::handlePreparationProgress(
  const std_msgs::msg::Float32::SharedPtr msg)
{
  // 当 robot_control 报告准备插值进度达到 1.0 时，标记状态机准备完成。
  if (msg->data >= 1.0F) {
    state_machine_->onPreparationDone();
  }
}

}  // namespace robot_state
