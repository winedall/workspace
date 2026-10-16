#ifndef ROBOT_CONTROL__ROBOT_CONTROL_NODE_HPP_
#define ROBOT_CONTROL__ROBOT_CONTROL_NODE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "robot_control/ankle_command_converter.hpp"
#include "robot_control/joint_command_filter.hpp"
#include "robot_interfaces/msg/joint_commands.hpp"
#include "robot_interfaces/msg/joint_states.hpp"
#include "robot_interfaces/msg/rl_manager_status.hpp"
#include "robot_interfaces/msg/robot_state.hpp"

namespace robot_control
{

// robot_control_node 的 ROS 封装层。
//
// 节点订阅经过 command_router 准入的 JointCommands，再执行关节级限幅、
// 状态阻断、踝关节坐标变换和准备插补，最后发布给 dry-run/driver 层。
// 该节点不访问真实硬件。
//
// 踝关节串并联转换：由两个维度联合决定——
//   模型空间（ankle_in_series）：true=串联（heart/walk 等），false=并联（long_jump）
//   环境空间（ankle_env_parallel）：true=并联，false=串联；人形真机固定为并联
// 命令路径：模型输出 → 环境需要，空间不同则转换
// 状态路径：环境输出 → 模型需要，空间不同则转换
class RobotControlNode : public rclcpp::Node
{
public:
  explicit RobotControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 判断给定系统状态是否阻断当前命令。FAULT 下只允许阻尼命令继续下发。
  static bool systemBlocksCommand(
    const robot_interfaces::msg::RobotState & state,
    const robot_interfaces::msg::JointCommands & commands);

  // command_router 输出的 accepted_commands 入口。
  void handleAcceptedCommands(const robot_interfaces::msg::JointCommands::SharedPtr msg);

  // driver 反馈的原始关节状态入口（driver/joint_states）。
  void handleJointStates(const robot_interfaces::msg::JointStates::SharedPtr msg);

  // RL manager 状态入口：缓存当前模型踝关节空间信息（ankle_in_series）。
  void handleRlManagerStatus(const robot_interfaces::msg::RlManagerStatus::SharedPtr msg);

  // state_manager 输出的系统状态入口，用于 FAULT 全局阻断。
  void handleSystemState(const robot_interfaces::msg::RobotState::SharedPtr msg);

  // 控制周期 timer，只负责准备动作插值。
  void handleControlTimer();

  // 统一硬件命令发布入口：踝关节空间转换 + 组帧 + 发布。
  void publishHardwareCommands(const robot_interfaces::msg::JointCommands & cmd, const std::string & stage);

  // 构造硬件命令帧（更新 sequence/stage 等元数据）。
  robot_interfaces::msg::JointCommands makeHardwareCommandFrame(
    const robot_interfaces::msg::JointCommands & input,
    const std::string & stage);

  // 发布被 control 层拒绝的命令，便于上游调试和诊断。
  void publishRejected(const robot_interfaces::msg::JointCommands & commands);

  // 准备动作插值控制。
  void startPrepareInterpolation(const robot_interfaces::msg::JointCommands & commands);
  // 进行准备动作插值。
  void advancePrepareInterpolation();
  // 查找指定关节的当前状态。
  const robot_interfaces::msg::JointState * findJointState(const std::string & joint_name) const;
  const robot_interfaces::msg::JointState * findJointStateIn(
    const robot_interfaces::msg::JointStates & states,
    const std::string & joint_name) const;
  // 检查当前准备目标是否与指定命令相同。
  bool samePrepareTarget(const robot_interfaces::msg::JointCommands & commands) const;

  // 纯逻辑组件，便于单元测试；节点层只做参数、topic 和时间管理。
  std::unique_ptr<JointCommandFilter> filter_;

  // 最近状态缓存（已转换到模型期望空间）。JointStates 用于准备插值起点；
  // RobotState 用于安全阻断。
  robot_interfaces::msg::JointStates latest_raw_joint_states_;
  robot_interfaces::msg::JointStates latest_joint_states_;
  robot_interfaces::msg::RobotState latest_system_state_;
  bool have_raw_joint_states_{false};
  bool have_joint_states_{false};
  bool have_system_state_{false};

  // 踝关节命令转换器（等效增益 / 纯力矩前馈 / 混合前馈）。
  // 自带环境空间与模型空间的实测状态缓存，节点只需在收发回调里调用。
  AnkleCommandConverter ankle_converter_;

  // ROS 参数缓存。
  bool enable_rejected_command_topic_{true};
  bool enable_mujoco_sim_{true};
  bool ankle_env_parallel_{false};  // true=环境为并联空间, false=环境为串联空间
  std::string robot_type_;
  bool has_ankle_;

  // 当前模型踝关节空间缓存（来自 RlManagerStatus.ankle_in_series），
  // 由 handleRlManagerStatus 更新，handleJointStates 读取做 P2S 决策。
  bool ankle_in_series_{true};
  bool have_rl_status_{false};
  // 最近命令帧携带的 ankle_in_series，用于模型切换时快速纠正状态路径缓存。
  bool last_cmd_ankle_in_series_{true};

  // ROS 通信接口。
  rclcpp::Subscription<robot_interfaces::msg::JointCommands>::SharedPtr accepted_commands_sub_;
  // 订阅 driver 原始关节状态。
  rclcpp::Subscription<robot_interfaces::msg::JointStates>::SharedPtr joint_states_sub_;
  // 订阅 RL manager 状态，用于更新当前模型踝关节空间。
  rclcpp::Subscription<robot_interfaces::msg::RlManagerStatus>::SharedPtr rl_status_sub_;
  // 订阅 state_manager 状态，用于 FAULT 全局阻断。
  rclcpp::Subscription<robot_interfaces::msg::RobotState>::SharedPtr state_sub_;
  // 发布给 driver 的硬件命令（已转换到环境所需空间）。
  rclcpp::Publisher<robot_interfaces::msg::JointCommands>::SharedPtr hardware_commands_pub_;
  // 发布经空间转换后的硬件状态（匹配模型期望空间），供 RL/state_manager 等上游使用。
  rclcpp::Publisher<robot_interfaces::msg::JointStates>::SharedPtr hardware_states_pub_;
  // 发布被 control 层拒绝的命令，便于上游调试和诊断。
  rclcpp::Publisher<robot_interfaces::msg::JointCommands>::SharedPtr rejected_commands_pub_;
  // 发布准备动作插值进度，便于监控和调试。
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr preparation_progress_pub_;
  // 控制周期 timer，只负责准备动作插值。
  rclcpp::TimerBase::SharedPtr control_timer_;

  // 准备动作插值状态。
  bool preparing_{false};
  int prepare_cycle_{0};
  int prepare_total_cycles_{1000};
  std::vector<float> prepare_start_positions_;
  robot_interfaces::msg::JointCommands prepare_target_;
  std::uint32_t hardware_command_sequence_{0};
};

}  // namespace robot_control

#endif  // ROBOT_CONTROL__ROBOT_CONTROL_NODE_HPP_
