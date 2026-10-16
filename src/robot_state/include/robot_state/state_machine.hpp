#ifndef ROBOT_STATE__STATE_MACHINE_HPP_
#define ROBOT_STATE__STATE_MACHINE_HPP_

#include <string>
#include <iostream>

#include "robot_common/joint_config.hpp"
#include "robot_interfaces/msg/driver_status.hpp"
#include "robot_interfaces/msg/joint_states.hpp"
#include "robot_interfaces/msg/robot_intent.hpp"
#include "robot_interfaces/msg/rl_manager_status.hpp"
#include "robot_interfaces/msg/robot_state.hpp"

namespace robot_state
{

// 系统状态枚举，与 robot_interfaces/msg/RobotState.msg 中的 STATE_* 常量保持一致。
// 使用 enum class 是为了在 C++ 内部避免裸 uint8_t 状态值误用。
enum class SystemState : std::uint8_t
{
  BOOTING = robot_interfaces::msg::RobotState::STATE_BOOTING,
  INIT = robot_interfaces::msg::RobotState::STATE_INIT,
  IDLE = robot_interfaces::msg::RobotState::STATE_IDLE,
  DO_READY = robot_interfaces::msg::RobotState::STATE_DO_READY,
  READY = robot_interfaces::msg::RobotState::STATE_READY,
  RL_STANDBY = robot_interfaces::msg::RobotState::STATE_RL_STANDBY,
  RL_ACTIVE = robot_interfaces::msg::RobotState::STATE_RL_ACTIVE,
  RULE_STANDBY = robot_interfaces::msg::RobotState::STATE_RULE_STANDBY,
  RULE_ACTIVE = robot_interfaces::msg::RobotState::STATE_RULE_ACTIVE,
  TRANSITIONING = robot_interfaces::msg::RobotState::STATE_TRANSITIONING,
  FAULT = robot_interfaces::msg::RobotState::STATE_FAULT,
  DAMP = robot_interfaces::msg::RobotState::STATE_DAMP,
};

// 开机自检状态。
enum class SelfCheckState : std::uint8_t
{
  PENDING,  // 未执行，或需重新执行（如从 FAULT 恢复后）
  PASSED,   // 自检通过，可放行 READY
  FAILED,   // 自检失败（对应进入 FAULT）
};

// 机器人系统状态机核心。
//
// 该类只处理状态转换和安全准入判断，不继承 rclcpp::Node，不订阅/发布 topic，
// 因此可以通过 gtest 直接验证。ROS 节点层负责把订阅、服务、定时发布转换为
// 对这些方法的调用。
class StateMachine
{
public:
  // 创建状态机。初始状态固定为 BOOTING。
  //
  // robot_joints_config_path 用于加载 robot_joints.yaml。
  explicit StateMachine(
    const std::string & robot_joints_config_path,
    bool sim_mode = false);

  // 参数加载完成：BOOTING -> INIT。
  void onParametersLoaded();

  // 加载 robot_joints.yaml，成功后 INIT -> IDLE。加载失败由异常向节点层暴露。
  void loadConfiguration();

  // 更新最近一次关节状态。
  void onJointStates(const robot_interfaces::msg::JointStates & msg);

  // 更新最近一次驱动状态。
  void onDriverStatus(const robot_interfaces::msg::DriverStatus & msg);

  // 更新最近一次 RL 管理节点状态。
  void onRlManagerStatus(const robot_interfaces::msg::RlManagerStatus & msg);

  // 标记准备动作插值完成。外部节点收到 robot_control 的准备进度 topic 后调用此方法。
  void onPreparationDone();

  // 根据当前输入快照推进状态机。节点层周期调用该函数。
  void evaluate();

  // 外部请求切换模式。这里只接受高层模式，不发布任何 JointCommands。
  bool setMode(const std::string & target_mode, const std::string & reason);

  // 消费 command_router 已准入的 RobotIntent。只处理状态机相关 intent；
  // config/motion/model 等不属于状态机管理范围的意图会被忽略并返回 false。
  bool handleIntent(const robot_interfaces::msg::RobotIntent & intent);

  // 进入 FAULT 并记录原因。
  void enterFault(const std::string & reason);
  
  // 复位故障：只有 FAULT 且输入状态恢复健康时才能回到 IDLE。
  bool resetFault(const std::string & reason);

  // 以下访问器用于测试、节点发布和服务响应。
  SystemState state() const;
  bool faultActive() const;
  std::string faultReason() const;
  SelfCheckState selfCheckState() const;

  // 返回所有关节位置的格式化字符串（关节名 = 位置），用于日志调试。
  std::string jointPositionsString() const;

  robot_interfaces::msg::RobotState toMessage() const;

private:
  // 驱动是否在线且未报告 fault。
  bool driverHealthy() const;

  // RL 管理节点是否在线、模型已加载并 ready，且未报告推理错误。
  bool rlManagerReady() const;

  // 当前 RL 管理状态中的 IMU 输入是否可用。
  bool imuInputReady() const;

  // 关节错误码是否允许继续运行。温度告警暂不立即进入 FAULT。
  bool jointsHealthy() const;

  // 执行开机自检，对每个 enable_default=true 的关节：
  //   1) position == 0.0f → 通讯异常，未读到真实编码器值
  //   2) position 超出 [min_pos, max_pos] → 位置超限
  // 失败时通过 fail_reason 返回具体关节和原因。
  bool runSelfCheck(std::string & fail_reason) const;

  // 进入 READY 的综合条件。
  bool readyConditionsMet() const;

  // 内部状态更新。active_mode 暂不在基础状态机阶段维护。
  void setState(SystemState state);

  // 将状态枚举转换为稳定字符串，供 RobotState.state_name 使用。
  std::string stateName(SystemState state) const;

  // 配置参数。
  std::string robot_joints_config_path_;
  robot_common::RobotJointsConfig joints_config_;
  bool config_loaded_{false};

  // 最近一次输入快照。状态机只使用快照判断，不在回调中做复杂状态跳转。
  bool have_joint_states_{false};
  bool have_driver_status_{false};
  bool have_rl_manager_status_{false};
  // 关节数据是否已进入可信阶段。初始 IDLE 时 driver 可能尚未完成首个 EtherCAT 周期，
  // 此时数据全零或只有零散非零值，自检跳过以避免误判为通讯异常。当前帧中至少
  // 1/3 的 enable_default 关节 position 非零后置 true 且不再回退。
  bool data_settled_{false};
  // data_settled_ 置 true 后等待的帧数，达到 kSettleFrameCount 后才放行自检，
  // 确保所有关节编码器在可信帧之后再积累多帧稳定读数，避免启动初期少量有效
  // 读数导致自检误判通讯异常。
  size_t settled_frame_count_{0};
  static constexpr size_t kSettleFrameCount{100};
  robot_interfaces::msg::JointStates last_joint_states_;
  robot_interfaces::msg::DriverStatus last_driver_status_;
  robot_interfaces::msg::RlManagerStatus last_rl_manager_status_;
  size_t imu_fault_frame_count_{0};
  static constexpr size_t kImuFaultConfirmFrameCount{10};

  // 当前状态和对外发布的附加状态字段。
  SystemState state_{SystemState::BOOTING};
  bool fault_active_{false};
  std::string fault_reason_;
  std::string active_mode_;
  std::string active_control_source_;
  std::string active_rl_model_;

  // 仿真模式标志，仿真环境下跳过关节零位通讯异常检测。
  bool sim_mode_{false};

  // 准备动作插值是否已完成（由 robot_control 汇报）。
  bool preparation_done_{false};

  // 开机自检状态。启动时为 PENDING，进入 IDLE 且有 joint_states 后执行一次。
  SelfCheckState self_check_state_{SelfCheckState::PENDING};
};

}  // namespace robot_state

#endif  // ROBOT_STATE__STATE_MACHINE_HPP_
