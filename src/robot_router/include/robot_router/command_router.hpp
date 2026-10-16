#ifndef ROBOT_COMMAND_ROUTER__COMMAND_ROUTER_HPP_
#define ROBOT_COMMAND_ROUTER__COMMAND_ROUTER_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "robot_common/joint_config.hpp"
#include "robot_interfaces/msg/driver_status.hpp"
#include "robot_interfaces/msg/joint_commands.hpp"
#include "robot_interfaces/msg/robot_intent.hpp"
#include "robot_interfaces/msg/robot_state.hpp"

namespace robot_router
{

// 控制模式：决定是否合并外部控制指令。
enum class ControlMode : uint8_t
{
  RL = 0,   // 纯 RL 控制，忽略外部指令
  MIX = 1,  // 混合模式，合并外部指令到 RL 输出
};

// 路由器运行参数。这里只保存准入判断需要的配置，不直接持有 ROS 节点对象，
// 这样核心逻辑可以脱离 rclcpp 做单元测试。
struct RouterConfig
{
  // 机器人关节配置文件路径，用于校验 JointCommands 中的 joint_name 是否属于已知关节。
  std::string robot_joints_config;

  // 有效优先级由 source 决定范围；数值越小优先级越高。
  int teleop_priority{0};
  int app_priority_min{1};
  int app_priority_max{49};
  int api_priority_min{50};
  int api_priority_max{99};
  int unknown_priority{255};
};

// 单次路由判断结果。reason 用于日志和测试定位，不作为机器可解析协议。
struct RouteDecision
{
  bool accepted{false};
  std::string reason;
};

// CommandRouter 只负责"准入"和"路由选择"，不做最终限幅，也不访问真实硬件。
// 最终位置/速度/力矩限制属于 robot_control，硬件输出属于 robot_driver。
class CommandRouter
{
public:
  explicit CommandRouter(const RouterConfig & config);

  // 外部状态由 ROS 节点订阅后注入，核心类只缓存最近状态用于安全判断。
  void updateSystemState(const robot_interfaces::msg::RobotState & state);
  void updateDriverStatus(const robot_interfaces::msg::DriverStatus & status);

  // 评估上层意图：处理优先级、故障状态下的运动拒绝。
  RouteDecision evaluateIntent(
    const robot_interfaces::msg::RobotIntent & intent);

  // 评估执行级关节命令：处理来源、时序、重复关节和系统安全状态。
  RouteDecision evaluateJointCommands(
    const robot_interfaces::msg::JointCommands & commands);

  // 返回当前激活意图，主要用于测试和后续状态回报接入。
  const robot_interfaces::msg::RobotIntent & activeIntent() const;

  // 构造 command_router accepted 输出帧。输出 sequence 属于 command_router 自己，
  // 上游 desired sequence 通过 parent_sequence 保留。
  robot_interfaces::msg::JointCommands makeAcceptedJointCommands(
    const robot_interfaces::msg::JointCommands & commands);

  // 设置控制模式：rl（纯 RL）或 mix（混合模式）。
  void setControlMode(ControlMode mode);

  // 设置 app 模式开关。app 模式下 teleop 仅放行安全/模式切换白名单。
  void setAppMode(bool enabled);
  // 返回当前是否处于 app 模式。
  bool appMode() const;
  // 清空当前激活意图，让后续意图（如 APP）能够接管控制权。
  void clearActiveIntent();
  // 判断 teleop 意图在 app 模式下是否属于放行白名单。
  // 放行项：config/mode（模式切换）、status/damp、model/motion_stop。
  // getup/speed/nav 等配置、reset/reset_fault、switch_model 等均交由 APP 控制。
  bool isTeleopAllowedInAppMode(const robot_interfaces::msg::RobotIntent & intent) const;

  // MIX 模式下缓存外部控制指令，供 merge 时使用。
  void cacheExternalCommands(
    const robot_interfaces::msg::JointCommands & commands);

  // 将 RL 指令与缓存的外部指令合并。
  // mix 模式：外部关节覆盖 RL，enable 取 OR；rl 模式或无条件直接返回 RL。
  robot_interfaces::msg::JointCommands merge(
    const robot_interfaces::msg::JointCommands & rl_cmds) const;

  // MIX 模式下返回当前缓存的外部指令中涉及的关节名称集合，用于过滤硬件状态。
  std::unordered_set<std::string> externalJointNames() const;

private:
  // FAULT 或状态消息中的故障标志都会阻断运动类命令。
  bool isSystemBlockingMotion() const;
  // 判断当前是否请求手动 damp 命令。
  bool isManualFaultDampIntent(const robot_interfaces::msg::RobotIntent & intent) const;
  // 判断当前是否请求故障复位。
  bool isManualFaultResetIntent(const robot_interfaces::msg::RobotIntent & intent) const;
  // 判断当前是否请求故障 damp 命令。
  bool isFaultDampingCommand(const robot_interfaces::msg::JointCommands & commands) const;

  // 判断当前是否允许放行运动命令；默认必须收到在线且无故障的驱动状态。
  bool driverAllowsMotion() const;

  // 根据 intent_type 和 command 区分运动意图。
  bool isMotionIntent(const robot_interfaces::msg::RobotIntent & intent) const;

  // 判断 JointCommands 来源是否与当前控制模式或激活意图匹配。
  bool sourceAllowedForJointCommands(const robot_interfaces::msg::JointCommands & commands) const;

  // 计算意图有效优先级；source 决定优先级区间，消息 priority 只能在区间内微调。
  int intentPriority(const robot_interfaces::msg::RobotIntent & intent) const;

  RouterConfig config_;

  // 关节白名单来自 robot_joints.yaml，只用于准入检查，不保存运行时关节状态。
  robot_common::RobotJointsConfig joints_config_;

  // 最近一次系统状态和驱动状态缓存，供回调之间共享安全上下文。
  robot_interfaces::msg::RobotState system_state_;
  robot_interfaces::msg::DriverStatus driver_status_;

  // 当前激活意图用于优先级比较和控制源匹配。
  robot_interfaces::msg::RobotIntent active_intent_;
  bool have_active_intent_{false};
  bool have_driver_status_{false};

  // 当前是否处于 app 模式（mode 菜单切入）。app 模式下 teleop 仅放行安全/模式切换白名单。
  bool app_mode_{false};

  // 控制模式：rl（纯 RL）或 mix（混合模式）。
  ControlMode control_mode_{ControlMode::RL};

  // 最近一帧外部控制指令缓存，merge 时使用。
  std::optional<robot_interfaces::msg::JointCommands> cached_external_commands_;

  // 记录各命令源最近接受的输入 sequence，避免不同发布者的序号域互相污染。
  std::unordered_map<std::string, std::uint32_t> last_accepted_sequence_by_source_;
  // command_router 在 accepted topic 上的输出序号。
  std::uint32_t accepted_output_sequence_{0};
};

}  // namespace robot_router

#endif  // ROBOT_COMMAND_ROUTER__COMMAND_ROUTER_HPP_
