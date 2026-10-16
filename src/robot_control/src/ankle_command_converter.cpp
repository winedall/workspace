#include "robot_control/ankle_command_converter.hpp"

#include <algorithm>
#include <utility>

#include "robot_common/ankle_transform.hpp"

namespace robot_control
{

namespace
{

// 在命令数组中按名称查找可写条目，未找到返回 nullptr。
robot_interfaces::msg::JointCommand * findCommand(
  std::vector<robot_interfaces::msg::JointCommand> & commands, const std::string & joint_name)
{
  for (auto & command : commands) {
    if (command.joint_name == joint_name) {
      return &command;
    }
  }
  return nullptr;
}

// 在状态数组（已缓存）中按名称查找，未找到返回 nullptr。
const robot_interfaces::msg::JointState * findState(
  const robot_interfaces::msg::JointStates & states, const std::string & joint_name)
{
  for (const auto & state : states.states) {
    if (state.joint_name == joint_name) {
      return &state;
    }
  }
  return nullptr;
}

}  // namespace

void AnkleCommandConverter::Configure(rclcpp::Node & node)
{
  logger_ = node.get_logger();

  const auto mode_str = node.declare_parameter<std::string>("ankle_ff_mode", "off");
  if (mode_str == "torque") {
    mode_ = Mode::kTorque;
  } else if (mode_str == "hybrid") {
    mode_ = Mode::kHybrid;
  } else {
    mode_ = Mode::kGain;
    if (mode_str != "off") {
      RCLCPP_WARN(logger_, "Unknown ankle_ff_mode '%s', fallback to 'off'", mode_str.c_str());
    }
  }

  // 兼容旧参数名 ankle_torque_feedforward（等价于 ankle_ff_mode:=torque）。
  const bool legacy_torque_ff = node.declare_parameter<bool>("ankle_torque_feedforward", false);
  if (legacy_torque_ff && mode_ == Mode::kGain) {
    mode_ = Mode::kTorque;
    RCLCPP_WARN(logger_, "ankle_torque_feedforward is deprecated, use ankle_ff_mode:=torque");
  }

  SetMode(
    mode_,
    node.declare_parameter<double>("ankle_ff_torque_limit", 30.0),
    node.declare_parameter<double>("ankle_ff_scale", 1.0));
}

void AnkleCommandConverter::SetMode(Mode mode, double torque_limit, double feedforward_scale)
{
  mode_ = mode;
  torque_limit_ = torque_limit;
  feedforward_scale_ = feedforward_scale;
  RCLCPP_INFO(
    logger_, "Ankle control mode: %s (feedforward limit %.1f Nm, scale %.2f)",
    modeName(), torque_limit_, feedforward_scale_);
}

const char * AnkleCommandConverter::modeName() const
{
  switch (mode_) {
    case Mode::kTorque: return "torque feedforward";
    case Mode::kHybrid: return "hybrid feedforward";
    default: return "equivalent gain";
  }
}

void AnkleCommandConverter::CacheEnvStates(const robot_interfaces::msg::JointStates & msg)
{
  env_states_ = msg;
  has_env_states_ = true;
}

void AnkleCommandConverter::CacheModelStates(const robot_interfaces::msg::JointStates & msg)
{
  model_states_ = msg;
  has_model_states_ = true;
}

void AnkleCommandConverter::Convert(
  std::vector<robot_interfaces::msg::JointCommand> & commands) const
{
  if (mode_ == Mode::kGain) {
    convertGain(commands);
    return;
  }
  convertFeedforward(commands);
}

void AnkleCommandConverter::convertGain(
  std::vector<robot_interfaces::msg::JointCommand> & commands) const
{
  robot_common::ankle_transform::convertToParallel(commands);
}

// 前馈转换（纯力矩 / 混合）：
//   期望力矩 τ_L = J(实测)⁻ᵀ · [effort_ff + kp_H⊙(q*_H − q_H) + kd_H⊙(q̇*_H − q̇_H)]
//   纯力矩：下发 effort = τ_L，kp = kd = 0，position/velocity 写实测并联值
//   混合  ：下发 effort = τ_L − (转换后的前馈 + kp_L⊙Δq_L + kd_L⊙Δq̇_L)，其余字段保持
//           等效增益转换的结果（电机侧仍闭合 PD）
void AnkleCommandConverter::convertFeedforward(
  std::vector<robot_interfaces::msg::JointCommand> & commands) const
{
  using namespace robot_common::ankle_transform;

  // 安全约束 1：实测状态缺失时整体回退到等效增益，避免下发半串半并的帧。
  if (!has_model_states_ || !has_env_states_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "Ankle feedforward: missing joint states, fallback to equivalent gain");
    convertGain(commands);
    return;
  }

  // 先收集两条腿的输入，任一关节缺失同样整体回退。
  const std::pair<const char * const *, bool> legs[] = {
    {kLeftLegJointNames, false},
    {kRightLegJointNames, true},
  };

  Eigen::Vector<double, kLegJointNum> cmd_pos[2], cmd_vel[2], cmd_kp[2], cmd_kd[2], cmd_eff[2];
  Eigen::Vector<double, kLegJointNum> meas_pos[2], meas_vel[2];
  robot_interfaces::msg::JointCommand * ankle_cmd[2][2] = {{nullptr, nullptr}, {nullptr, nullptr}};
  const robot_interfaces::msg::JointState * env_state[2][2] = {{nullptr, nullptr}, {nullptr, nullptr}};

  for (int leg = 0; leg < 2; ++leg) {
    for (int i = 0; i < kLegJointNum; ++i) {
      auto * command = findCommand(commands, legs[leg].first[i]);
      const auto * model_state = findState(model_states_, legs[leg].first[i]);
      const auto * env = findState(env_states_, legs[leg].first[i]);
      if (command == nullptr || model_state == nullptr || env == nullptr) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "Ankle feedforward: joint %s incomplete, fallback to equivalent gain",
          legs[leg].first[i]);
        convertGain(commands);
        return;
      }
      cmd_pos[leg](i) = command->position;
      cmd_vel[leg](i) = command->velocity;
      cmd_kp[leg](i) = command->kp;
      cmd_kd[leg](i) = command->kd;
      cmd_eff[leg](i) = command->effort;
      meas_pos[leg](i) = model_state->position;
      meas_vel[leg](i) = model_state->velocity;
      if (i == kAnklePitchIdx) {
        ankle_cmd[leg][0] = command;
        env_state[leg][0] = env;
      } else if (i == kAnkleRollIdx) {
        ankle_cmd[leg][1] = command;
        env_state[leg][1] = env;
      }
    }
  }

  // 等效增益转换的结果是混合模式的"电机侧基线"；纯力矩模式随后覆盖踝关节字段。
  // 注意：该调用会原地改写 commands，所以上面的串联命令值必须已收集完。
  convertGain(commands);

  for (int leg = 0; leg < 2; ++leg) {
    Eigen::Vector2d tau_desired;
    AnkleTorqueH2L(
      cmd_pos[leg], cmd_vel[leg], cmd_kp[leg], cmd_kd[leg], cmd_eff[leg],
      meas_pos[leg], meas_vel[leg], tau_desired, legs[leg].second);

    for (int motor = 0; motor < 2; ++motor) {
      auto & command = *ankle_cmd[leg][motor];
      const auto & env = *env_state[leg][motor];

      double feedforward = 0.0;
      if (mode_ == Mode::kTorque) {
        feedforward = tau_desired(motor);
        command.kp = 0.0F;
        command.kd = 0.0F;
        // kp=kd=0 时位置/速度被电机忽略，写实测值避免切回位置模式时目标跳变。
        command.position = env.position;
        command.velocity = env.velocity;
      } else {
        // 混合：减去电机侧基线 PD 即将产生的力矩，只补差额。
        const double baseline_pd =
          command.kp * (command.position - env.position) +
          command.kd * (command.velocity - env.velocity);
        feedforward = tau_desired(motor) - (command.effort + baseline_pd);
        // position/velocity/kp/kd 保持等效增益转换的结果不动。
      }

      // 安全约束 2：前馈力矩限幅。
      feedforward *= feedforward_scale_;
      command.effort = static_cast<float>(
        std::clamp(feedforward, -torque_limit_, torque_limit_));
    }
  }
}

}  // namespace robot_control
