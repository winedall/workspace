#ifndef ROBOT_CONTROL__ANKLE_COMMAND_CONVERTER_HPP_
#define ROBOT_CONTROL__ANKLE_COMMAND_CONVERTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/joint_commands.hpp"
#include "robot_interfaces/msg/joint_states.hpp"

namespace robot_control
{

// 踝关节命令转换器：把模型空间（串联）的踝命令转换到环境空间（并联）电机命令。
//
// 三种模式（由 ankle_ff_mode 参数选择）：
//   kGain   等效增益（默认，原行为）：串联 kp/kd 换算成并联电机增益，电机内闭合 PD。
//   kTorque 纯力矩前馈：串联空间算 PD → J⁻ᵀ → 并联力矩，kp/kd 置零由电机执行纯力矩。
//           精度最高，但整个 PD 闭环移到 200 Hz 控制链路，回路内延迟（命令 + 反馈）
//           必须亚毫秒~几毫秒，且断流时电机会保持最后一帧力矩。
//   kHybrid 混合前馈（推荐）：保留等效增益的 position/kp/kd（电机内闭合 PD），只把
//           "串联 PD 应产生 − 电机 PD 实际产生"的差额作为 effort 前馈补上。刚度/
//           阻尼/断流安全留在电机内环，前馈只修正增益等效做不对的部分（roll 增益、
//           耦合项、姿态漂移），对命令延迟和反馈延迟都不敏感。
//
// 数值实现全部在 robot_common/ankle_transform（AnkleTorqueH2L 等纯数学），本类只负责
// 参数、状态缓存和消息级管线。节点侧只需：构造时 Configure()、收到反馈时 Cache*()、
// 下发前 Convert()。
class AnkleCommandConverter
{
public:
  enum class Mode
  {
    kGain,     // 等效增益（原行为）
    kTorque,   // 纯力矩前馈
    kHybrid,   // 混合前馈
  };

  // 从 ROS 参数读取配置并打印生效模式。
  //   ankle_ff_mode            : "off" | "torque" | "hybrid"（默认 off）
  //   ankle_torque_feedforward : 旧参数名，true 等价于 ankle_ff_mode=torque（弃用）
  //   ankle_ff_torque_limit    : 前馈力矩限幅（Nm，默认 30）
  //   ankle_ff_scale           : 前馈权重 0~1（默认 1.0，用于扫稳定性/精度折中）
  void Configure(rclcpp::Node & node);

  // 直接设置模式与参数（供单元测试和非 ROS 场景使用，不依赖 rclcpp）。
  void SetMode(Mode mode, double torque_limit = 30.0, double feedforward_scale = 1.0);

  // 缓存最近一帧实测状态。
  // handleJointStates 中调用两次：串并联转换**前**缓存环境空间（并联），转换**后**缓存模型空间（串联）。
  void CacheEnvStates(const robot_interfaces::msg::JointStates & msg);
  void CacheModelStates(const robot_interfaces::msg::JointStates & msg);

  // 就地转换命令中的踝关节字段（模型串联 → 环境并联），按 Configure 的模式分派。
  // 实测状态缺失或踝关节数据不完整时整体回退到等效增益转换，避免下发半串半并的帧。
  void Convert(std::vector<robot_interfaces::msg::JointCommand> & commands) const;

  Mode mode() const {return mode_;}
  const char * modeName() const;

private:
  // 等效增益转换（robot_common::ankle_transform::convertToParallel）。
  void convertGain(std::vector<robot_interfaces::msg::JointCommand> & commands) const;
  // 前馈转换（纯力矩 / 混合），需要实测状态。
  void convertFeedforward(std::vector<robot_interfaces::msg::JointCommand> & commands) const;

  Mode mode_{Mode::kGain};
  double torque_limit_{30.0};
  double feedforward_scale_{1.0};

  robot_interfaces::msg::JointStates env_states_;    // 环境空间（并联）实测
  robot_interfaces::msg::JointStates model_states_;  // 模型空间（串联）实测
  bool has_env_states_{false};
  bool has_model_states_{false};

  rclcpp::Logger logger_{rclcpp::get_logger("ankle_command_converter")};
  rclcpp::Clock::SharedPtr clock_{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)};
};

}  // namespace robot_control

#endif  // ROBOT_CONTROL__ANKLE_COMMAND_CONVERTER_HPP_
