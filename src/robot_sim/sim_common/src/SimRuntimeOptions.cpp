#include "sim_common/SimRuntimeOptions.hpp"

namespace sim_common {

SimViewerOptions DeclareViewerOptions(rclcpp::Node* node) {
  // 用结构体自身的默认值作为 declare 的默认值：不配置时行为与改造前一致。
  SimViewerOptions o;
  o.ui0_enable = node->declare_parameter<bool>("viewer.ui0_enable", o.ui0_enable);
  o.ui1_enable = node->declare_parameter<bool>("viewer.ui1_enable", o.ui1_enable);
  o.spacing = node->declare_parameter<int>("viewer.spacing", o.spacing);
  o.color = node->declare_parameter<int>("viewer.color", o.color);
  o.font = node->declare_parameter<int>("viewer.font", o.font);
  o.help = node->declare_parameter<bool>("viewer.help", o.help);
  o.info = node->declare_parameter<bool>("viewer.info", o.info);
  o.profiler = node->declare_parameter<bool>("viewer.profiler", o.profiler);
  o.sensor = node->declare_parameter<bool>("viewer.sensor", o.sensor);
  o.vsync = node->declare_parameter<bool>("viewer.vsync", o.vsync);
  o.pause_update = node->declare_parameter<bool>("viewer.pause_update", o.pause_update);
  return o;
}

SimDynamicsOptions DeclareDynamicsOptions(rclcpp::Node* node) {
  SimDynamicsOptions o;
  o.start_paused = node->declare_parameter<bool>("dynamics.start_paused", o.start_paused);
  o.busywait = node->declare_parameter<bool>("dynamics.busywait", o.busywait);
  o.refresh_rate = node->declare_parameter<int>("dynamics.refresh_rate", o.refresh_rate);
  o.realtime_percent = node->declare_parameter<double>("dynamics.realtime_percent", o.realtime_percent);
  return o;
}

SimErrorOptions DeclareErrorOptions(rclcpp::Node* node) {
  SimErrorOptions o;
  o.seed = node->declare_parameter<int>("errors.seed", static_cast<int>(o.seed));
  o.joint_zero_offset_rad =
      node->declare_parameter<std::vector<double>>("errors.joint_zero_offset_rad", o.joint_zero_offset_rad);
  o.joint_zero_offset_std_rad =
      node->declare_parameter<double>("errors.joint_zero_offset_std_rad", o.joint_zero_offset_std_rad);
  o.joint_pos_noise_std_rad =
      node->declare_parameter<double>("errors.joint_pos_noise_std_rad", o.joint_pos_noise_std_rad);
  o.joint_vel_noise_std_rad_s =
      node->declare_parameter<double>("errors.joint_vel_noise_std_rad_s", o.joint_vel_noise_std_rad_s);
  o.mount_std_deg = node->declare_parameter<double>("errors.mount_std_deg", o.mount_std_deg);
  o.gyro_bias_std_rad_s =
      node->declare_parameter<double>("errors.gyro_bias_std_rad_s", o.gyro_bias_std_rad_s);
  o.acc_bias_std_m_s2 = node->declare_parameter<double>("errors.acc_bias_std_m_s2", o.acc_bias_std_m_s2);
  o.gyro_noise_std_rad_s =
      node->declare_parameter<double>("errors.gyro_noise_std_rad_s", o.gyro_noise_std_rad_s);
  o.acc_noise_std_m_s2 =
      node->declare_parameter<double>("errors.acc_noise_std_m_s2", o.acc_noise_std_m_s2);
  o.att_bias_std_deg = node->declare_parameter<double>("errors.att_bias_std_deg", o.att_bias_std_deg);
  o.att_noise_std_deg = node->declare_parameter<double>("errors.att_noise_std_deg", o.att_noise_std_deg);
  o.torque_delay_ms = node->declare_parameter<double>("errors.torque_delay_ms", o.torque_delay_ms);
  o.drop_probability = node->declare_parameter<double>("errors.drop_probability", o.drop_probability);
  o.command_delay_ms =
      node->declare_parameter<double>("errors.command_delay_ms", o.command_delay_ms);
  o.feedback_hold_ms =
      node->declare_parameter<double>("errors.feedback_hold_ms", o.feedback_hold_ms);
  o.feedback_delay_ms =
      node->declare_parameter<double>("errors.feedback_delay_ms", o.feedback_delay_ms);
  return o;
}

}  // namespace sim_common
