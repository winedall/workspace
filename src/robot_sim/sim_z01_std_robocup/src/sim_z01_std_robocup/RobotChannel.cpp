#include "sim_z01_std_robocup/RobotChannel.h"

#include <cmath>
#include <utility>

#include "sim_common/control_math.hpp"

namespace sim_z01_std_robocup {
namespace {

// 由偏航角构造 MuJoCo 浮动基座四元数（w, x, y, z）。
void YawToQuat(double yaw, double* quat) {
  quat[0] = std::cos(yaw * 0.5);
  quat[1] = 0.0;
  quat[2] = 0.0;
  quat[3] = std::sin(yaw * 0.5);
}

}  // namespace

RobotChannel::RobotChannel(const RobotModelSpec& spec, const RobotChannelOptions& opts,
                           const rclcpp::NodeOptions& node_options)
    : spec_(spec),
      opts_(opts),
      mj_interface_(opts.mj_config),
      ros_(std::make_shared<sim_common::RobotRosChannel>(opts.ros, node_options)),
      motor_specs_(MotorSpecs().begin(), MotorSpecs().end()),
      q_rel_(Eigen::VectorXd::Zero(static_cast<Eigen::Index>(spec.kJointCount) + 7)),
      dq_rel_(Eigen::VectorXd::Zero(static_cast<Eigen::Index>(spec.kJointCount) + 6)),
      tau_rel_(Eigen::VectorXd::Zero(static_cast<Eigen::Index>(spec.kJointCount) + 6)),
      q_pub_(Eigen::VectorXd::Zero(static_cast<Eigen::Index>(spec.kJointCount) + 7)),
      dq_pub_(Eigen::VectorXd::Zero(static_cast<Eigen::Index>(spec.kJointCount) + 6)),
      tau_pub_(Eigen::VectorXd::Zero(static_cast<Eigen::Index>(spec.kJointCount) + 6)) {}

bool RobotChannel::LoadModel(mjModel* model) {
  ros_->set_camera_period_us(opts_.camera_period_us);
  mj_interface_.LoadMjModel(model);
  if (!mj_interface_.InitDataHandle()) {
    RCLCPP_ERROR(ros_->get_logger(),
                 "MuJoCo 名字映射失败（检查 robot_0X_ 前缀与生成头是否同步）: base=%s",
                 opts_.mj_config.base_name.c_str());
    return false;
  }
  model_ = model;
  ready_ = true;

  // 误差注入：模型加载后按 errors.seed + 机器人序号抽一次常值误差（Reload 后不变）。
  // 种子混入序号，6 台的零位偏置/IMU 零偏各不相同。
  error_model_.Configure(opts_.errors);
  if (error_model_.enabled()) {
    error_model_.Init(mj_interface_.GetJointCount(), mj_interface_.GetMotorIds(), model->opt.timestep,
                      static_cast<unsigned int>(spec_.index));
    mj_interface_.SetErrorModel(&error_model_);
    // 通信层也要拿到模型：指令延迟施加在 UpdateCommandCache（所有包共用的指令写入点）。
    ros_->SetErrorModel(&error_model_);
    RCLCPP_INFO(ros_->get_logger(), "误差注入: %s", error_model_.Summary().c_str());
    if (!error_model_.Warning().empty()) {
      RCLCPP_WARN(ros_->get_logger(), "%s", error_model_.Warning().c_str());
    }
  } else {
    mj_interface_.SetErrorModel(nullptr);
    ros_->SetErrorModel(nullptr);
    // 单机路径（SimNodeBase）有这个提示，多机这里原来没有——配了却静默不生效更难查。
    if (opts_.errors.torque_delay_ms > 0.0 || opts_.errors.drop_probability > 0.0 ||
        opts_.errors.feedback_hold_ms > 0.0 || opts_.errors.feedback_delay_ms > 0.0 ||
        opts_.errors.command_delay_ms > 0.0) {
      RCLCPP_WARN(ros_->get_logger(),
                  "errors 下的时序参数已配置但 errors.seed=0（整体关闭），不会生效；"
                  "只加时序（延迟/采样保持）请设 seed>0 并把各误差标准差留 0");
    }
  }
  return true;
}

void RobotChannel::ApplyTorqueDelay(mjData* data) {
  error_model_.ApplyTorqueDelay(data);
}

bool RobotChannel::ConfigureCamera(const mjModel* model, mj::Simulate* sim) {
  if (!ready_ || model == nullptr || sim == nullptr || mj_interface_.GetCamIds().empty()) {
    return false;
  }
  sim_ = sim;
  const int cam_id = mj_interface_.GetCamIds()[0];
  const int width = model->cam_resolution[cam_id * 2];
  const int height = model->cam_resolution[cam_id * 2 + 1];
  {
    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
    // 每台机器人一个深度相机槽位（槽位号 = 机器人序号）。多机场景不绘制窗口预览：
    // 6 路平铺在窗口里既费性能也没意义。
    depth_slot_ = spec_.index;
    if (!sim_->ConfigureDepthCamera(depth_slot_, cam_id, width, height)) {
      RCLCPP_ERROR(ros_->get_logger(), "深度相机槽位 %d 配置失败", depth_slot_);
      depth_slot_ = -1;
    }
  }
  if (opts_.camera_period_us > 0) {
    ros_->UpdateCameraInfo(*model, cam_id);
  }
  RCLCPP_INFO(ros_->get_logger(), "robot initialized: topics=%s camera_period_us=%ld slot=%d",
              opts_.ros.topic_prefix.c_str(), static_cast<long>(opts_.camera_period_us),
              depth_slot_);
  return true;
}

void RobotChannel::ComputeControl(const mjModel* model, mjData* data) {
  (void)model;
  if (!ready_ || data == nullptr) {
    return;
  }
  // 读取当前仿真位置和速度，用于 PD 控制计算。
  mj_interface_.ReadJointState(data, q_rel_, dq_rel_);
  // 只取队列中的最新一帧并合并到缓存（可能高于仿真消费频率）。
  ros_->ConsumeLatestCommand(data->time);

  // 统一力矩计算路径（与 football_player 一致）：前馈 + PD → 电机力矩限幅 → TN 曲线。
  const auto& motor_ids = mj_interface_.GetMotorIds();
  const auto& commands = ros_->commands();
  const bool enable_control = ros_->enable_control();
  for (std::size_t i = 0; i < mj_interface_.GetJointCount(); i++) {
    const Eigen::Index idx = static_cast<Eigen::Index>(i);
    const auto& command = commands[i];
    data->ctrl[motor_ids[i]] = sim_common::ComputeJointTorque(
        motor_specs_[i], enable_control && command.enable, command.effort, command.kp, command.kd,
        command.position, q_rel_[idx + 7], command.velocity, dq_rel_[idx + 6]);
  }
}

void RobotChannel::PublishTick(const mjData* data, std::int64_t time_us, bool time_reset) {
  if (!ready_ || data == nullptr) {
    return;
  }
  // 仿真时间重置或节拍落后过多时重新对齐，避免长期不发布或连续补发。
  if (time_reset || time_us + opts_.state_publish_period_us < next_pub_time_us_) {
    next_pub_time_us_ = time_us;
  }
  if (time_us < next_pub_time_us_) {
    return;
  }
  next_pub_time_us_ += opts_.state_publish_period_us;

  mj_interface_.ReadState(data, q_rel_, dq_rel_, tau_rel_, acc_);
  // 反馈上行的采样保持 / 纯延迟（errors.feedback_*）：只改发布出去的这份快照，
  // q_rel_/dq_rel_/tau_rel_ 保持新鲜供 PD 回路使用。IMU 仍用新鲜值——真机上
  // IMU 与关节反馈是两路独立传感器。
  error_model_.ApplyFeedbackDelay(data->time, q_rel_, dq_rel_, tau_rel_, q_pub_, dq_pub_,
                                  tau_pub_);
  ros_->PublishJointStateFrame(q_pub_, dq_pub_, tau_pub_);
  ros_->PublishImuFrame(q_rel_, dq_rel_, acc_);
}

void RobotChannel::CameraTick(std::int64_t time_us, bool time_reset) {
  if (!ready_ || sim_ == nullptr) {
    return;
  }
  const std::int64_t period_us = ros_->camera_period_us();
  if (period_us <= 0) {
    return;
  }
  if (time_reset || time_us + period_us < next_cam_time_us_) {
    next_cam_time_us_ = time_us;
  }
  if (time_us < next_cam_time_us_) {
    return;
  }
  next_cam_time_us_ += period_us;
  PublishDepthCameraFrame();
}

void RobotChannel::PublishDepthCameraFrame() {
  if (depth_slot_ < 0) {
    return;
  }
  // 请求渲染线程捕获一帧；depthcam_wait_us 默认 0 = 不等待（异步），
  // 这一轮没拿到新帧就跳过（下一轮自然拿到），物理线程不因相机阻塞。
  sim_->RequestDepthCamera(depth_slot_);
  if (!sim_->TryTakeDepthFrame(depth_slot_, last_depth_frame_,
                               std::chrono::microseconds(opts_.depthcam_wait_us), depth_rgb_buf_,
                               depth_buf_, depth_width_, depth_height_)) {
    return;
  }
  ros_->PublishCameraFrame(depth_rgb_buf_, depth_buf_, depth_width_, depth_height_);
}

void RobotChannel::ReloadToInitial(mjData* data, const SpawnPose& pose) {
  ApplyPose(data, pose);
  // "力矩回到初始设置值"：清空命令缓存（kp/kd/目标全 0、enable=false），
  // 机器人变回未使能状态，等控制侧重新下发命令。
  ros_->ClearCommands();
}

void RobotChannel::ApplyPose(mjData* data, const SpawnPose& pose) {
  if (!ready_ || data == nullptr) {
    return;
  }
  if (!ResetBase(data, pose)) {
    return;
  }
  // 额外写关节角与关节速度：把整机摆成 initial_joint_positions
  //（默认全 0；后续可换成"能稳定坐在地上"的姿态）。
  const auto& qpos_adr = mj_interface_.GetJointQposAdr();
  const auto& dof_adr = mj_interface_.GetJointDofAdr();
  for (std::size_t i = 0; i < qpos_adr.size(); ++i) {
    data->qpos[qpos_adr[i]] =
        i < opts_.initial_joint_positions.size() ? opts_.initial_joint_positions[i] : 0.0;
    data->qvel[dof_adr[i]] = 0.0;
  }
  ForcePublishNextTick();
}

bool RobotChannel::ResetBase(mjData* data, const SpawnPose& pose) {
  if (!ready_ || data == nullptr) {
    return false;
  }
  const int base_qpos = mj_interface_.GetBaseQposAdr();
  const int base_dof = mj_interface_.GetBaseDofAdr();
  if (base_qpos < 0 || base_dof < 0) {
    RCLCPP_WARN(ros_->get_logger(), "基座 free joint 地址无效，位置更新被跳过");
    return false;
  }

  double quat[4];
  YawToQuat(pose.yaw, quat);
  data->qpos[base_qpos + 0] = pose.x;
  data->qpos[base_qpos + 1] = pose.y;
  data->qpos[base_qpos + 2] = pose.z;
  for (int i = 0; i < 4; ++i) {
    data->qpos[base_qpos + 3 + i] = quat[i];
  }
  // 只清基座速度：关节速度/关节角保持原样（/reset_* 的语义）。
  for (int i = 0; i < 6; ++i) {
    data->qvel[base_dof + i] = 0.0;
  }

  // 瞬移后清掉求解器热启动：接触集完全变了，沿用旧热启动会让第一帧接触力尖峰。
  // 它属于求解器状态而非机器人状态，不影响"关节角/速度/力矩保持不变"。
  if (data->qacc_warmstart != nullptr && model_ != nullptr) {
    for (int i = 0; i < model_->nv; ++i) {
      data->qacc_warmstart[i] = 0.0;
    }
  }
  ForcePublishNextTick();
  return true;
}

Eigen::Vector2d RobotChannel::BaseXy(const mjData* data) const {
  const int base_qpos = mj_interface_.GetBaseQposAdr();
  if (data == nullptr || base_qpos < 0) {
    return Eigen::Vector2d::Zero();
  }
  return Eigen::Vector2d(data->qpos[base_qpos + 0], data->qpos[base_qpos + 1]);
}

}  // namespace sim_z01_std_robocup
