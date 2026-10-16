#include "sim_common/SimNodeBase.hpp"

#include <chrono>

#include "sim_common/SimRuntimeOptions.hpp"
#include "sim_common/SimStateFill.hpp"
#include "sim_common/quaternion_utils.hpp"

namespace sim_common {

SimNodeBase::SimNodeBase(const SimNodeOptions& opts, const rclcpp::NodeOptions& node_options)
    : MujocoSimBase(),
      SimRosNodeBase(opts.ros, node_options),
      mj_interface_(opts.mj_config),
      q_rel_(Eigen::VectorXd::Zero(
          static_cast<Eigen::Index>(opts.mj_config.joint_names.size()) + 7)),
      dq_rel_(Eigen::VectorXd::Zero(
          static_cast<Eigen::Index>(opts.mj_config.joint_names.size()) + 6)),
      tau_rel_(Eigen::VectorXd::Zero(
          static_cast<Eigen::Index>(opts.mj_config.joint_names.size()) + 6)),
      q_pub_(Eigen::VectorXd::Zero(
          static_cast<Eigen::Index>(opts.mj_config.joint_names.size()) + 7)),
      dq_pub_(Eigen::VectorXd::Zero(
          static_cast<Eigen::Index>(opts.mj_config.joint_names.size()) + 6)),
      tau_pub_(Eigen::VectorXd::Zero(
          static_cast<Eigen::Index>(opts.mj_config.joint_names.size()) + 6)),
      opts_(opts) {
  RCLCPP_INFO(this->get_logger(), "%s node created!", opts.ros.node_name.c_str());
  // xml_path 由 launch 指向 mj_sim 包中的 MuJoCo XML。这样仿真模型资源随包安装，
  // 不依赖源码路径。
  this->declare_parameter<std::string>("xml_path", "");
  xml_path = this->get_parameter("xml_path").as_string();

  // 发布频率与可调物理常量：默认值仅作兜底，实际值由各仿真包的 config YAML 传入。
  this->declare_parameter<double>("base_z", 1.0);
  base_z_ = this->get_parameter("base_z").as_double();
  this->declare_parameter<double>("camera_distance", 3.0);
  camera_distance_ = this->get_parameter("camera_distance").as_double();
  this->declare_parameter<double>("camera_azimuth", 145.0);
  camera_azimuth_ = this->get_parameter("camera_azimuth").as_double();
  this->declare_parameter<double>("camera_elevation", -20.0);
  camera_elevation_ = this->get_parameter("camera_elevation").as_double();
  // 初始窗口相机：0 = 自由，1 = 跟踪基座（默认，与改造前一致），
  // >=2 = 模型里的第 (camera_index - 2) 个固定相机（即窗口里 [ / ] 键切到的那些）。
  this->declare_parameter<int>("camera_index", 1);
  camera_index_ = this->get_parameter("camera_index").as_int();
  this->declare_parameter<int64_t>("state_publish_period_us", 5000);
  state_publish_period_us_ = this->get_parameter("state_publish_period_us").as_int();

  // 窗口/物理节奏的分组参数：viewer.* 只影响界面显示，dynamics.* 只影响物理推进节奏。
  // 参数名与默认值统一定义在 SimRuntimeOptions.hpp；这里只负责读取，实际写入
  // sim_ 由 MujocoSimBase::ApplyStartupOptions() 在 RenderLoop() 之前完成。
  viewer_options_ = sim_common::DeclareViewerOptions(this);
  dynamics_options_ = sim_common::DeclareDynamicsOptions(this);

  // 误差注入（传感器误差 + 执行延迟）。实际抽样在 InitInterface() 里做——那里才
  // 知道模型加载后的关节/执行器表与 timestep。
  error_options_ = sim_common::DeclareErrorOptions(this);

  if (enable_control_) {
    RCLCPP_INFO(this->get_logger(), "MuJoCo simulation control input is enabled.");
  }

  // launch 文件会传 record_folder；CSV 记录功能已移除，仅保留参数声明避免告警。
  this->declare_parameter<std::string>("record_folder", "");
}

SimNodeBase::~SimNodeBase() {
  ros_spin_running_ = false;
  executor_.cancel();
  if (ros_spin_thread_.joinable()) {
    ros_spin_thread_.join();
  }
}

void SimNodeBase::Start() {
  // MuJoCo Run() 会占用主线程，ROS 订阅和定时器需要独立 executor 线程。
  // Start() 在 std::make_shared 之后调用，此时 enable_shared_from_this 已就绪。
  ros_spin_running_ = true;
  ros_spin_thread_ = std::thread(&SimNodeBase::RunRosExecutor, this);
}

void SimNodeBase::RunRosExecutor() {
  executor_.add_node(shared_from_this());
  while (ros_spin_running_ && rclcpp::ok()) {
    executor_.spin_once(std::chrono::milliseconds(1));
  }
  // ROS 2 收到 SIGINT/SIGTERM 后同步通知 MuJoCo GUI/物理循环退出，
  // 否则 RenderLoop 会继续阻塞，launch 最终只能 SIGKILL 仿真进程。
  if (sim_ != nullptr) {
    sim_->exitrequest.store(1);
  }
  executor_.remove_node(shared_from_this());
}

void SimNodeBase::InitRobotState(mjData* d) {
  if (m_ == nullptr || d == nullptr) {
    return;
  }

  // 浮动基座初始位置：z 由 config YAML 的 base_z 配置，让机器人从指定高度开始落地/站立。
  d->qpos[0] = 0.0;
  d->qpos[1] = 0.0;
  d->qpos[2] = base_z_;
  d->qpos[3] = 1.0;
  d->qpos[4] = 0.0;
  d->qpos[5] = 0.0;
  d->qpos[6] = 0.0;

  if (opts_.initial_joint_positions.empty()) {
    // 人形路径：不额外设置关节角，所有 MuJoCo 关节从 0 位开始。
    const std::size_t n = mj_interface_.GetJointCount();
    for (std::size_t i = 7; i < n + 7; ++i) {
      d->qpos[i] = 0.0;
    }
  } else {
    // 狗形路径：按关节名设置初始姿态并清零速度。
    const auto& joint_names = opts_.mj_config.joint_names;
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      const int joint_id = mj_name2id(m_, mjOBJ_JOINT, joint_names[i].c_str());
      if (joint_id < 0) {
        RCLCPP_ERROR(get_logger(), "Initial-state joint not found: %s", joint_names[i].c_str());
        continue;
      }
      d->qpos[m_->jnt_qposadr[joint_id]] = opts_.initial_joint_positions[i];
      d->qvel[m_->jnt_dofadr[joint_id]] = 0.0;
    }
  }
}

void SimNodeBase::InitInterface() {
  // MuJoCo 模型加载完成后，建立关节/执行器/传感器名称到内部索引的映射。
  mj_interface_.LoadMjModel(m_);
  interface_ready_ = mj_interface_.InitDataHandle();
  if (!interface_ready_) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize MuJoCo object mappings for %s",
                 get_name());
    return;
  }

  // 误差注入：模型加载后按 errors.seed 抽一次常值误差。GUI Reload 会重新走这里，
  // 同 seed 抽到的是同一台"个体"（零位偏置等保持不变）。
  error_model_.Configure(error_options_);
  if (error_model_.enabled()) {
    error_model_.Init(mj_interface_.GetJointCount(), mj_interface_.GetMotorIds(), m_->opt.timestep);
    mj_interface_.SetErrorModel(&error_model_);
    // 通信层也要拿到模型：指令延迟施加在 UpdateCommandCache（所有包共用的指令写入点）。
    SetErrorModel(&error_model_);
    RCLCPP_INFO(get_logger(), "误差注入: %s", error_model_.Summary().c_str());
    if (!error_model_.Warning().empty()) {
      RCLCPP_WARN(get_logger(), "%s", error_model_.Warning().c_str());
    }
  } else {
    mj_interface_.SetErrorModel(nullptr);
    SetErrorModel(nullptr);
    if (error_options_.torque_delay_ms > 0.0 || error_options_.drop_probability > 0.0 ||
        error_options_.feedback_hold_ms > 0.0 || error_options_.feedback_delay_ms > 0.0 ||
        error_options_.command_delay_ms > 0.0) {
      RCLCPP_WARN(get_logger(),
                  "errors 下的时序参数已配置但 errors.seed=0（整体关闭），不会生效；"
                  "只加时序（延迟/采样保持）请设 seed>0 并把各误差标准差留 0");
    }
  }
  HandleInterfaceReady();
}

void SimNodeBase::PostComputeControl(const mjModel* m, mjData* d) {
  (void)m;
  // 把 N 个仿真步之前算出的力矩施加到当前步（等价于闭环整体滞后 N 步）。
  error_model_.ApplyTorqueDelay(d);
}

void SimNodeBase::InitCamera() {
  if (m_ == nullptr || sim_ == nullptr) {
    return;
  }
  // 基座 body id 用 MjInterface 缓存的那份：InitInterface() 必先于本函数执行，
  // 与下面深度相机走 GetCamIds() 是同一套名字映射。InitDataHandle() 失败时
  // 返回 -1，等同于"XML 里查不到基座"——跳过相机配置。
  const int base_id = mj_interface_.GetBaseBodyId();
  if (base_id < 0) {
    return;
  }
  InitTrackingCamera(base_id);
  InitDepthCamera();
}

void SimNodeBase::InitTrackingCamera(int base_id) {
  const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
  // 距离/方位/俯角对自由相机与跟踪相机都有效；固定相机用自己的 <camera> 定义。
  sim_->cam.distance = camera_distance_;
  sim_->cam.azimuth = camera_azimuth_;
  sim_->cam.elevation = camera_elevation_;

  if (camera_index_ >= 2) {
    // 固定相机：Simulate 约定 camera = 2 + fixedcamid（与窗口里 [ / ] 键一致）。
    sim_->cam.type = mjCAMERA_FIXED;
    sim_->cam.fixedcamid = camera_index_ - 2;
    sim_->cam.trackbodyid = 0;
    sim_->camera = camera_index_;
    return;
  }
  if (camera_index_ == 0) {
    sim_->cam.type = mjCAMERA_FREE;
    sim_->cam.fixedcamid = -1;
    sim_->cam.trackbodyid = 0;
    sim_->camera = 0;
    return;
  }
  // 默认（camera_index = 1）：跟踪机器人基座，便于启动后直接观察整机姿态和步态。
  sim_->cam.type = mjCAMERA_TRACKING;
  sim_->cam.trackbodyid = base_id;
  sim_->cam.fixedcamid = -1;
  sim_->camera = 1;
}

void SimNodeBase::InitDepthCamera() {
  if (mj_interface_.GetCamIds().empty()) {
    return;
  }
  const int cam_id = mj_interface_.GetCamIds()[0];
  const int width = m_->cam_resolution[cam_id * 2];
  const int height = m_->cam_resolution[cam_id * 2 + 1];
  {
    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
    // 单机器人仿真固定使用深度相机槽位 0（同时作为窗口预览槽位，
    // 与改造前"始终渲染一路预览"的行为一致）。
    if (!sim_->ConfigureDepthCamera(0, cam_id, width, height)) {
      RCLCPP_ERROR(get_logger(), "Failed to configure depth camera slot for %s", get_name());
      return;
    }
    depth_camera_slot_ = 0;
  }
  if (camera_period_us_ <= 0) {
    // 不发布图像话题：仅保留窗口预览（与改造前一致）。
    return;
  }
  PrepareCameraInfo();
}

void SimNodeBase::Cleanup() { interface_ready_ = false; }

void SimNodeBase::ExchangeMessages(double sync_sim) {
  if (!interface_ready_ || d_ == nullptr) {
    return;
  }

  // 从 MuJoCo 读取当前状态，转换为项目内部数组形式。
  mj_interface_.ReadState(d_, q_rel_, dq_rel_, tau_rel_, acc_);

  const int64_t time_us = static_cast<int64_t>(d_->time * 1.0e6);
  // 仿真时间同步重置或节拍落后过多时，重新对齐发布节拍，
  // 避免时间回退后长时间不发布，或落后后的连续补发。
  if ((d_->time - sync_sim) <= 1.0e-4 ||
      time_us + state_publish_period_us_ < next_pub_time_us_) {
    next_pub_time_us_ = time_us;
    next_cam_pub_time_us_ = time_us;
  }

  if (time_us >= next_pub_time_us_) {
    // 按 state_publish_period_us 周期发布仿真状态（默认 5000 us ≈ 200 Hz）。
    next_pub_time_us_ += state_publish_period_us_;

    FillJointStates();
    PublishJointStates();

    FillImu();
    PublishImu();
    PublishExtra(d_);
  }

  if (camera_period_us_ > 0 && time_us >= next_cam_pub_time_us_) {
    next_cam_pub_time_us_ += camera_period_us_;
    PublishDepthCamera();
  }
}

void SimNodeBase::FillJointStates() {
  // 反馈上行的采样保持 / 纯延迟（errors.feedback_*）：只改发布出去的这份快照，
  // q_rel_/dq_rel_/tau_rel_ 保持新鲜供 PD 回路使用（见 SimErrorModel.hpp 的说明）。
  error_model_.ApplyFeedbackDelay(d_->time, q_rel_, dq_rel_, tau_rel_, q_pub_, dq_pub_, tau_pub_);

  // 填充逻辑与多机器人仿真共用（sim_common/SimStateFill.hpp）。
  FillJointStatesMessage(sim_joint_commands_, joint_states_layout_,
                         joint_enabled_requires_enable_control_, enable_control_, q_pub_, dq_pub_,
                         tau_pub_, joint_states_msg_);
}

void SimNodeBase::PublishDepthCamera() {
  // 请求渲染线程捕获一帧深度相机图像，短暂等待完成后发布。
  // 超时时沿用上一次取回的缓冲（与改造前"读到旧数据就发布"的行为一致）。
  if (sim_ == nullptr || depth_camera_slot_ < 0) {
    return;
  }
  sim_->RequestDepthCamera(depth_camera_slot_);
  sim_->TryTakeDepthFrame(depth_camera_slot_, last_depth_frame_, std::chrono::microseconds(500),
                          depth_rgb_buf_, depth_buf_, depth_width_, depth_height_);

  if (!depth_buf_.empty()) {
    PublishImages(depth_rgb_buf_, depth_buf_, depth_width_, depth_height_);
  }
}

void SimNodeBase::FillImu() {
  // 发布动作由 SimRosNodeBase::PublishImu() 完成，这里只注入物理量。
  FillImuMessage(q_rel_, dq_rel_, acc_, imu_data_);
}

void SimNodeBase::PrepareCameraInfo() {
  // 使用第一个相机（depth_cam）；填充逻辑与多机器人仿真共用。
  if (mj_interface_.GetCamIds().empty()) {
    return;
  }
  FillCameraInfoMessage(*m_, mj_interface_.GetCamIds()[0], camera_frame_id_, camera_info_msg_);
  camera_info_ready_ = true;
}

}  // namespace sim_common
