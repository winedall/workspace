#include "sim_z01_std_robocup/SimZ01StdRobocup.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <sstream>
#include <stdexcept>

#include "sim_common/SimRuntimeOptions.hpp"

using namespace std::placeholders;

namespace sim_z01_std_robocup {
namespace {

// 空闲/复位时的球门坐标兜底值（模型里读不到时才用）。
constexpr double kGoalOppositeX = 7.02;
constexpr double kGoalSelfX = -7.02;
// 球门"门口"高度：与 football_player 的 target_destination 约定一致。
constexpr double kGoalPublishZ = 0.11;
// 跟踪模式下的窗口相机预设（俯瞰模式用 config 里的 camera_* 参数）。
constexpr double kTrackCameraDistance = 4.0;
constexpr double kTrackCameraAzimuth = 145.0;
constexpr double kTrackCameraElevation = -20.0;
// 容量自检间隔与球状态发布节拍。
constexpr std::int64_t kCapacityCheckIntervalUs = 1000000;

std::string DescribeRequest(uint32_t robot_mask, bool ball_reset) {
  std::ostringstream out;
  out << "accepted:";
  if (robot_mask != 0) {
    out << " robots=[";
    bool first = true;
    for (int i = 0; i < RobotModelSpec::kRobotCount; ++i) {
      if ((robot_mask & (1u << i)) == 0) {
        continue;
      }
      out << (first ? "" : ",") << "robot_0" << (i + 1);
      first = false;
    }
    out << "]";
  }
  if (ball_reset) {
    out << " ball";
  }
  return out.str();
}

}  // namespace

SimZ01StdRobocup::SimZ01StdRobocup(const rclcpp::NodeOptions& options)
    : rng_(std::random_device{}()) {
  // 与生成头做一次一致性自检：前缀拼错/模板改名会在这里立刻暴露。
  ValidateAgainstGeneratedHeader();

  master_node_ = std::make_shared<rclcpp::Node>("sim_z01_std_robocup", options);
  DeclareParameters();
  CreateRobotChannels();

  // ---- 全局（世界系）球与球门话题：这些是 6 台机器人共享的客观量 ----
  ball_pose_pub_ = master_node_->create_publisher<geometry_msgs::msg::Point>("/sim/ball_pose", 10);
  ball_twist_pub_ =
      master_node_->create_publisher<geometry_msgs::msg::Vector3>("/sim/ball_twist", 10);
  goal_team_a_pub_ =
      master_node_->create_publisher<geometry_msgs::msg::Point>("/sim/goal_team_a", 10);
  goal_team_b_pub_ =
      master_node_->create_publisher<geometry_msgs::msg::Point>("/sim/goal_team_b", 10);

  set_ball_pose_sub_ = master_node_->create_subscription<geometry_msgs::msg::Point>(
      "/sim/set_ball_pose", 10, std::bind(&SimZ01StdRobocup::HandleSetBallPose, this, _1));
  set_ball_twist_sub_ = master_node_->create_subscription<geometry_msgs::msg::Vector3>(
      "/sim/set_ball_twist", 10, std::bind(&SimZ01StdRobocup::HandleSetBallTwist, this, _1));
  ball_preview_sub_ = master_node_->create_subscription<geometry_msgs::msg::Point>(
      "/sim/ball_preview", 10, std::bind(&SimZ01StdRobocup::HandleBallPreview, this, _1));

  // ---- 重载 / 复位服务：回调只置原子请求位，真正的 qpos 写入在物理线程完成 ----
  reload_robot_srvs_.resize(RobotModelSpec::kRobotCount);
  for (int i = 0; i < RobotModelSpec::kRobotCount; ++i) {
    const std::string name = "/reload_robot_0" + std::to_string(i + 1);
    reload_robot_srvs_[i] = master_node_->create_service<std_srvs::srv::Trigger>(
        name, [this, i](const std_srvs::srv::Trigger::Request::SharedPtr& req,
                        std_srvs::srv::Trigger::Response::SharedPtr res) {
          HandleReloadRobot(i, req, res);
        });
  }
  reload_all_srv_ = master_node_->create_service<std_srvs::srv::Trigger>(
      "/reload_all_robot",
      std::bind(&SimZ01StdRobocup::HandleReloadAll, this, _1, _2));
  // /reset_*：只挪基座（位置 + 朝向 + 基座速度清零），关节角/速度/力矩保持不变。
  reset_robot_srvs_.resize(RobotModelSpec::kRobotCount);
  for (int i = 0; i < RobotModelSpec::kRobotCount; ++i) {
    const std::string name = "/reset_robot_0" + std::to_string(i + 1);
    reset_robot_srvs_[i] = master_node_->create_service<std_srvs::srv::Trigger>(
        name, [this, i](const std_srvs::srv::Trigger::Request::SharedPtr& req,
                        std_srvs::srv::Trigger::Response::SharedPtr res) {
          HandleResetRobot(i, req, res);
        });
  }
  reset_all_srv_ = master_node_->create_service<std_srvs::srv::Trigger>(
      "/reset_all_robot", std::bind(&SimZ01StdRobocup::HandleResetAll, this, _1, _2));
  reset_ball_srv_ = master_node_->create_service<std_srvs::srv::Trigger>(
      "/reset_ball", std::bind(&SimZ01StdRobocup::HandleResetBall, this, _1, _2));

  // 球门位置是静态量，5Hz 重发即可（晚启动的订阅者也能拿到）。
  goal_pub_timer_ = master_node_->create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&SimZ01StdRobocup::PublishGoals, this));

  // ---- 7 个节点共用一个 executor 线程 ----
  executor_ = std::make_unique<SimRosExecutor>([this]() {
    if (sim_ != nullptr) {
      sim_->exitrequest.store(1);  // 通知 RenderLoop 退出，避免 launch 只能 SIGKILL
    }
  });
  executor_->AddNode(master_node_);
  for (auto& robot : robots_) {
    executor_->AddNode(robot->node());
  }
}

SimZ01StdRobocup::~SimZ01StdRobocup() {
  // 先停 spin 线程，保证后续成员析构时没有回调在访问它们。
  if (executor_) {
    executor_->Stop();
  }
}

void SimZ01StdRobocup::DeclareParameters() {
  // xml_path 由 launch 指向 share/mj_sim/description/Z01_std_robocup/xml/ 下的模型。
  xml_path = master_node_->declare_parameter<std::string>("xml_path", "");
  master_node_->declare_parameter<std::string>("record_folder", "");
  const bool enable_control = master_node_->declare_parameter<bool>("enable_control", true);
  state_publish_period_us_ =
      master_node_->declare_parameter<std::int64_t>("state_publish_period_us", 5000);
  // 只声明不取值：实际值由 CreateRobotChannels 通过 parameter_overrides 注入机器人节点。
  master_node_->declare_parameter<std::int64_t>("driver_status_period_ms", 200);
  base_z_ = master_node_->declare_parameter<double>("base_z", 1.0);
  const auto ball_spawn = master_node_->declare_parameter<std::vector<double>>(
      "ball_spawn", std::vector<double>{0.0, 0.0, 0.11});
  if (ball_spawn.size() != 3) {
    throw std::invalid_argument("ball_spawn 必须是 [x, y, z] 三个值");
  }
  std::copy(ball_spawn.begin(), ball_spawn.end(), ball_spawn_);
  camera_period_us_list_ = master_node_->declare_parameter<std::vector<int64_t>>(
      "camera_period_us_list",
      std::vector<int64_t>(RobotModelSpec::kRobotCount, static_cast<int64_t>(33333)));
  depthcam_wait_us_ = master_node_->declare_parameter<std::int64_t>("depthcam_wait_us", 0);
  track_robot_index_ = master_node_->declare_parameter<int>("track_robot_index", 0);
  camera_distance_ = master_node_->declare_parameter<double>("camera_distance", 16.0);
  camera_azimuth_ = master_node_->declare_parameter<double>("camera_azimuth", 90.0);
  camera_elevation_ = master_node_->declare_parameter<double>("camera_elevation", -35.0);
  camera_lookat_z_ = master_node_->declare_parameter<double>("camera_lookat_z", 0.5);
  sideline_margin_ = master_node_->declare_parameter<double>("sideline_margin", 0.7);
  const auto team_a = master_node_->declare_parameter<std::vector<double>>(
      "team_a_x_range", std::vector<double>{0.5, 6.5});
  const auto team_b = master_node_->declare_parameter<std::vector<double>>(
      "team_b_x_range", std::vector<double>{-6.5, -0.5});
  min_teammate_distance_ =
      master_node_->declare_parameter<double>("min_teammate_distance", 1.0);
  spawn_max_attempts_ = master_node_->declare_parameter<int>("spawn_max_attempts", 50);
  initial_joint_positions_ = master_node_->declare_parameter<std::vector<double>>(
      "initial_joint_positions", std::vector<double>{});

  // 窗口/物理节奏的分组参数：viewer.* 只影响界面显示，dynamics.* 只影响物理推进节奏。
  // 参数名与默认值统一定义在 sim_common/SimRuntimeOptions.hpp（与单机 SimNodeBase 共用），
  // 实际写入 sim_ 由 MujocoSimBase::ApplyStartupOptions() 在 RenderLoop() 之前完成。
  viewer_options_ = sim_common::DeclareViewerOptions(master_node_.get());
  dynamics_options_ = sim_common::DeclareDynamicsOptions(master_node_.get());
  // 误差注入：抽样在每台机器人的 RobotChannel::LoadModel() 里做（那里才知道关节表
  // 与 timestep），这里只读取参数并注入给 6 台。
  error_options_ = sim_common::DeclareErrorOptions(master_node_.get());

  const auto read_range = [](const std::vector<double>& values, const char* name) {
    if (values.size() != 2) {
      throw std::invalid_argument(std::string(name) + " 必须是 [min, max] 两个值");
    }
    return std::array<double, 2>{values[0], values[1]};
  };
  const auto range_a = read_range(team_a, "team_a_x_range");
  const auto range_b = read_range(team_b, "team_b_x_range");
  std::copy(range_a.begin(), range_a.end(), team_a_x_range_);
  std::copy(range_b.begin(), range_b.end(), team_b_x_range_);
  if (camera_period_us_list_.size() != static_cast<std::size_t>(RobotModelSpec::kRobotCount)) {
    throw std::invalid_argument("camera_period_us_list 必须给 6 个值（robot_01..robot_06）");
  }
  if (!initial_joint_positions_.empty() &&
      initial_joint_positions_.size() != static_cast<std::size_t>(RobotModelSpec::kJointCount)) {
    throw std::invalid_argument("initial_joint_positions 要么留空（全 0），要么给 29 个值");
  }

  RCLCPP_INFO(master_node_->get_logger(), "xml_path = %s", xml_path.c_str());
  for (int i = 0; i < RobotModelSpec::kRobotCount; ++i) {
    RCLCPP_INFO(master_node_->get_logger(), "  robot_0%d: /robot_0%d camera_period_us=%ld",
                i + 1, i + 1, static_cast<long>(camera_period_us_list_[i]));
  }
  RCLCPP_INFO(master_node_->get_logger(), "enable_control=%d track_robot_index=%d", enable_control,
              track_robot_index_);
}

void SimZ01StdRobocup::CreateRobotChannels() {
  const bool enable_control = master_node_->get_parameter("enable_control").as_bool();
  const std::int64_t driver_status_period_ms =
      master_node_->get_parameter("driver_status_period_ms").as_int();
  for (int i = 0; i < RobotModelSpec::kRobotCount; ++i) {
    const RobotModelSpec spec = MakeRobotModelSpec(i);

    RobotChannelOptions opts;
    opts.ros.node_name = spec.RosNodeName();
    opts.ros.topic_prefix = spec.ros_prefix;
    opts.ros.control_joint_names = ControlJointNames();
    opts.ros.joint_states_layout = PublishedJointStatesLayout();
    opts.ros.driver_status_message =
        "mujoco simulation online (robocup " + spec.mj_prefix.substr(0, spec.mj_prefix.size() - 1) +
        ")";
    opts.ros.camera_frame_id = spec.mj_prefix + "depth_cam_optical_frame";
    opts.mj_config.joint_names = spec.mj_joint_names;
    opts.mj_config.actuator_names = spec.mj_actuator_names;
    opts.mj_config.base_name = spec.mj_base_name;
    opts.mj_config.orientation_sensor_name = spec.mj_orientation_sensor;
    opts.mj_config.gyro_sensor_name = spec.mj_gyro_sensor;
    opts.mj_config.acc_sensor_name = spec.mj_acc_sensor;
    opts.mj_config.camera_names = {spec.mj_camera_name};
    opts.initial_joint_positions = initial_joint_positions_;
    opts.state_publish_period_us = state_publish_period_us_;
    opts.camera_period_us = camera_period_us_list_[i];
    opts.depthcam_wait_us = depthcam_wait_us_;
    opts.errors = error_options_;

    // 机器人节点的参数用 parameter_overrides 注入：它们不在 YAML 的
    // sim_z01_std_robocup 段里，避免"段名写错 -> 参数静默回默认值"。
    rclcpp::NodeOptions node_options;
    // 关闭全局参数/remap：launch 命令行里的 __node:=sim_z01_std_robocup 会作用于
    // 进程内所有节点，不关掉的话 6 个机器人节点会被改成同一个名字（话题前缀仍由
    // topic_prefix 决定，所以功能不受影响，但节点名会重名、日志与 node list 混乱）。
    node_options.use_global_arguments(false);
    node_options.parameter_overrides({
        {"enable_control", enable_control},
        {"driver_status_period_ms", driver_status_period_ms},
        {"camera_period_us", camera_period_us_list_[i]},
    });
    robots_[i] = std::make_unique<RobotChannel>(spec, opts, node_options);
  }
}

void SimZ01StdRobocup::Start() {
  if (executor_) {
    executor_->Start();
  }
}

// ---------------------------------------------------------------- MuJoCo 生命周期

void SimZ01StdRobocup::InitRobotState(mjData* d) {
  if (m_ == nullptr || d == nullptr) {
    return;
  }
  // 名字映射 + 场景查表都只需要 m_，在写 qpos 之前完成。
  if (!LookupSceneObjects()) {
    return;
  }
  bool ok = true;
  for (auto& robot : robots_) {
    ok = robot->LoadModel(m_) && ok;
  }
  if (!ok) {
    RCLCPP_ERROR(master_node_->get_logger(),
                 "机器人名字映射失败：机器人将不受控（检查 robot_0X_ 前缀与模型是否一致）");
    return;
  }

  // 6 台按出生点规则随机摆位：两条边线外，同队三台互距 >= min_teammate_distance。
  std::vector<Eigen::Vector2d> team_poses[2];
  for (int i = 0; i < RobotModelSpec::kRobotCount; ++i) {
    const int team = robots_[i]->team() - 1;
    const SpawnPose pose = SampleSpawn(i, team_poses[team]);
    robots_[i]->ApplyPose(d, pose);
    team_poses[team].emplace_back(pose.x, pose.y);
    RCLCPP_INFO(master_node_->get_logger(), "robot_0%d spawn: x=%.2f y=%.2f yaw=%.2f", i + 1,
                pose.x, pose.y, pose.yaw);
  }
  // 球放到场心（模型默认也是场心，这里再确保一次）。
  ApplyBallPose(d, ball_spawn_);
  ApplyBallTwist(d, nullptr);
}

void SimZ01StdRobocup::InitInterface() {
  // InitRobotState 已完成名字映射与查表；这里只标记接口可用。
  interface_ready_ = ball_body_id_ >= 0;
  if (!interface_ready_) {
    RCLCPP_ERROR(master_node_->get_logger(), "接口初始化失败：球/球门查表不完整");
  }
}

void SimZ01StdRobocup::InitCamera() {
  if (sim_ == nullptr || m_ == nullptr) {
    return;
  }
  for (auto& robot : robots_) {
    if (!robot->ConfigureCamera(m_, sim_.get())) {
      RCLCPP_ERROR(master_node_->get_logger(), "robot_0%d 深度相机配置失败", robot->index() + 1);
    }
  }
  {
    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
    sim_->SetDepthCameraCount(RobotModelSpec::kRobotCount);
    sim_->SetDepthPreviewSlot(-1);  // 6 路平铺预览没意义，关掉常驻预览
    if (track_robot_index_ >= 1 && track_robot_index_ <= RobotModelSpec::kRobotCount) {
      // 跟踪指定机器人（窗口 UI 里仍可手动改视角）。
      sim_->cam.type = mjCAMERA_TRACKING;
      sim_->cam.fixedcamid = -1;
      sim_->cam.trackbodyid = robots_[track_robot_index_ - 1]->base_body_id();
      sim_->cam.distance = kTrackCameraDistance;
      sim_->cam.azimuth = kTrackCameraAzimuth;
      sim_->cam.elevation = kTrackCameraElevation;
      sim_->camera = 1;
      RCLCPP_INFO(master_node_->get_logger(), "窗口相机跟踪 robot_0%d", track_robot_index_);
    } else {
      // 默认自由俯瞰全场：机位固定在场心上方，看向场地中心。
      sim_->cam.type = mjCAMERA_FREE;
      sim_->cam.fixedcamid = -1;
      sim_->cam.trackbodyid = 0;
      sim_->cam.lookat[0] = 0.0;
      sim_->cam.lookat[1] = 0.0;
      sim_->cam.lookat[2] = camera_lookat_z_;
      sim_->cam.distance = camera_distance_;
      sim_->cam.azimuth = camera_azimuth_;
      sim_->cam.elevation = camera_elevation_;
      sim_->camera = 0;
      RCLCPP_INFO(master_node_->get_logger(),
                  "窗口相机自由俯瞰全场（distance=%.1f azimuth=%.1f elevation=%.1f）",
                  camera_distance_, camera_azimuth_, camera_elevation_);
    }
  }
}

void SimZ01StdRobocup::ComputeControl(const mjModel* m, mjData* d) {
  if (!interface_ready_ || d == nullptr) {
    return;
  }
  for (auto& robot : robots_) {
    robot->ComputeControl(m, d);
  }
}

void SimZ01StdRobocup::PostComputeControl(const mjModel* m, mjData* d) {
  (void)m;
  if (d == nullptr) {
    return;
  }
  // 每台机器人只改写自己那 29 个执行器，互不干扰。
  for (auto& robot : robots_) {
    robot->ApplyTorqueDelay(d);
  }
}

void SimZ01StdRobocup::ExchangeMessages(double sync_sim) {
  if (!interface_ready_ || d_ == nullptr) {
    return;
  }
  // 复位/球操作在物理线程执行（ROS 回调只置位）：趁步进之前把状态改好。
  ApplyPendingRequests(d_);

  const std::int64_t time_us = static_cast<std::int64_t>(d_->time * 1.0e6);
  const bool time_reset = (d_->time - sync_sim) <= 1.0e-4;

  PublishTick(d_, time_us, time_reset);
  for (auto& robot : robots_) {
    robot->PublishTick(d_, time_us, time_reset);
    robot->CameraTick(time_us, time_reset);
  }
  CheckCapacity(d_);
}

void SimZ01StdRobocup::Cleanup() { interface_ready_ = false; }

// ---------------------------------------------------------------- 场景查表

bool SimZ01StdRobocup::LookupSceneObjects() {
  ball_body_id_ = mj_name2id(m_, mjOBJ_BODY, "soccer_ball");
  if (ball_body_id_ < 0) {
    RCLCPP_ERROR(master_node_->get_logger(), "模型里找不到 soccer_ball");
    return false;
  }
  const int ball_joint = m_->body_jntadr[ball_body_id_];
  ball_qpos_adr_ = m_->jnt_qposadr[ball_joint];
  ball_dof_adr_ = m_->jnt_dofadr[ball_joint];

  const int preview_body = mj_name2id(m_, mjOBJ_BODY, "ball_preview");
  ball_preview_mocap_id_ = preview_body >= 0 ? m_->body_mocapid[preview_body] : -1;

  // 球门位置从模型读出（不硬编码）：team A 守 +x 球门、攻 -x 球门；team B 相反。
  const auto goal_x = [this](const char* name, double fallback) {
    const int id = mj_name2id(m_, mjOBJ_BODY, name);
    if (id < 0) {
      RCLCPP_WARN(master_node_->get_logger(), "球门 body %s 不存在，用兜底值 %.2f", name, fallback);
      return fallback;
    }
    return m_->body_pos[3 * id];
  };
  goal_world_pos_[0][0] = goal_x("goal_self_post_l", kGoalSelfX);   // team A 进攻球门
  goal_world_pos_[1][0] = goal_x("goal_oppo_group", kGoalOppositeX);  // team B 进攻球门
  for (auto& goal : goal_world_pos_) {
    goal[1] = 0.0;
    goal[2] = kGoalPublishZ;
  }
  RCLCPP_INFO(master_node_->get_logger(),
              "场景就绪: ball qpos=%d dof=%d, goal_team_a x=%.2f, goal_team_b x=%.2f",
              ball_qpos_adr_, ball_dof_adr_, goal_world_pos_[0][0], goal_world_pos_[1][0]);
  return true;
}

// ---------------------------------------------------------------- 复位

SpawnPose SimZ01StdRobocup::SampleSpawn(
    int index, const std::vector<Eigen::Vector2d>& teammates) {
  const bool team_a = robots_[index]->team() == 1;
  const double y = (team_a ? 1.0 : -1.0) * (kSidelineY + sideline_margin_);
  const double* range = team_a ? team_a_x_range_ : team_b_x_range_;
  std::uniform_real_distribution<double> dist(range[0], range[1]);

  SpawnPose pose;
  pose.y = y;
  pose.z = base_z_;
  // 朝向场内：+y 边线外朝 -y（yaw=-90°），-y 边线外朝 +y（yaw=+90°）。
  pose.yaw = team_a ? -M_PI_2 : M_PI_2;

  for (int attempt = 0; attempt < spawn_max_attempts_; ++attempt) {
    pose.x = dist(rng_);
    const bool too_close = std::any_of(
        teammates.begin(), teammates.end(), [&](const Eigen::Vector2d& other) {
          return std::hypot(pose.x - other.x(), y - other.y()) < min_teammate_distance_;
        });
    if (!too_close) {
      return pose;
    }
  }
  RCLCPP_WARN(master_node_->get_logger(),
              "robot_0%d 出生点重采样 %d 次仍不满足 %.2fm 间距，放宽约束", index + 1,
              spawn_max_attempts_, min_teammate_distance_);
  return pose;
}

void SimZ01StdRobocup::ApplyPendingRequests(mjData* d) {
  if (d == nullptr) {
    return;
  }
  bool changed = false;

  const std::uint32_t reload_mask = reload_robot_mask_.exchange(0, std::memory_order_acq_rel);
  const std::uint32_t reset_mask = reset_robot_mask_.exchange(0, std::memory_order_acq_rel);
  // 两种请求都要"摆到边线"，因此按并集统一采样一次出生点，再按各自的语义落位。
  const std::uint32_t mask = reload_mask | reset_mask;
  if (mask != 0) {
    // 同队内按序号顺序采样：后采样的机器人避开"本轮已摆好的同队位置"，
    // 未被复位的同队机器人则用当前位置作为约束。
    std::vector<Eigen::Vector2d> team_poses[2];
    for (int i = 0; i < RobotModelSpec::kRobotCount; ++i) {
      if ((mask & (1u << i)) == 0) {
        team_poses[robots_[i]->team() - 1].push_back(robots_[i]->BaseXy(d));
      }
    }
    for (int i = 0; i < RobotModelSpec::kRobotCount; ++i) {
      if ((mask & (1u << i)) == 0) {
        continue;
      }
      const int team = robots_[i]->team() - 1;
      const SpawnPose pose = SampleSpawn(i, team_poses[team]);
      const bool full_reload = (reload_mask & (1u << i)) != 0;
      if (full_reload) {
        // /reload_*：基座位姿 + 关节角回到初始 + 速度清零 + 命令缓存清空（力矩归零）。
        robots_[i]->ReloadToInitial(d, pose);
        RCLCPP_INFO(master_node_->get_logger(),
                    "robot_0%d reload: x=%.2f y=%.2f（关节回到初始位姿、速度清零、力矩清空）", i + 1,
                    pose.x, pose.y);
      } else {
        // /reset_*：只挪基座（位置 + 朝向 + 基座速度清零），关节状态保持原样。
        if (!robots_[i]->ResetBase(d, pose)) {
          continue;
        }
        RCLCPP_INFO(master_node_->get_logger(),
                    "robot_0%d reset: x=%.2f y=%.2f（关节角/速度/力矩保持不变）", i + 1, pose.x,
                    pose.y);
      }
      team_poses[team].emplace_back(pose.x, pose.y);
      changed = true;
    }
  }

  if (reset_ball_pending_.exchange(false, std::memory_order_acq_rel)) {
    ApplyBallPose(d, ball_spawn_);
    ApplyBallTwist(d, nullptr);
    changed = true;
    RCLCPP_INFO(master_node_->get_logger(), "ball reset: (%.2f, %.2f, %.2f)", ball_spawn_[0],
                ball_spawn_[1], ball_spawn_[2]);
  }
  if (ball_pose_pending_.exchange(false, std::memory_order_acq_rel)) {
    const double xyz[3] = {ball_pose_req_[0].load(), ball_pose_req_[1].load(),
                           ball_pose_req_[2].load()};
    ApplyBallPose(d, xyz);
    changed = true;
  }
  if (ball_twist_pending_.exchange(false, std::memory_order_acq_rel)) {
    const double vxyz[3] = {ball_twist_req_[0].load(), ball_twist_req_[1].load(),
                            ball_twist_req_[2].load()};
    ApplyBallTwist(d, vxyz);
    changed = true;
  }
  if (ball_preview_pending_.exchange(false, std::memory_order_acq_rel)) {
    if (ball_preview_mocap_id_ >= 0) {
      const double x = ball_preview_req_[0].load();
      const double y = ball_preview_req_[1].load();
      const double z = ball_preview_req_[2].load();
      d->mocap_pos[3 * ball_preview_mocap_id_ + 0] = x;
      d->mocap_pos[3 * ball_preview_mocap_id_ + 1] = y;
      // z < 0 表示隐藏预览点（放到地面以下，和 football_player 的约定一致）。
      d->mocap_pos[3 * ball_preview_mocap_id_ + 2] =
          z < 0.0 ? -std::abs(z) - 10.0 : z;
      changed = true;
    }
  }

  if (changed) {
    // 位置/速度改完后前向计算一次，保证接触、传感器等派生量立刻自洽。
    mj_forward(m_, d);
  }
}

void SimZ01StdRobocup::ApplyBallPose(mjData* d, const double* xyz) {
  if (ball_qpos_adr_ < 0 || d == nullptr || xyz == nullptr) {
    return;
  }
  // 球半径 0.11：请求高度低于半径时按半径放置，避免把球埋进地面。
  const double z = std::max(xyz[2], 0.11);
  d->qpos[ball_qpos_adr_ + 0] = xyz[0];
  d->qpos[ball_qpos_adr_ + 1] = xyz[1];
  d->qpos[ball_qpos_adr_ + 2] = z;
  d->qpos[ball_qpos_adr_ + 3] = 1.0;
  d->qpos[ball_qpos_adr_ + 4] = 0.0;
  d->qpos[ball_qpos_adr_ + 5] = 0.0;
  d->qpos[ball_qpos_adr_ + 6] = 0.0;
}

void SimZ01StdRobocup::ApplyBallTwist(mjData* d, const double* vxyz) {
  if (ball_dof_adr_ < 0 || d == nullptr) {
    return;
  }
  // 自由关节有 6 个自由度：vxyz 只给世界系线速度（3 个），角速度一律清零。
  for (int i = 0; i < 6; ++i) {
    d->qvel[ball_dof_adr_ + i] = (vxyz != nullptr && i < 3) ? vxyz[i] : 0.0;
  }
}

// ---------------------------------------------------------------- 发布

void SimZ01StdRobocup::PublishTick(const mjData* d, std::int64_t time_us, bool time_reset) {
  if (ball_body_id_ < 0) {
    return;
  }
  // 与机器人通道同样的节拍对齐策略：时间重置/落后过多时重新对齐。
  if (time_reset || time_us + state_publish_period_us_ < next_ball_pub_time_us_) {
    next_ball_pub_time_us_ = time_us;
  }
  if (time_us < next_ball_pub_time_us_) {
    return;
  }
  next_ball_pub_time_us_ += state_publish_period_us_;

  // 球的位置/速度（世界系）：6 台机器人共享的客观量，机器人侧自行换算到基座系。
  geometry_msgs::msg::Point pose;
  pose.x = d->qpos[ball_qpos_adr_ + 0];
  pose.y = d->qpos[ball_qpos_adr_ + 1];
  pose.z = d->qpos[ball_qpos_adr_ + 2];
  ball_pose_pub_->publish(pose);

  geometry_msgs::msg::Vector3 twist;
  twist.x = d->qvel[ball_dof_adr_ + 0];
  twist.y = d->qvel[ball_dof_adr_ + 1];
  twist.z = d->qvel[ball_dof_adr_ + 2];
  ball_twist_pub_->publish(twist);
}

void SimZ01StdRobocup::PublishGoals() {
  geometry_msgs::msg::Point team_a;
  team_a.x = goal_world_pos_[0][0];
  team_a.y = goal_world_pos_[0][1];
  team_a.z = goal_world_pos_[0][2];
  goal_team_a_pub_->publish(team_a);

  geometry_msgs::msg::Point team_b;
  team_b.x = goal_world_pos_[1][0];
  team_b.y = goal_world_pos_[1][1];
  team_b.z = goal_world_pos_[1][2];
  goal_team_b_pub_->publish(team_b);
}

void SimZ01StdRobocup::CheckCapacity(const mjData* d) {
  const std::int64_t time_us = static_cast<std::int64_t>(d->time * 1.0e6);
  if (time_us < next_capacity_check_us_) {
    return;
  }
  next_capacity_check_us_ = time_us + kCapacityCheckIntervalUs;
  const int contact_full = d->warning[mjWARN_CONTACTFULL].number;
  const int constraint_full = d->warning[mjWARN_CNSTRFULL].number;
  if (contact_full != 0 || constraint_full != 0) {
    RCLCPP_WARN(master_node_->get_logger(),
                "模型容量不足：CONTACTFULL=%d CNSTRFULL=%d (ncon=%d nefc=%d) "
                "请提高 XML 的 nconmax/njmax",
                contact_full, constraint_full, d->ncon, d->nefc);
  }
}

// ---------------------------------------------------------------- 服务与订阅回调

std::string SimZ01StdRobocup::StatusMessage(bool reload, uint32_t robot_mask,
                                            bool ball_reset) const {
  std::string message = DescribeRequest(robot_mask, ball_reset);
  if (robot_mask != 0) {
    message += reload ? "（完整重载：基座位姿 + 关节角回到初始 + 速度清零 + 力矩清空）"
                      : "（只挪基座：位置 + 朝向 + 基座速度清零，关节角/速度/力矩保持不变）";
  }
  message += "; 将在下一个物理步生效";
  if (sim_ != nullptr) {
    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
    if (sim_->run == 0) {
      message += "（注意：MuJoCo 窗口当前是暂停状态，恢复运行后才会执行）";
    }
  }
  return message;
}

void SimZ01StdRobocup::HandleReloadRobot(int index,
                                         const std_srvs::srv::Trigger::Request::SharedPtr& req,
                                         std_srvs::srv::Trigger::Response::SharedPtr res) {
  (void)req;
  const uint32_t mask = 1u << index;
  reload_robot_mask_.fetch_or(mask, std::memory_order_acq_rel);
  res->success = true;
  res->message = StatusMessage(true, mask, false);
}

void SimZ01StdRobocup::HandleReloadAll(const std_srvs::srv::Trigger::Request::SharedPtr& req,
                                       std_srvs::srv::Trigger::Response::SharedPtr res) {
  (void)req;
  // 一次性置 6 位：6 台在同一个物理步内一起重载，而不是 6 次独立请求。
  const uint32_t mask = (1u << RobotModelSpec::kRobotCount) - 1u;
  reload_robot_mask_.fetch_or(mask, std::memory_order_acq_rel);
  res->success = true;
  res->message = StatusMessage(true, mask, false);
}

void SimZ01StdRobocup::HandleResetRobot(int index,
                                        const std_srvs::srv::Trigger::Request::SharedPtr& req,
                                        std_srvs::srv::Trigger::Response::SharedPtr res) {
  (void)req;
  const uint32_t mask = 1u << index;
  reset_robot_mask_.fetch_or(mask, std::memory_order_acq_rel);
  res->success = true;
  res->message = StatusMessage(false, mask, false);
}

void SimZ01StdRobocup::HandleResetAll(const std_srvs::srv::Trigger::Request::SharedPtr& req,
                                      std_srvs::srv::Trigger::Response::SharedPtr res) {
  (void)req;
  const uint32_t mask = (1u << RobotModelSpec::kRobotCount) - 1u;
  reset_robot_mask_.fetch_or(mask, std::memory_order_acq_rel);
  res->success = true;
  res->message = StatusMessage(false, mask, false);
}

void SimZ01StdRobocup::HandleResetBall(const std_srvs::srv::Trigger::Request::SharedPtr& req,
                                       std_srvs::srv::Trigger::Response::SharedPtr res) {
  (void)req;
  reset_ball_pending_.store(true, std::memory_order_release);
  res->success = true;
  res->message = StatusMessage(false, 0, true);
}

void SimZ01StdRobocup::HandleSetBallPose(const geometry_msgs::msg::Point::SharedPtr msg) {
  ball_pose_req_[0].store(msg->x, std::memory_order_relaxed);
  ball_pose_req_[1].store(msg->y, std::memory_order_relaxed);
  ball_pose_req_[2].store(msg->z, std::memory_order_relaxed);
  ball_pose_pending_.store(true, std::memory_order_release);
}

void SimZ01StdRobocup::HandleSetBallTwist(const geometry_msgs::msg::Vector3::SharedPtr msg) {
  ball_twist_req_[0].store(msg->x, std::memory_order_relaxed);
  ball_twist_req_[1].store(msg->y, std::memory_order_relaxed);
  ball_twist_req_[2].store(msg->z, std::memory_order_relaxed);
  ball_twist_pending_.store(true, std::memory_order_release);
}

void SimZ01StdRobocup::HandleBallPreview(const geometry_msgs::msg::Point::SharedPtr msg) {
  ball_preview_req_[0].store(msg->x, std::memory_order_relaxed);
  ball_preview_req_[1].store(msg->y, std::memory_order_relaxed);
  ball_preview_req_[2].store(msg->z, std::memory_order_relaxed);
  ball_preview_pending_.store(true, std::memory_order_release);
}

}  // namespace sim_z01_std_robocup
