// SimZ01StdFootballPlayer.cpp
#include <cstring>

#include "sim_z01_std_football_player/SimZ01StdFootballPlayer.h"

using namespace std::placeholders;

#include "mj_sim/MotorSpec.h"
#include "robot_header/Z01_std_football_player.h"
#include "sim_common/control_math.hpp"

using namespace Z01_std_football_player;

namespace
{

// z01_std_football_player 的电机参数：并联腿踝关节使用 4310Parallel 电机，
// 其余关节与 z01_std_add_head 保持一致。
// 每个关节对应的电机规格，按 kActuatorsName / kJointsName 顺序排列。
// Z01_std 量产版电机 TN 曲线数据暂缺，先用无限制电机占位（力矩指令不限幅、
// TN 曲线不削减），后续拿到量产电机数据后再替换为实际型号。
// 每个关节对应的电机规格，按 kActuatorsName / kJointsName 顺序排列。
const std::array<motor_spec::MotorSpec, 29> kJointMotorSpecs = {
    motor_spec::kMotorEncos2806,
    motor_spec::kMotorEncos2806,
    motor_spec::kMotorTienKungHRA58P, 
    motor_spec::kMotorTienKungHRA58P,
    motor_spec::kMotorTienKungHRA58P,  
    motor_spec::kMotorTienKungHRA58P, 
    motor_spec::kMotorTienKungHRA58P, 
    motor_spec::kMotorEncos2806, 
    motor_spec::kMotorEncos2806,  
    motor_spec::kMotorTienKungHRA58P,  
    motor_spec::kMotorTienKungHRA58P,  
    motor_spec::kMotorTienKungHRA58P,  
    motor_spec::kMotorTienKungHRA58P,  
    motor_spec::kMotorTienKungHRA58P,  
    motor_spec::kMotorEncos2806,  
    motor_spec::kMotorEncos2806,  
    motor_spec::kMotorTienKungHRA88P,  // 16: M_waist_yaw
    motor_spec::kMotorTienKungHRA88P,  // 17: M_hip_l_pitch
    motor_spec::kMotorTienKungHRA88P,  // 18: M_hip_l_roll
    motor_spec::kMotorTienKungHRA88P,  // 19: M_hip_l_yaw
    motor_spec::kMotorTienKungHRA88P,  // 20: M_knee_l_pitch
    motor_spec::kMotorTienKungHRA58P,  // 21: M_ankle_l_pitch
    motor_spec::kMotorTienKungHRA58P,  // 22: M_ankle_l_roll
    motor_spec::kMotorTienKungHRA88P,  // 23: M_hip_r_pitch
    motor_spec::kMotorTienKungHRA88P,  // 24: M_hip_r_roll
    motor_spec::kMotorTienKungHRA88P,  // 25: M_hip_r_yaw
    motor_spec::kMotorTienKungHRA88P,  // 26: M_knee_r_pitch
    motor_spec::kMotorTienKungHRA58P,  // 27: M_ankle_r_pitch
    motor_spec::kMotorTienKungHRA58P,  // 28: M_ankle_r_roll
};


// 本模型的构造参数。控制关节名以 head_yaw 开头（与 add_head 的
// head_pitch 开头相反）；并联腿模型对控制系统的接口仍使用
// left_ankle_pitch/left_ankle_roll 统一命名（MuJoCo 内部为 *_active）。
sim_common::SimNodeOptions MakeSimNodeOptions()
{
  sim_common::SimNodeOptions opts;
  opts.ros.node_name = "sim_z01_std_football_player";
  opts.ros.control_joint_names = {
      "head_yaw",            "head_pitch",          "left_shoulder_pitch", "left_shoulder_roll",
      "left_shoulder_yaw",   "left_elbow",          "left_wrist_yaw",      "left_wrist_pitch",
      "left_wrist_roll",     "right_shoulder_pitch","right_shoulder_roll", "right_shoulder_yaw",
      "right_elbow",         "right_wrist_yaw",     "right_wrist_pitch",   "right_wrist_roll",
      "waist_yaw",           "left_hip_pitch",      "left_hip_roll",       "left_hip_yaw",
      "left_knee",           "left_ankle_pitch",    "left_ankle_roll",     "right_hip_pitch",
      "right_hip_roll",      "right_hip_yaw",       "right_knee",          "right_ankle_pitch",
      "right_ankle_roll",
  };
  // 对外发布的 JointStates 要保持全系统 31 关节视图。
  // 本模型没有 waist_pitch/waist_roll 执行器：两者作为零状态占位，
  // 按真机顺序排在 waist_yaw 之前。
  opts.ros.joint_states_layout = sim_common::BuildJointStatesLayout(
      opts.ros.control_joint_names,
      {sim_common::PlaceholdersBefore("waist_yaw", {"waist_pitch", "waist_roll"})});
  opts.ros.driver_status_message = "mujoco simulation online (football player)";

  opts.mj_config.joint_names.assign(kJointsName.begin(), kJointsName.end());
  opts.mj_config.actuator_names.assign(kActuatorsName.begin(), kActuatorsName.end());
  opts.mj_config.base_name = kBaseName;
  opts.mj_config.orientation_sensor_name = kOrientationSensorName;
  opts.mj_config.vel_sensor_name = kVelSensorName;
  opts.mj_config.gyro_sensor_name = kGyroSensorName;
  opts.mj_config.acc_sensor_name = kAccSensorName;
  opts.mj_config.camera_names.assign(kCameraNames.begin(), kCameraNames.end());
  return opts;
}

}  // namespace

SimZ01StdFootballPlayer::SimZ01StdFootballPlayer(const rclcpp::NodeOptions& options)
    : sim_common::SimNodeBase(MakeSimNodeOptions(), options) {
  // 足球目标点发布（基座标系下的球和球门位置）
  target_point_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/robot/perception/target_point", 10);
  target_destination_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/robot/perception/target_destination", 10);

  // 订阅键盘/外部设置球位置与速度
  set_ball_pos_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/robot/teleop/set_ball_position", 10,
      std::bind(&SimZ01StdFootballPlayer::HandleBallPosition, this, _1));
  set_ball_vel_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/robot/teleop/set_ball_velocity", 10,
      std::bind(&SimZ01StdFootballPlayer::HandleBallVelocity, this, _1));

  // 订阅足球放置预览点
  ball_preview_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/robot/teleop/ball_preview", 10,
      std::bind(&SimZ01StdFootballPlayer::HandleBallPreview, this, _1));
  // 订阅足球模式切换（键盘 m，独立于踢球模型）
  soccer_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/robot/sim/soccer_mode", 10,
      std::bind(&SimZ01StdFootballPlayer::HandleSoccerMode, this, _1));
}

void SimZ01StdFootballPlayer::HandleInterfaceReady() {
  // 查表：足球 body 和 joint 的 MuJoCo ID
  soccer_ball_body_id_ = mj_name2id(m_, mjOBJ_BODY, "soccer_ball");
  if (soccer_ball_body_id_ >= 0) {
    int jnt_adr = m_->body_jntadr[soccer_ball_body_id_];
    soccer_ball_joint_qposadr_ = m_->jnt_qposadr[jnt_adr];
    RCLCPP_INFO(this->get_logger(), "soccer_ball body found, qposadr=%d", soccer_ball_joint_qposadr_);
  } else {
    RCLCPP_WARN(this->get_logger(), "soccer_ball body not found in MuJoCo model");
  }

  // 收集球门和球的几何信息，用于足球模式切换
  soccer_geoms_.clear();
  CollectBodyGeoms("goal_left_post", soccer_geoms_);
  CollectBodyGeoms("goal_right_post", soccer_geoms_);
  CollectBodyGeoms("goal_crossbar", soccer_geoms_);
  CollectBodyGeoms("goal_net", soccer_geoms_);
  // 本方球门（soccer_field.xml 的 goal_self_*）同样随模式显隐，避免非
  // 足球模式下残留一个球门在场景里。
  CollectBodyGeoms("goal_self_post_l", soccer_geoms_);
  CollectBodyGeoms("goal_self_post_r", soccer_geoms_);
  CollectBodyGeoms("goal_self_crossbar", soccer_geoms_);
  CollectBodyGeoms("goal_self_net", soccer_geoms_);

  // 收集场地元素（草坪/白线/中圈/点球点等，按 soccer_field.xml 的命名
  // 前缀分组），随足球模式整体显示/隐藏。
  field_geoms_.clear();
  CollectGeomsByPrefix("field_", field_geoms_);
  CollectGeomsByPrefix("line_", field_geoms_);
  CollectGeomsByPrefix("center_circle", field_geoms_);
  CollectGeomsByPrefix("pen_", field_geoms_);
  CollectGeomsByPrefix("goal_area_", field_geoms_);
  CollectGeomsByPrefix("penalty_spot", field_geoms_);

  // 对方球门 body 位置（发布 target_destination 用，替代硬编码坐标）。
  goal_body_id_ = mj_name2id(m_, mjOBJ_BODY, "goal_oppo_group");
  if (goal_body_id_ >= 0) {
    goal_world_pos_[0] = m_->body_pos[3 * goal_body_id_ + 0];
    goal_world_pos_[1] = m_->body_pos[3 * goal_body_id_ + 1];
    goal_world_pos_[2] = 0.11;
    RCLCPP_INFO(this->get_logger(), "goal position: (%.2f, %.2f, %.2f)",
                goal_world_pos_[0], goal_world_pos_[1], goal_world_pos_[2]);
  } else {
    RCLCPP_WARN(this->get_logger(), "goal_oppo_group body not found, fallback to (5.0, 0, 0.11)");
    goal_world_pos_[0] = 5.0;
    goal_world_pos_[1] = 0.0;
    goal_world_pos_[2] = 0.11;
  }

  int ball_geom_id = mj_name2id(m_, mjOBJ_GEOM, "ball_geom");
  if (ball_geom_id >= 0) {
    ball_geom_state_.id = ball_geom_id;
    ball_geom_state_.rgba[0] = m_->geom_rgba[4 * ball_geom_id + 0];
    ball_geom_state_.rgba[1] = m_->geom_rgba[4 * ball_geom_id + 1];
    ball_geom_state_.rgba[2] = m_->geom_rgba[4 * ball_geom_id + 2];
    ball_geom_state_.rgba[3] = m_->geom_rgba[4 * ball_geom_id + 3];
    ball_geom_state_.contype = m_->geom_contype[ball_geom_id];
    ball_geom_state_.conaffinity = m_->geom_conaffinity[ball_geom_id];
  }

  // 足球放置预览点（mocap body）
  ball_preview_body_id_ = mj_name2id(m_, mjOBJ_BODY, "ball_preview");
  if (ball_preview_body_id_ >= 0) {
    ball_preview_mocap_id_ = m_->body_mocapid[ball_preview_body_id_];
  } else {
    RCLCPP_WARN(this->get_logger(), "ball_preview body not found in MuJoCo model");
  }

  // 初始为非足球模式：隐藏球门和球
  SetSoccerMode(false);
}

void SimZ01StdFootballPlayer::ComputeControl(const mjModel* m, mjData* d) {
  (void)m;
  if (!interface_ready_ || d == nullptr) {
    return;
  }
  // 读取当前仿真位置和速度，用于 PD 控制计算。
  mj_interface_.ReadJointState(d, q_rel_, dq_rel_);

  // 只取队列中的最新一帧并合并到缓存。排空队列保证不会执行陈旧目标；
  // 缓存设计可以在没有新命令时保持上一帧目标。
  robot_interfaces::msg::JointCommands joint_commands;
  if (TryTakeLatestCommand(joint_commands)) {
    UpdateCommandCache(joint_commands, d->time);
  }

  // 统一力矩计算路径（与狗形仿真一致）：前馈 + PD → 电机力矩指令限幅 → TN 曲线。
  const auto& motor_ids = mj_interface_.GetMotorIds();
  for (std::size_t i = 0; i < mj_interface_.GetJointCount(); i++) {
    const Eigen::Index idx = static_cast<Eigen::Index>(i);
    const auto& command = sim_joint_commands_[i];
    d->ctrl[motor_ids[i]] = sim_common::ComputeJointTorque(
        kJointMotorSpecs[i], enable_control_ && command.enable,
        command.effort, command.kp, command.kd,
        command.position, q_rel_[idx + 7],
        command.velocity, dq_rel_[idx + 6]);
  }
}

void SimZ01StdFootballPlayer::PublishExtra(const mjData* d) {
  // 发布基座标系下的球和球门位置（仅足球模式）
  PublishTargetPoints(d);
}

void SimZ01StdFootballPlayer::HandleBallPosition(const geometry_msgs::msg::Point::SharedPtr msg) {
  if (soccer_ball_joint_qposadr_ < 0 || !d_) {
    return;
  }
  int qpos_adr = soccer_ball_joint_qposadr_;
  d_->qpos[qpos_adr + 0] = msg->x;
  d_->qpos[qpos_adr + 1] = msg->y;
  d_->qpos[qpos_adr + 2] = 0.11;
  d_->qpos[qpos_adr + 3] = 1.0;
  d_->qpos[qpos_adr + 4] = 0.0;
  d_->qpos[qpos_adr + 5] = 0.0;
  d_->qpos[qpos_adr + 6] = 0.0;
  // 速度归零
  if (soccer_ball_body_id_ >= 0) {
    int jnt_adr = m_->body_jntadr[soccer_ball_body_id_];
    int dof_adr = m_->jnt_dofadr[jnt_adr];
    for (int i = 0; i < 6; ++i) {
      d_->qvel[dof_adr + i] = 0.0;
    }
  }
}

void SimZ01StdFootballPlayer::HandleBallVelocity(const geometry_msgs::msg::Vector3::SharedPtr msg) {
  if (soccer_ball_body_id_ < 0 || !d_) {
    return;
  }
  int jnt_adr = m_->body_jntadr[soccer_ball_body_id_];
  int dof_adr = m_->jnt_dofadr[jnt_adr];
  d_->qvel[dof_adr + 0] = msg->x;
  d_->qvel[dof_adr + 1] = msg->y;
  d_->qvel[dof_adr + 2] = msg->z;
  d_->qvel[dof_adr + 3] = 0.0;
  d_->qvel[dof_adr + 4] = 0.0;
  d_->qvel[dof_adr + 5] = 0.0;
}

void SimZ01StdFootballPlayer::HandleBallPreview(const geometry_msgs::msg::Point::SharedPtr msg) {
  if (ball_preview_mocap_id_ < 0 || !d_) {
    return;
  }
  if (msg->z < 0.0) {
    d_->mocap_pos[3 * ball_preview_mocap_id_ + 0] = 0.0;
    d_->mocap_pos[3 * ball_preview_mocap_id_ + 1] = 0.0;
    d_->mocap_pos[3 * ball_preview_mocap_id_ + 2] = -100.0;
  } else {
    d_->mocap_pos[3 * ball_preview_mocap_id_ + 0] = msg->x;
    d_->mocap_pos[3 * ball_preview_mocap_id_ + 1] = msg->y;
    d_->mocap_pos[3 * ball_preview_mocap_id_ + 2] = 0.11;
  }
}

void SimZ01StdFootballPlayer::HandleSoccerMode(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data != soccer_mode_) {
    SetSoccerMode(msg->data);
  }
}

void SimZ01StdFootballPlayer::SetSoccerMode(bool enabled) {
  soccer_mode_ = enabled;
  // 场地（草坪/白线）与球门一起随模式显隐，初始进入仿真不显示足球场。
  SetGeomsVisible(field_geoms_, enabled);
  if (enabled) {
    // 显示球门和球（球复位到默认位置）
    SetGeomsVisible(soccer_geoms_, true);
    if (soccer_ball_joint_qposadr_ >= 0 && d_) {
      int qpos_adr = soccer_ball_joint_qposadr_;
      d_->qpos[qpos_adr + 0] = 0.4;
      d_->qpos[qpos_adr + 1] = 0.0;
      d_->qpos[qpos_adr + 2] = 0.11;
      d_->qpos[qpos_adr + 3] = 1.0;
      d_->qpos[qpos_adr + 4] = 0.0;
      d_->qpos[qpos_adr + 5] = 0.0;
      d_->qpos[qpos_adr + 6] = 0.0;
      int jnt_adr = m_->body_jntadr[soccer_ball_body_id_];
      int dof_adr = m_->jnt_dofadr[jnt_adr];
      for (int i = 0; i < 6; ++i) d_->qvel[dof_adr + i] = 0.0;
    }
    if (ball_geom_state_.id >= 0) {
      m_->geom_rgba[4 * ball_geom_state_.id + 3] = ball_geom_state_.rgba[3];
      m_->geom_contype[ball_geom_state_.id] = ball_geom_state_.contype;
      m_->geom_conaffinity[ball_geom_state_.id] = ball_geom_state_.conaffinity;
    }
  } else {
    // 隐藏球门和球（球移到远处地面，避免与行走干涉）
    SetGeomsVisible(soccer_geoms_, false);
    if (soccer_ball_joint_qposadr_ >= 0 && d_) {
      int qpos_adr = soccer_ball_joint_qposadr_;
      d_->qpos[qpos_adr + 0] = 100.0;
      d_->qpos[qpos_adr + 1] = 100.0;
      d_->qpos[qpos_adr + 2] = 0.11;
      d_->qpos[qpos_adr + 3] = 1.0;
      d_->qpos[qpos_adr + 4] = 0.0;
      d_->qpos[qpos_adr + 5] = 0.0;
      d_->qpos[qpos_adr + 6] = 0.0;
      int jnt_adr = m_->body_jntadr[soccer_ball_body_id_];
      int dof_adr = m_->jnt_dofadr[jnt_adr];
      for (int i = 0; i < 6; ++i) d_->qvel[dof_adr + i] = 0.0;
    }
    if (ball_geom_state_.id >= 0) {
      m_->geom_rgba[4 * ball_geom_state_.id + 3] = 0.0f;
      m_->geom_contype[ball_geom_state_.id] = 0;
      m_->geom_conaffinity[ball_geom_state_.id] = 0;
    }
  }
}

void SimZ01StdFootballPlayer::PublishTargetPoints(const mjData* d) {
  // 仅在足球模式下发布
  if (!soccer_mode_ || soccer_ball_body_id_ < 0) return;
  if (d->xpos[3 * soccer_ball_body_id_ + 2] < 0.0) {
    return;
  }
  // 机器人浮动基座：torso_link 是第一个自由关节，qpos[0:7] = xyz + quat(w,x,y,z)
  const double* robot_pos = d->qpos;
  const double* robot_quat = d->qpos + 3;
  double inv_robot_quat[4] = {robot_quat[0], -robot_quat[1], -robot_quat[2], -robot_quat[3]};

  // 1. 足球：世界坐标 -> 基座标系
  double ball_delta[3] = {
      d->xpos[3 * soccer_ball_body_id_ + 0] - robot_pos[0],
      d->xpos[3 * soccer_ball_body_id_ + 1] - robot_pos[1],
      d->xpos[3 * soccer_ball_body_id_ + 2] - robot_pos[2],
  };
  double ball_base[3];
  mju_rotVecQuat(ball_base, ball_delta, inv_robot_quat);
  auto target_msg = geometry_msgs::msg::Point();
  target_msg.x = ball_base[0];
  target_msg.y = ball_base[1];
  target_msg.z = ball_base[2];
  target_point_pub_->publish(target_msg);

  // 2. 球门（对方球门 body 的门口位置，从模型读出，不硬编码）
  double goal_delta[3] = {goal_world_pos_[0] - robot_pos[0],
                          goal_world_pos_[1] - robot_pos[1],
                          goal_world_pos_[2] - robot_pos[2]};
  double goal_base[3];
  mju_rotVecQuat(goal_base, goal_delta, inv_robot_quat);
  auto dest_msg = geometry_msgs::msg::Point();
  dest_msg.x = goal_base[0];
  dest_msg.y = goal_base[1];
  dest_msg.z = goal_base[2];
  target_destination_pub_->publish(dest_msg);
}

void SimZ01StdFootballPlayer::CollectBodyGeoms(const char* body_name, std::vector<GeomState>& out) {
  int body_id = mj_name2id(m_, mjOBJ_BODY, body_name);
  if (body_id < 0) {
    RCLCPP_WARN(this->get_logger(), "body %s not found in MuJoCo model", body_name);
    return;
  }
  int geom_start = m_->body_geomadr[body_id];
  int geom_num = m_->body_geomnum[body_id];
  for (int i = 0; i < geom_num; ++i) {
    int gid = geom_start + i;
    GeomState s;
    s.id = gid;
    s.rgba[0] = m_->geom_rgba[4 * gid + 0];
    s.rgba[1] = m_->geom_rgba[4 * gid + 1];
    s.rgba[2] = m_->geom_rgba[4 * gid + 2];
    s.rgba[3] = m_->geom_rgba[4 * gid + 3];
    s.contype = m_->geom_contype[gid];
    s.conaffinity = m_->geom_conaffinity[gid];
    out.push_back(s);
  }
}

void SimZ01StdFootballPlayer::CollectGeomsByPrefix(const char* prefix, std::vector<GeomState>& out) {
  // 按 geom 名前缀收集（soccer_field.xml 的命名约定：field_*/line_*/
  // center_circle/pen_*/goal_area_*/penalty_spot_*）。
  const std::size_t prefix_len = std::strlen(prefix);
  for (int gid = 0; gid < m_->ngeom; ++gid) {
    const char* name = mj_id2name(m_, mjOBJ_GEOM, gid);
    if (name == nullptr || std::strncmp(name, prefix, prefix_len) != 0) {
      continue;
    }
    GeomState s;
    s.id = gid;
    s.rgba[0] = m_->geom_rgba[4 * gid + 0];
    s.rgba[1] = m_->geom_rgba[4 * gid + 1];
    s.rgba[2] = m_->geom_rgba[4 * gid + 2];
    s.rgba[3] = m_->geom_rgba[4 * gid + 3];
    s.contype = m_->geom_contype[gid];
    s.conaffinity = m_->geom_conaffinity[gid];
    out.push_back(s);
  }
}

void SimZ01StdFootballPlayer::SetGeomsVisible(const std::vector<GeomState>& geoms, bool visible) {
  for (const auto& s : geoms) {
    if (visible) {
      m_->geom_rgba[4 * s.id + 0] = s.rgba[0];
      m_->geom_rgba[4 * s.id + 1] = s.rgba[1];
      m_->geom_rgba[4 * s.id + 2] = s.rgba[2];
      m_->geom_rgba[4 * s.id + 3] = s.rgba[3];
      m_->geom_contype[s.id] = s.contype;
      m_->geom_conaffinity[s.id] = s.conaffinity;
    } else {
      m_->geom_rgba[4 * s.id + 3] = 0.0f;
      m_->geom_contype[s.id] = 0;
      m_->geom_conaffinity[s.id] = 0;
    }
  }
}
