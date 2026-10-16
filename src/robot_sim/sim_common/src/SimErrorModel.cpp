#include "sim_common/SimErrorModel.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace sim_common {
namespace {

// d->time 的"前进/相等/倒退"判据容差。d->time 是 k·dt 累加出来的，同一步内的
// 重复调用（暂停时每轮的 mj_forward）会传完全相同的值，所以用很小的绝对容差即可。
constexpr double kTimeEps = 1e-9;

constexpr double kDegToRad = M_PI / 180.0;

}  // namespace

//---------------------------------- TorqueDelay ------------------------------------------------

void TorqueDelay::Init(int steps, double drop_probability, unsigned int seed) {
  steps_ = std::max(0, steps);
  drop_probability_ = std::clamp(drop_probability, 0.0, 1.0);
  drop_rng_.seed(seed);
  drop_dist_.reset();
  ring_.clear();
  applied_.clear();
  count_ = 0;
  write_index_ = 0;
  started_ = false;
}

void TorqueDelay::Refill(std::size_t count, const int* actuator_ids, double* ctrl) {
  applied_.assign(count, 0.0);
  for (std::size_t i = 0; i < count; ++i) {
    applied_[i] = ctrl[actuator_ids[i]];
  }
  // 每一帧都填当前值：这样前 N 步"弹出来的值"就等于当前值，延迟从稳态开始，
  // 不会在启动/Reload 后引入 N 步零力矩的瞬态。
  ring_.resize(static_cast<std::size_t>(steps_) * count);
  for (int frame = 0; frame < steps_; ++frame) {
    std::copy(applied_.begin(), applied_.end(), ring_.begin() + static_cast<std::size_t>(frame) * count);
  }
  write_index_ = 0;
  started_ = true;
}

void TorqueDelay::Apply(std::size_t count, const int* actuator_ids, double* ctrl, double time) {
  if (bypass() || count == 0 || actuator_ids == nullptr || ctrl == nullptr) {
    return;
  }

  const auto hold_current = [&]() {
    for (std::size_t i = 0; i < count; ++i) {
      ctrl[actuator_ids[i]] = applied_[i];
    }
  };

  // 首次调用、或执行器数量变化（模型重载后执行器表不同）：重开环。
  if (!started_ || count != count_) {
    count_ = count;
    Refill(count, actuator_ids, ctrl);
    last_time_ = time;
    return;  // 环里全是当前值，等价于本步无延迟
  }

  const double dt = time - last_time_;
  if (dt < -kTimeEps) {
    // 时间倒退（GUI Reset 把仿真时间拨回 0、拖动历史时间轴）：清环重填，否则会
    // 吐出上一轮运行残留的力矩。
    Refill(count, actuator_ids, ctrl);
    last_time_ = time;
    return;
  }

  if (dt <= kTimeEps) {
    // 时间未前进（暂停时每轮的 mj_forward、/reload_* 与球复位后的 mj_forward）：
    // 覆盖掉本步新算出的力矩、保持当前施加值，且不推进环。
    hold_current();
    return;
  }

  // 正常步进：这一帧按概率丢弃 —— 丢帧的物理含义是"驱动器这一拍没收到新命令，
  // 继续执行上一帧"，表现为施加值保持不变。
  if (drop_probability_ > 0.0 && drop_dist_(drop_rng_) < drop_probability_) {
    hold_current();
    last_time_ = time;
    return;
  }

  // 弹出环中最旧的一帧作为本步施加值，再把本步新值写进该槽位。
  double* oldest = ring_.data() + write_index_ * count;
  for (std::size_t i = 0; i < count; ++i) {
    applied_[i] = oldest[i];
    oldest[i] = ctrl[actuator_ids[i]];
    ctrl[actuator_ids[i]] = applied_[i];
  }
  write_index_ = (write_index_ + 1) % static_cast<std::size_t>(steps_);
  last_time_ = time;
}

//---------------------------------- CommandDelay -------------------------------------------------

void CommandDelay::Init(std::size_t joint_count, double delay_seconds) {
  delay_seconds_ = std::max(0.0, delay_seconds);
  has_delay_ = delay_seconds_ > 0.0;
  joint_count_ = joint_count;

  ring_time_.assign(kRingCapacity, 0.0);
  ring_.assign(kRingCapacity, std::vector<SimJointCommand>());
  ring_head_ = 0;
  ring_count_ = 0;
  last_time_ = 0.0;
  started_ = false;
}

void CommandDelay::Reset(double time, const std::vector<SimJointCommand>& fresh,
                         std::vector<SimJointCommand>& out) {
  out = fresh;
  last_time_ = time;
  started_ = true;

  // 历史环全部填当前帧：这样启动/Reload 后第一次"取 Δ 之前那一帧"就能取到当前值，
  // 不引入 Δ 长的陈旧参考瞬态（与 TorqueDelay::Refill 同样的考虑）。
  ring_.assign(kRingCapacity, fresh);
  ring_time_.assign(kRingCapacity, time);
  ring_head_ = 0;
  ring_count_ = kRingCapacity;
}

void CommandDelay::PushSnapshot(double time, const std::vector<SimJointCommand>& fresh) {
  ring_time_[ring_head_] = time;
  ring_[ring_head_] = fresh;
  ring_head_ = (ring_head_ + 1) % kRingCapacity;
  ring_count_ = std::min(ring_count_ + 1, kRingCapacity);
}

void CommandDelay::LookupDelayed(double time, std::vector<SimJointCommand>& out) const {
  if (ring_count_ == 0) {
    return;  // 不该发生（Reset 已填满）；保守起见保持 out 不动
  }

  const double target = time - delay_seconds_;
  // 最旧一份：环满时在 ring_head_（下一个待写位置），未满时是 0。
  std::size_t index = (ring_count_ == kRingCapacity) ? ring_head_ : 0;
  for (std::size_t n = 0; n < ring_count_; ++n) {
    const std::size_t i = (ring_head_ + kRingCapacity - 1 - n) % kRingCapacity;
    if (ring_time_[i] <= target + kTimeEps) {
      index = i;  // 从最新往回扫，第一次命中即"Δ 之前最新的那一帧"
      break;
    }
  }
  // 环里没有足够老的（刚 Reset，或 Δ 超过历史长度）：退回最旧一帧，不报错。

  out = ring_[index];
}

void CommandDelay::Apply(double time, const std::vector<SimJointCommand>& fresh,
                         std::vector<SimJointCommand>& out) {
  if (bypass()) {
    out = fresh;  // 旁路：只做拷贝
    return;
  }

  // 首次调用、或关节数变化（模型重载）：重开环。
  if (!started_ || joint_count_ != fresh.size()) {
    joint_count_ = fresh.size();
    Reset(time, fresh, out);
    return;
  }

  const double dt = time - last_time_;
  if (dt < -kTimeEps) {
    // 时间倒退（GUI Reset / /reset_*）：清环重填，否则会吐出上一轮运行的参考值。
    Reset(time, fresh, out);
    return;
  }
  if (dt <= kTimeEps) {
    // 时间未前进：同一帧被重复合并（暂停时每轮的 mj_forward）。不推进环，
    // 也不改 out——保持上一次给出的延迟参考。
    return;
  }

  PushSnapshot(time, fresh);
  LookupDelayed(time, out);
  last_time_ = time;
}

//---------------------------------- FeedbackDelay ------------------------------------------------

void FeedbackDelay::Init(double hold_seconds, double delay_seconds) {
  hold_seconds_ = std::max(0.0, hold_seconds);
  delay_seconds_ = std::max(0.0, delay_seconds);
  has_hold_ = hold_seconds_ > 0.0;
  has_delay_ = delay_seconds_ > 0.0;

  ring_time_.assign(kRingCapacity, 0.0);
  ring_q_.assign(kRingCapacity, Eigen::VectorXd());
  ring_dq_.assign(kRingCapacity, Eigen::VectorXd());
  ring_tau_.assign(kRingCapacity, Eigen::VectorXd());
  ring_head_ = 0;
  ring_count_ = 0;

  held_q_.resize(0);
  held_dq_.resize(0);
  held_tau_.resize(0);
  next_sample_time_ = 0.0;
  last_time_ = 0.0;
  started_ = false;
  min_forward_dt_ = 0.0;
  forward_count_ = 0;
  hold_warned_ = false;
}

void FeedbackDelay::Reset(double time, const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                          const Eigen::VectorXd& tau) {
  held_q_ = q;
  held_dq_ = dq;
  held_tau_ = tau;
  next_sample_time_ = time + hold_seconds_;
  last_time_ = time;
  started_ = true;

  // 历史环全部填当前值：这样启动/Reload 后第一次"取 delay 之前那份"就能取到当前值，
  // 不引入 delay 长的陈旧值瞬态（与 TorqueDelay::Refill 同样的考虑）。
  if (has_delay_) {
    ring_time_.assign(kRingCapacity, time);
    ring_q_.assign(kRingCapacity, q);
    ring_dq_.assign(kRingCapacity, dq);
    ring_tau_.assign(kRingCapacity, tau);
    ring_head_ = 0;
    ring_count_ = kRingCapacity;
  } else {
    ring_count_ = 0;
  }
}

void FeedbackDelay::PushSnapshot(double time, const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                                 const Eigen::VectorXd& tau) {
  ring_time_[ring_head_] = time;
  ring_q_[ring_head_] = q;
  ring_dq_[ring_head_] = dq;
  ring_tau_[ring_head_] = tau;
  ring_head_ = (ring_head_ + 1) % kRingCapacity;
  ring_count_ = std::min(ring_count_ + 1, kRingCapacity);
}

void FeedbackDelay::LookupDelayed(double time, Eigen::VectorXd& q, Eigen::VectorXd& dq,
                                  Eigen::VectorXd& tau) const {
  if (ring_count_ == 0) {
    // 不该发生（Reset 已填满）；保守起见保持寄存器不动。
    return;
  }

  const double target = time - delay_seconds_;
  // 最旧一份：环满时在 ring_head_（下一个待写位置），未满时是 0。
  std::size_t index = (ring_count_ == kRingCapacity) ? ring_head_ : 0;
  for (std::size_t n = 0; n < ring_count_; ++n) {
    const std::size_t i = (ring_head_ + kRingCapacity - 1 - n) % kRingCapacity;
    if (ring_time_[i] <= target + kTimeEps) {
      index = i;  // 从最新往回扫，第一次命中即"delay 之前最新的那一份"
      break;
    }
  }
  // 环里没有足够老的（刚 Reset，或 delay 超过历史长度）：退回最旧一份，不报错。

  q = ring_q_[index];
  dq = ring_dq_[index];
  tau = ring_tau_[index];
}

void FeedbackDelay::Apply(double time, const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                          const Eigen::VectorXd& tau, Eigen::VectorXd& q_out,
                          Eigen::VectorXd& dq_out, Eigen::VectorXd& tau_out) {
  if (bypass()) {
    // 旁路：只做拷贝，不做任何数值运算（保证与改造前逐位一致）。
    q_out = q;
    dq_out = dq;
    tau_out = tau;
    return;
  }

  // 首次调用、或维度变化（模型重载后关节数不同）：重开寄存器。
  if (!started_ || held_q_.size() != q.size() || held_dq_.size() != dq.size() ||
      held_tau_.size() != tau.size()) {
    Reset(time, q, dq, tau);
  } else {
    const double dt = time - last_time_;
    if (dt < -kTimeEps) {
      // 时间倒退（GUI Reset 把仿真时间拨回 0、拖动历史时间轴）：清空重采，否则会
      // 吐出上一轮运行残留的值。
      Reset(time, q, dq, tau);
    } else if (dt <= kTimeEps) {
      // 时间未前进（暂停时每轮的 mj_forward）：原样写回上一次的上报值，不推进寄存器、
      // 也不压快照——同一时刻反复调用会灌满历史环。
    } else {
      if (has_delay_) {
        PushSnapshot(time, q, dq, tau);
      }

      if (forward_count_ == 0 || dt < min_forward_dt_) {
        min_forward_dt_ = dt;
      }
      ++forward_count_;

      // 比较都带 kTimeEps：next_sample_time_ 靠累加推进，末位误差会累积到 1e-17 量级，
      // 用裸 >= 比较会让"本该到点"的那一拍偶尔被判成没到（保持晚一拍）。
      if (!has_hold_ || time + kTimeEps >= next_sample_time_) {
        if (has_delay_) {
          LookupDelayed(time, held_q_, held_dq_, held_tau_);
        } else {
          held_q_ = q;
          held_dq_ = dq;
          held_tau_ = tau;
        }
        if (has_hold_) {
          // 相位用绝对时刻累加，保持相对发布栅格持续漂移（复现实测的 10/15 ms 混合
          // 间隔）。若跨过多个周期（仿真时间被大步推进/发布节拍重对齐），只采一次并把
          // 相位重置到当前，避免连续补采。
          next_sample_time_ += hold_seconds_;
          if (time + kTimeEps >= next_sample_time_) {
            next_sample_time_ = time + hold_seconds_;
          }
        }
      }

      // 自检：hold 不大于发布周期时等于每拍都刷新，保持不会生效。与 ApplyTorqueDelay
      // 的 RK4 自检同样只报一次。
      if (has_hold_ && !hold_warned_ && forward_count_ >= 64 && min_forward_dt_ >= hold_seconds_) {
        std::printf(
            "WARN: errors.feedback_hold_ms=%.3f 不大于反馈发布周期 %.3f ms，采样保持不会生效"
            "（每次发布都会刷新）\n",
            hold_seconds_ * 1000.0, min_forward_dt_ * 1000.0);
        hold_warned_ = true;
      }
    }
  }

  q_out = held_q_;
  dq_out = held_dq_;
  tau_out = held_tau_;
  last_time_ = time;
}

//---------------------------------- SimErrorModel -------------------------------------------------

void SimErrorModel::Configure(const SimErrorOptions& options) {
  options_ = options;
  // seed == 0 表示整体关闭：所有注入走早退分支，不产生任何数值改动（连 "+0.0"
  // 这种会改动 -0.0 位模式的运算都不做），保证与改造前逐位一致。
  enabled_ = options_.seed != 0;

  joint_errors_active_ =
      options_.joint_zero_offset_std_rad > 0.0 || !options_.joint_zero_offset_rad.empty() ||
      options_.joint_pos_noise_std_rad > 0.0 || options_.joint_vel_noise_std_rad_s > 0.0;
  imu_errors_active_ = options_.mount_std_deg > 0.0 || options_.gyro_bias_std_rad_s > 0.0 ||
                       options_.acc_bias_std_m_s2 > 0.0 || options_.gyro_noise_std_rad_s > 0.0 ||
                       options_.acc_noise_std_m_s2 > 0.0 || options_.att_bias_std_deg > 0.0 ||
                       options_.att_noise_std_deg > 0.0;
}

Eigen::Quaterniond SimErrorModel::DrawSmallRotation(double std_deg) {
  if (std_deg <= 0.0) {
    return Eigen::Quaterniond::Identity();
  }
  const double std_rad = std_deg * kDegToRad;
  const Eigen::Vector3d axis(normal_(rng_) * std_rad, normal_(rng_) * std_rad, normal_(rng_) * std_rad);
  const double angle = axis.norm();
  if (angle < 1e-12) {
    return Eigen::Quaterniond::Identity();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis / angle));
}

void SimErrorModel::Init(std::size_t joint_count, const std::vector<int>& motor_ids, double timestep,
                         unsigned int seed_index) {
  joint_count_ = joint_count;
  motor_ids_ = motor_ids;
  timestep_ = timestep;
  warning_.clear();
  noise_valid_ = false;
  check_time_valid_ = false;
  timestep_warned_ = false;

  if (!enabled_) {
    torque_delay_.Init(0, 0.0, 0);
    feedback_.Init(0.0, 0.0);
    command_delay_.Init(0, 0.0);
    return;
  }

  // 常值误差的抽样顺序固定（零位 → 安装角 → 陀螺零偏 → 加计零偏 → 姿态误差）：
  // 顺序变了就不再是同一台"个体"。多机用 seed_seq 把机器人序号混进种子，保证
  // 6 台各不相同，且不会像 seed*100+index 那样在机器人数变多时撞车。
  std::seed_seq seq{options_.seed, seed_index};
  rng_.seed(seq);
  normal_.reset();

  joint_zero_offset_.assign(joint_count_, 0.0);
  if (!options_.joint_zero_offset_rad.empty()) {
    const std::size_t n = std::min(joint_count_, options_.joint_zero_offset_rad.size());
    std::copy_n(options_.joint_zero_offset_rad.begin(), n, joint_zero_offset_.begin());
  } else if (options_.joint_zero_offset_std_rad > 0.0) {
    for (double& offset : joint_zero_offset_) {
      offset = normal_(rng_) * options_.joint_zero_offset_std_rad;
    }
  }

  mount_quat_ = DrawSmallRotation(options_.mount_std_deg);
  for (int i = 0; i < 3; ++i) {
    gyro_bias_[i] = normal_(rng_) * options_.gyro_bias_std_rad_s;
    acc_bias_[i] = normal_(rng_) * options_.acc_bias_std_m_s2;
  }
  att_bias_quat_ = DrawSmallRotation(options_.att_bias_std_deg);

  joint_pos_noise_.assign(joint_count_, 0.0);
  joint_vel_noise_.assign(joint_count_, 0.0);

  // 延迟只认整数个仿真步：3 ms 在 dt=2 ms 的模型上只能取整到 4 ms，所以把实际
  // 生效值放进 Summary()，启动日志里能直接看到，避免"配了没效果"的误判。
  if (options_.torque_delay_ms > 0.0 && timestep_ > 0.0) {
    const int steps = static_cast<int>(std::llround(options_.torque_delay_ms / 1000.0 / timestep_));
    torque_delay_.Init(steps, options_.drop_probability, options_.seed + seed_index);
    if (steps <= 0) {
      warning_ += "errors.torque_delay_ms 小于一个仿真步，延迟被取整为 0；";
    }
  } else {
    torque_delay_.Init(0, 0.0, 0);
    if (options_.torque_delay_ms > 0.0) {
      warning_ += "errors.torque_delay_ms > 0 但 m->opt.timestep 无效，延迟未生效；";
    }
  }
  if (options_.drop_probability > 0.0 && torque_delay_.bypass()) {
    warning_ += "errors.drop_probability 需要 torque_delay_ms > 0 才生效；";
  }

  // 反馈上行：确定性时序量，不需要随机数，按秒配置后直接生效。hold 与发布周期的关系
  // 只能在运行期观测（见 FeedbackDelay::Apply 的自检），这里不做判断。
  feedback_.Init(options_.feedback_hold_ms / 1000.0, options_.feedback_delay_ms / 1000.0);

  // 指令延迟：同样按秒配置，**不取整到仿真步**——它作用在指令帧上，而指令是按时间到达的，
  // 与 timestep 无关。
  command_delay_.Init(joint_count_, options_.command_delay_ms / 1000.0);
}

std::string SimErrorModel::Summary() const {
  if (!enabled_) {
    return "disabled (errors.seed = 0)";
  }
  std::ostringstream os;
  os << "seed=" << options_.seed << " joints=" << joint_count_;
  if (joint_errors_active_) {
    os << " joint(zero_offset_std=" << options_.joint_zero_offset_std_rad
       << " pos_noise=" << options_.joint_pos_noise_std_rad
       << " vel_noise=" << options_.joint_vel_noise_std_rad_s << ")";
  }
  if (imu_errors_active_) {
    os << " imu(mount=" << options_.mount_std_deg << "deg gyro_bias=" << options_.gyro_bias_std_rad_s
       << " gyro_noise=" << options_.gyro_noise_std_rad_s << " acc_bias=" << options_.acc_bias_std_m_s2
       << " acc_noise=" << options_.acc_noise_std_m_s2 << " att_bias=" << options_.att_bias_std_deg
       << "deg att_noise=" << options_.att_noise_std_deg << "deg)";
  }
  if (!command_delay_.bypass()) {
    os << " command_delay=" << options_.command_delay_ms << "ms(参考路径, 回路外)";
  }
  if (!torque_delay_.bypass()) {
    os << " torque_delay=" << torque_delay_.steps() << " steps(" << torque_delay_.steps() * timestep_ * 1000.0
       << "ms, 配置 " << options_.torque_delay_ms << "ms, drop=" << options_.drop_probability << ")";
  }
  if (!feedback_.bypass()) {
    os << " feedback(";
    if (feedback_.has_hold()) {
      os << "hold=" << feedback_.holdSeconds() * 1000.0 << "ms/" << 1.0 / feedback_.holdSeconds()
         << "Hz";
    }
    if (feedback_.has_hold() && feedback_.has_delay()) {
      os << " ";
    }
    if (feedback_.has_delay()) {
      os << "delay=" << feedback_.delaySeconds() * 1000.0 << "ms";
    }
    os << ", 只作用于发布值)";
  }
  return os.str();
}

void SimErrorModel::RefreshStepNoise(double time) {
  // 按 d->time 键控：时间没变就直接复用 —— 同一步内 PD 与发布读到同一个编码器
  // 采样，且暂停时每轮的 mj_forward 不会白吃随机数。
  if (noise_valid_ && time == noise_time_) {
    return;
  }
  noise_time_ = time;
  noise_valid_ = true;

  if (joint_errors_active_) {
    for (double& n : joint_pos_noise_) {
      n = normal_(rng_) * options_.joint_pos_noise_std_rad;
    }
    for (double& n : joint_vel_noise_) {
      n = normal_(rng_) * options_.joint_vel_noise_std_rad_s;
    }
  }
  if (imu_errors_active_) {
    for (int i = 0; i < 3; ++i) {
      gyro_noise_[i] = normal_(rng_) * options_.gyro_noise_std_rad_s;
      acc_noise_[i] = normal_(rng_) * options_.acc_noise_std_m_s2;
    }
    att_noise_quat_ = DrawSmallRotation(options_.att_noise_std_deg);
  }
}

void SimErrorModel::PerturbJoint(std::size_t i, double time, double& q, double& dq) {
  if (!enabled_ || !joint_errors_active_ || i >= joint_count_) {
    return;
  }
  RefreshStepNoise(time);
  q += joint_zero_offset_[i] + joint_pos_noise_[i];
  dq += joint_vel_noise_[i];
}

void SimErrorModel::PerturbImu(double time, double* quat_xyzw, double* gyro, Eigen::Vector3d& acc) {
  if (!enabled_ || !imu_errors_active_ || quat_xyzw == nullptr || gyro == nullptr) {
    return;
  }
  RefreshStepNoise(time);

  // 陀螺/加计的真值都在机体系，先用安装角误差转到 IMU 系，再叠加零偏与白噪声。
  const Eigen::Vector3d gyro_meas =
      mount_quat_ * Eigen::Vector3d(gyro[0], gyro[1], gyro[2]) + gyro_bias_ + gyro_noise_;
  gyro[0] = gyro_meas.x();
  gyro[1] = gyro_meas.y();
  gyro[2] = gyro_meas.z();

  acc = mount_quat_ * acc + acc_bias_ + acc_noise_;

  // 姿态：安装角、AHRS 姿态估计误差与白噪声统一右乘（机体系），输出 (x,y,z,w)。
  const Eigen::Vector4d raw(quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3]);
  Eigen::Quaterniond q_true(raw[3], raw[0], raw[1], raw[2]);
  if (q_true.norm() < 1.0e-9) {
    q_true = Eigen::Quaterniond::Identity();
  }
  Eigen::Quaterniond q_pub = q_true * mount_quat_ * att_bias_quat_ * att_noise_quat_;
  q_pub.normalize();
  quat_xyzw[0] = q_pub.x();
  quat_xyzw[1] = q_pub.y();
  quat_xyzw[2] = q_pub.z();
  quat_xyzw[3] = q_pub.w();
}

void SimErrorModel::ApplyTorqueDelay(mjData* data) {
  if (!enabled_ || torque_delay_.bypass() || data == nullptr || motor_ids_.empty()) {
    return;
  }

  // 自检：延迟按"整数个仿真步"实现，前提是每个 mj_step 恰好触发一次控制回调。
  // RK4 积分器每步会触发 4 次，实际延迟会变成配置值的 4 倍；这里只报一次。
  if (!timestep_warned_ && timestep_ > 0.0) {
    if (check_time_valid_) {
      const double dt = data->time - check_time_;
      if (dt > kTimeEps && std::abs(dt - timestep_) > 1e-6) {
        std::printf(
            "WARN: 控制步长 %.6f s 与模型 timestep %.6f s 不一致（积分器改成 RK4？），"
            "errors.torque_delay_ms 的实际延迟会被放大\n",
            dt, timestep_);
        timestep_warned_ = true;
      }
    }
    check_time_ = data->time;
    check_time_valid_ = true;
  }

  torque_delay_.Apply(motor_ids_.size(), motor_ids_.data(), data->ctrl, data->time);
}

void SimErrorModel::ApplyFeedbackDelay(double time, const Eigen::VectorXd& q,
                                       const Eigen::VectorXd& dq, const Eigen::VectorXd& tau,
                                       Eigen::VectorXd& q_out, Eigen::VectorXd& dq_out,
                                       Eigen::VectorXd& tau_out) {
  // 关闭时不必走 FeedbackDelay 的分支：直接拷贝，保持"零配置逐位一致"。
  if (!enabled_ || feedback_.bypass()) {
    q_out = q;
    dq_out = dq;
    tau_out = tau;
    return;
  }
  feedback_.Apply(time, q, dq, tau, q_out, dq_out, tau_out);
}

void SimErrorModel::ApplyCommandDelay(double time, const std::vector<SimJointCommand>& fresh,
                                      std::vector<SimJointCommand>& out) {
  // 关闭时不必走 CommandDelay 的分支：直接拷贝，保持"零配置逐位一致"。
  if (!enabled_ || command_delay_.bypass()) {
    out = fresh;
    return;
  }
  command_delay_.Apply(time, fresh, out);
}

double SimErrorModel::jointZeroOffset(std::size_t i) const {
  return i < joint_zero_offset_.size() ? joint_zero_offset_[i] : 0.0;
}

}  // namespace sim_common
