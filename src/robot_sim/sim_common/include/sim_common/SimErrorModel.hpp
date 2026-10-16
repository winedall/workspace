#pragma once

// 仿真误差注入：把 MuJoCo 的"真值"变成"传感器读数"，并模拟执行延迟与反馈上报延迟。
//
// 语义（与真机一致）：本类替换的是"编码器 + 驱动器"这一层，误差进入仿真内部的
// PD 回路。因此稳态下关节**真值**偏离指令（q_true ≈ q_des − 零位偏置），而对外
// **发布值等于指令值**——这正是真机上零位标定错时的现象。
//
// 注入点只有三处：
//   MjInterface::ReadJointState → PerturbJoint（关节测量）
//   MjInterface::ReadState      → PerturbImu  （IMU 测量）
//     —— 上面两处在 q_rel_/dq_rel_/acc_ 的唯一来源里，发布路径与各包 ComputeControl
//        的 PD 路径都从它们取值，所以一处注入同时覆盖"发布值"与"控制回路"。
//   SimErrorModel::ApplyTorqueDelay → 由 MujocoSimBase::PostComputeControl 调用
//     （d->ctrl 刚写好、MuJoCo 还没读它算 actuator_force）。
//   SimErrorModel::ApplyFeedbackDelay → 由发布填充处调用（SimNodeBase::FillJointStates /
//     RobotChannel::PublishTick），**只作用于发布出去的那份快照，不碰 PD 回路**。
//
// 抽样规则：
//   - 常值误差（零位偏置/零偏/安装角/姿态误差）在 Init() 里抽一次，全进程不变，
//     GUI Reload 后仍是同一台"个体"；
//   - 白噪声按 d->time 键控，每个仿真步抽一次并缓存 —— 同一步内 PD 与发布必须读到
//     同一个采样（同一个编码器），这也是"同 seed + 同 dt 逐位可复现"的前提。
//
// 明确不做（non-goals，需要时另开）：
//   - 零偏是常值，没有随机游走 / 温漂；
//   - 没有编码器量化误差；
//   - 不修改 mjModel 物理参数（质量/质心/摩擦），不调 mj_setConst。
//
// 本文件保持与 ROS 无关（只依赖 Eigen + mujoco），这样 MjInterface.hpp 可以直接
// include 它而不会把 ROS 消息层拖进物理接口层。

#include <cstddef>
#include <random>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sim_common/SimJointCommand.hpp"

namespace sim_common {

// YAML `errors.*` 分组的映射。所有默认值为 0，seed == 0 表示整体关闭（含延迟）。
struct SimErrorOptions {
  unsigned int seed{0};

  // ---- 关节测量（编码器）----
  // 可选：按关节显式指定零位偏置（长度 = 关节数）；非空时覆盖 joint_zero_offset_std_rad。
  std::vector<double> joint_zero_offset_rad;
  double joint_zero_offset_std_rad{0.0};
  double joint_pos_noise_std_rad{0.0};
  double joint_vel_noise_std_rad_s{0.0};

  // ---- IMU ----
  double mount_std_deg{0.0};         // 安装角误差（三轴独立）
  double gyro_bias_std_rad_s{0.0};
  double acc_bias_std_m_s2{0.0};
  double gyro_noise_std_rad_s{0.0};
  double acc_noise_std_m_s2{0.0};
  double att_bias_std_deg{0.0};      // AHRS 姿态估计常值误差（只作用于姿态输出）
  double att_noise_std_deg{0.0};

  // ---- 时序（力矩延迟：回路内）----
  // 延迟的是 d->ctrl，也就是"算完的力矩"。它落在**回路内**：阻尼项 kd·q̇ 会被一起延迟，
  // 等价于"延迟阻尼"，直接消耗相位裕度，所以量级必须很小（亚毫秒~几毫秒）。
  // 实测的 25~40 ms 指令延迟**不属于这里**，见下面的 command_delay_ms。
  double torque_delay_ms{0.0};
  double drop_probability{0.0};      // 每个仿真步丢一帧的概率 [0,1)

  // ---- 时序（指令延迟：参考路径）----
  // 延迟的是整份指令帧（目标位置/速度/前馈/增益），而 PD 仍用**当前**状态计算——与真机
  // 一致：驱动器收到的是 Δ 之前的目标，但它用自己本地的编码器闭合 PD。因为延迟在回路外，
  // 只让跟踪滞后、不损失相位裕度，所以可以给到几十毫秒。
  // 这是"指令发布 → 力矩体现"那 25~40 ms 的正确落点（含总线/网关管线）。
  double command_delay_ms{0.0};

  // ---- 时序（反馈上行）----
  // 真机上驱动以固定频率发 JointStates，但底层电机快照刷新得更慢（实测 86.8~199 Hz，
  // 而发布是 200 Hz），表现为整帧数值逐位重复。feedback_hold_ms 复现这个"采样保持"，
  // feedback_delay_ms 复现反馈链路的纯延迟。两者都只改发布出去的那份快照，不动 PD 回路。
  //
  // 注意：采样保持本身已隐含 hold/2 的平均滞后（11.5 ms → +5.8 ms），标定时应只用
  // 其中一个去拟合同一个观测滞后，同时开两个会重复计数。
  double feedback_hold_ms{0.0};     // 0 = 每次发布都刷新（不保持）
  double feedback_delay_ms{0.0};    // 0 = 不延迟
};

// 力矩延迟环：把"本步算出的执行器力矩"延后 N 步再交给 MuJoCo。
//
// **它落在回路内**：延迟的是算完的力矩，等于把阻尼项 kd·q̇ 也一起延迟——这是"延迟阻尼"，
// 直接消耗相位裕度。所以量级必须很小（亚毫秒~几毫秒），给到十几毫秒简单关节控制就会抖，
// 几十毫秒必然发散。想复现实测的 25~40 ms 指令延迟请用 CommandDelay，那在回路外。
//
// 只操作 double 数组与执行器下标，与 mjData 解耦，便于单元测试。
//
// 时间语义必须分三态（只判"时间前进"会漏掉 GUI Reset）：
//   前进 → 推入本步值，弹出 N 步前的值写回；
//   相等 → 只把当前施加值写回（暂停时每轮的 mj_forward、/reload_* 后的 mj_forward）；
//   倒退 → 清环重填（GUI Reset 把仿真时间拨回 0、拖动历史时间轴）。
class TorqueDelay {
 public:
  // steps <= 0 时整个类退化为旁路（不读写任何值）。
  void Init(int steps, double drop_probability, unsigned int seed);
  // 把 count 个执行器（下标 actuator_ids）的 ctrl 原地替换为延迟后的值。
  void Apply(std::size_t count, const int* actuator_ids, double* ctrl, double time);

  int steps() const { return steps_; }
  bool bypass() const { return steps_ <= 0; }

 private:
  // 用当前值填满整环并清零进度：惰性初始化与"时间倒退"共用，保证不会吐出
  // 上一轮运行的陈旧力矩，也不会在启动/Reload 后引入 N 步零力矩瞬态。
  void Refill(std::size_t count, const int* actuator_ids, double* ctrl);

  int steps_{0};
  std::vector<double> ring_;      // steps_ × count_ 的环形缓冲
  std::vector<double> applied_;   // 当前施加值（"时间未前进"时原样写回）
  std::size_t count_{0};
  std::size_t write_index_{0};    // 指向环中最旧的一帧
  double last_time_{0.0};
  bool started_{false};
  std::mt19937 drop_rng_{0};
  std::uniform_real_distribution<double> drop_dist_{0.0, 1.0};
  double drop_probability_{0.0};
};

// 指令延迟：把整份"指令帧"（目标位置/速度/前馈/增益）按仿真时间延后 Δ 再交给控制器，
// 而 PD 仍用**当前**的 q/dq 计算。
//
// 这是与 TorqueDelay 的本质区别，也是它能给到几十毫秒的原因：
//   真机  τ = kp·(q_target(t−Δ) − q_enc(t)) + kd·(v_target(t−Δ) − v_enc(t)) + τ_ff(t−Δ)
//                 ↑ 延迟的只有参考          ↑ 编码器在驱动器本地，是新鲜的
//   本类  τ = PD(延迟后的指令帧, 当前 q/dq)   —— 延迟在参考路径上（回路外），只让跟踪
//                                              滞后，不消耗相位裕度
//   TorqueDelay 延迟的是算完的力矩，等于把 kd·q̇ 也一起延迟（延迟阻尼），直接吃相位裕度
//
// 实测的"指令发布 → 力矩体现"25~40 ms 属于本类，不属于 TorqueDelay。
//
// 按时间索引而不是按步：UpdateCommandCache 只在**有新指令时**被调用（~200 Hz），
// 而 ComputeControl 每步都读（500 Hz），两者频率不同，只有按 d->time 索引才对齐。
// 时间三态语义与 TorqueDelay 一致（前进/相等/倒退）。
class CommandDelay {
 public:
  // delay_seconds <= 0 或 joint_count == 0 时退化为旁路（Apply 只做拷贝）。
  void Init(std::size_t joint_count, double delay_seconds);
  bool bypass() const { return !has_delay_ || joint_count_ == 0; }
  double delaySeconds() const { return delay_seconds_; }

  // fresh 是刚合并好的"最新参考"，out 写回 (time − Δ) 时刻的那一帧。
  void Apply(double time, const std::vector<SimJointCommand>& fresh,
             std::vector<SimJointCommand>& out);

 private:
  // 重新填满并清零进度：Init 与"时间倒退"共用，保证启动/Reload 后不吐陈旧帧。
  void Reset(double time, const std::vector<SimJointCommand>& fresh,
             std::vector<SimJointCommand>& out);
  void PushSnapshot(double time, const std::vector<SimJointCommand>& fresh);
  // 取 (time − Δ) 之前最新的那一帧；环里没有更早的则退回最旧一帧。
  void LookupDelayed(double time, std::vector<SimJointCommand>& out) const;

  double delay_seconds_{0.0};
  bool has_delay_{false};
  std::size_t joint_count_{0};

  // 历史环按指令到达节拍压入。128 份在 200 Hz 下约 0.64 s、500 Hz 下约 0.26 s，
  // 远大于任何合理的指令延迟（实测最大 40 ms），也远小于每实例的内存预算。
  static constexpr std::size_t kRingCapacity = 128;
  std::vector<double> ring_time_;
  std::vector<std::vector<SimJointCommand>> ring_;
  std::size_t ring_head_{0};   // 下一个写入位置
  std::size_t ring_count_{0};  // 已填充的有效份数
  double last_time_{0.0};
  bool started_{false};
};

// 反馈上行的"采样保持 + 纯延迟"，复现真机上"驱动按固定节拍发布、而底层电机快照
// 刷新得更慢"这一现象（实测底层刷新 86.8~199 Hz，发布固定 200 Hz，表现为整帧逐位重复）。
//
// 真机链路是两级：总线每 hold 周期轮询一次，拿回来的是 delay 之前的数据，驱动再按发布
// 节拍原样重发。所以本类也分两级——先从历史环取 (time - delay) 的快照，再按 hold 周期
// 决定是否把"上报寄存器"推进到该快照。hold 与 delay 都为 0 时整个类退化为旁路。
//
// 与 TorqueDelay 的两点关键区别：
//   1. **按仿真时间而不是整数步**。11.5 ms 不是 timestep=2 ms 的整数倍，而且真正的量化
//      来自发布节拍（5 ms 栅格），所以既无法也不该取整到步。
//   2. **刷新时刻用"绝对下次时刻累加"**（next_sample_time_ += hold），不是"距上次刷新
//      超过 hold 就刷新"。后者会被发布栅格锁相：5 ms 栅格配 11.5 ms 保持恒得 15 ms 间隔
//      （66.7 Hz），而复现实测的 10/15 ms 混合（均值 11.52 ms = 86.8 Hz）恰恰靠刷新时刻
//      相对栅格持续漂移（11.5/5 = 2.3，每轮漂 1.5 ms）。驱动侧 ethercatWorkerLoop 的
//      next_wake 累加是同一个做法。
//
// 时间语义沿用 TorqueDelay 的三态（见上）：前进按上述规则推进；相等时原样写回上一次
// 的上报值（暂停时每轮 mj_forward 都会走一遍发布路径）；倒退时清空重采（GUI Reset）。
class FeedbackDelay {
 public:
  // hold_seconds <= 0 且 delay_seconds <= 0 时退化为旁路（Apply 只做拷贝）。
  void Init(double hold_seconds, double delay_seconds);
  // 重新填满并清零进度：Init 与"时间倒退"共用，保证启动/Reload 后不会吐出零值或陈旧值。
  void Reset(double time, const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
             const Eigen::VectorXd& tau);

  bool bypass() const { return !has_hold_ && !has_delay_; }
  bool has_hold() const { return has_hold_; }
  bool has_delay() const { return has_delay_; }
  double holdSeconds() const { return hold_seconds_; }
  double delaySeconds() const { return delay_seconds_; }

  // 把当前真值 (q, dq, tau) 转换成"驱动上报值"写入 out_*。bypass() 时逐位拷贝。
  void Apply(double time, const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
             const Eigen::VectorXd& tau, Eigen::VectorXd& q_out, Eigen::VectorXd& dq_out,
             Eigen::VectorXd& tau_out);

 private:
  // 压入一份真值快照（仅 has_delay_ 时需要历史）。
  void PushSnapshot(double time, const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                    const Eigen::VectorXd& tau);
  // 取 (time - delay_seconds_) 之前最新的那份快照；环里没有更早的则退回最旧一份。
  void LookupDelayed(double time, Eigen::VectorXd& q, Eigen::VectorXd& dq,
                     Eigen::VectorXd& tau) const;

  double hold_seconds_{0.0};
  double delay_seconds_{0.0};
  bool has_hold_{false};
  bool has_delay_{false};

  // 历史环（仅 has_delay_ 时使用）。按发布节拍压入，容量足够覆盖常见延迟
  // （512 份 × 5 ms 发布周期 ≈ 2.5 s）。
  static constexpr std::size_t kRingCapacity = 512;
  std::vector<double> ring_time_;
  std::vector<Eigen::VectorXd> ring_q_;
  std::vector<Eigen::VectorXd> ring_dq_;
  std::vector<Eigen::VectorXd> ring_tau_;
  std::size_t ring_head_{0};  // 下一个写入位置
  std::size_t ring_count_{0}; // 已填充的有效份数

  // 上报寄存器
  Eigen::VectorXd held_q_, held_dq_, held_tau_;
  double next_sample_time_{0.0};
  double last_time_{0.0};
  bool started_{false};

  // 自检：hold 必须大于发布周期才有意义（配 3 ms 而发布是 5 ms，等于每拍都刷新）。
  // 发布周期 SimErrorModel 不知道，只能从实际调用间隔观测，做法同 ApplyTorqueDelay
  // 里的 RK4 自检。
  double min_forward_dt_{0.0};
  int forward_count_{0};
  bool hold_warned_{false};
};

class SimErrorModel {
 public:
  // 读取 YAML 参数；seed == 0 时 enabled() 为 false，后续所有注入都是空操作。
  void Configure(const SimErrorOptions& options);

  // 模型加载后调用（初次加载与 GUI Reload 都会走）。motor_ids 为执行器下标，
  // timestep 取 m->opt.timestep（延迟按整数个仿真步取整）。seed_index 用于多机
  // 派生不同种子（单机传 0），保证 6 台机器人不是完全相同的误差。
  void Init(std::size_t joint_count, const std::vector<int>& motor_ids, double timestep,
            unsigned int seed_index = 0);

  bool enabled() const { return enabled_; }
  bool hasTorqueDelay() const { return !torque_delay_.bypass(); }
  int torqueDelaySteps() const { return torque_delay_.steps(); }
  bool hasFeedbackDelay() const { return !feedback_.bypass(); }
  bool hasCommandDelay() const { return !command_delay_.bypass(); }
  const FeedbackDelay& feedbackDelay() const { return feedback_; }

  // 启动日志用：一行摘要；Warning() 非空时调用方应以 WARN 级别打印。
  std::string Summary() const;
  std::string Warning() const { return warning_; }

  // ---- 测量误差注入（MjInterface 的读取函数调用）----
  // 关节 i：q_meas = q + 零位常值偏置 + 位置白噪声，dq_meas 同理。
  void PerturbJoint(std::size_t i, double time, double& q, double& dq);
  // IMU：quat_xyzw 指向 4 个分量，顺序 (x, y, z, w)，与 SystemQ 的 q[3..6] 一致。
  //
  // 复合规则（统一右乘机体系，小角下与左乘差二阶量）：
  //   mount  : 安装角误差（机体系 → IMU 系），姿态右乘、向量左乘；
  //   att    : AHRS 姿态估计误差，**只作用于姿态**，不动陀螺/加计；
  //   q_pub  = q_true ⊗ q_mount ⊗ q_att ⊗ q_noise   （输出归一化）
  //   gyro   = q_mount · gyro_true + b_g + n_g
  //   acc    = q_mount · acc_true  + b_a + n_a
  void PerturbImu(double time, double* quat_xyzw, double* gyro, Eigen::Vector3d& acc);

  // ---- 执行延迟（MujocoSimBase::PostComputeControl 调用）----
  void ApplyTorqueDelay(mjData* data);

  // ---- 指令延迟（SimRosNodeBase::UpdateCommandCache 调用）----
  // 把"刚合并好的最新参考"延后 command_delay_ms 再写进 out（供 ComputeControl 读取）。
  // 放在通信层是因为那是所有仿真包唯一共用的指令写入点；PD 回路本身不受影响。
  void ApplyCommandDelay(double time, const std::vector<SimJointCommand>& fresh,
                         std::vector<SimJointCommand>& out);

  // ---- 反馈上行采样保持 / 延迟（发布填充处调用，只作用于发布出去的那份快照）----
  // 调用方（SimNodeBase::FillJointStates / RobotChannel::PublishTick）必须把结果写进
  // **独立向量**再交给 FillJointStatesMessage，不能原地改 q_rel_——那会污染 PD 回路。
  void ApplyFeedbackDelay(double time, const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                          const Eigen::VectorXd& tau, Eigen::VectorXd& q_out,
                          Eigen::VectorXd& dq_out, Eigen::VectorXd& tau_out);

  // 诊断/可观测性（测试用）
  double jointZeroOffset(std::size_t i) const;
  const Eigen::Vector3d& gyroBias() const { return gyro_bias_; }
  const Eigen::Vector3d& accBias() const { return acc_bias_; }
  const Eigen::Quaterniond& mountQuat() const { return mount_quat_; }
  const Eigen::Quaterniond& attitudeBiasQuat() const { return att_bias_quat_; }

 private:
  // 白噪声按 d->time 键控：时间变化时才重新抽样，同一步内所有读取共享同一组样本。
  void RefreshStepNoise(double time);
  // 抽一个三轴小角误差对应的四元数（每轴独立同分布）。
  Eigen::Quaterniond DrawSmallRotation(double std_deg);

  SimErrorOptions options_{};
  bool enabled_{false};
  bool joint_errors_active_{false};  // 关节误差是否真的会改变数值（全 0 时跳过）
  bool imu_errors_active_{false};

  std::size_t joint_count_{0};
  std::vector<int> motor_ids_;
  double timestep_{0.0};

  // 常值误差（Init 时抽一次）
  std::vector<double> joint_zero_offset_;
  Eigen::Vector3d gyro_bias_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acc_bias_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond mount_quat_{Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond att_bias_quat_{Eigen::Quaterniond::Identity()};

  // 每步白噪声（按时间缓存）
  double noise_time_{0.0};
  bool noise_valid_{false};
  std::vector<double> joint_pos_noise_;
  std::vector<double> joint_vel_noise_;
  Eigen::Vector3d gyro_noise_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acc_noise_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond att_noise_quat_{Eigen::Quaterniond::Identity()};

  TorqueDelay torque_delay_;
  CommandDelay command_delay_;
  FeedbackDelay feedback_;
  std::mt19937 rng_{0};
  std::normal_distribution<double> normal_{0.0, 1.0};
  std::string warning_;
  // 延迟按"N 个仿真步"实现，所以实际延迟依赖"每步恰好触发一次控制回调"。
  // 若积分器被改成 RK4（每步 4 次回调）或 timestep 与时序不符，这里报警一次。
  double check_time_{0.0};
  bool check_time_valid_{false};
  bool timestep_warned_{false};
};

}  // namespace sim_common
