#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "sim_common/SimErrorModel.hpp"

namespace {

using sim_common::CommandDelay;
using sim_common::SimJointCommand;
using sim_common::TorqueDelay;
using sim_common::FeedbackDelay;
using sim_common::SimErrorModel;
using sim_common::SimErrorOptions;

constexpr double kDt = 0.002;  // 与 Z01 系列模型一致（500 Hz）

// 复现仿真实际的发布时刻序列。发布闸门是 `time_us >= next_pub_time_us_`（每次 += 5000），
// 而 d->time 以 2 ms 递增，所以实际触发在 0/6/10/16/20/26 ms…，即 6/4 ms 交替，并不是
// 整齐的 5 ms。写断言时不要假设 5.0 ms。
std::vector<double> SimPublishTimes(int count) {
  std::vector<double> out;
  std::int64_t next_us = 0;
  const std::int64_t limit_us = static_cast<std::int64_t>(count) * 20000;
  for (std::int64_t t_us = 0; static_cast<int>(out.size()) < count && t_us < limit_us; t_us += 2000) {
    if (t_us >= next_us) {
      next_us += 5000;
      out.push_back(static_cast<double>(t_us) * 1e-6);
    }
  }
  return out;
}

// 打开误差注入但所有标准差留 0：用于测试"参数为 0 时不产生任何数值改动"。
SimErrorOptions EnabledBaseOptions() {
  SimErrorOptions o;
  o.seed = 1;
  return o;
}

// 两个 double 是否逐位相同（含 -0.0 与 NaN 的位模式）。
bool SameBits(double a, double b) {
  return std::memcmp(&a, &b, sizeof(double)) == 0;
}

}  // namespace

//--------------------------------------- 关闭与可复现性 -------------------------------------------

TEST(ErrorModel, ZeroSeedDisablesEverythingBitExactly) {
  // errors.seed=0 = 整体关闭：即使把各标准差配成非 0，也不能改动任何数值。
  SimErrorOptions options;
  options.seed = 0;
  options.joint_zero_offset_std_rad = 0.05;
  options.joint_pos_noise_std_rad = 0.01;
  options.joint_vel_noise_std_rad_s = 0.1;
  options.mount_std_deg = 5.0;
  options.gyro_noise_std_rad_s = 0.01;
  options.torque_delay_ms = 10.0;

  SimErrorModel model;
  model.Configure(options);
  model.Init(2, {0, 1}, kDt);
  EXPECT_FALSE(model.enabled());
  EXPECT_FALSE(model.hasTorqueDelay());

  double q = -0.0;  // -0.0 专门用来暴露 "+0.0" 这类会改位模式的运算
  double dq = 1.25;
  model.PerturbJoint(0, 0.0, q, dq);
  EXPECT_TRUE(SameBits(q, -0.0));
  EXPECT_TRUE(SameBits(dq, 1.25));

  double quat[4] = {0.0, 0.0, 0.0, 1.0};
  double gyro[3] = {0.1, 0.2, 0.3};
  Eigen::Vector3d acc(1.0, 2.0, 3.0);
  model.PerturbImu(0.0, quat, gyro, acc);
  EXPECT_TRUE(SameBits(quat[3], 1.0));
  EXPECT_TRUE(SameBits(gyro[0], 0.1));
  EXPECT_TRUE(SameBits(acc[1], 2.0));

  // 关闭时执行延迟整体旁路：ApplyTorqueDelay 直接返回（连 data 都不需要）。
  model.ApplyTorqueDelay(nullptr);
  EXPECT_EQ(model.torqueDelaySteps(), 0);
  EXPECT_EQ(model.jointZeroOffset(0), 0.0);
  EXPECT_EQ(model.Summary(), "disabled (errors.seed = 0)");
}

TEST(ErrorModel, SameSeedDrawsSameIndividualAndDifferentIndexDiffers) {
  // 同 seed 两次 Init 必须是同一台"个体"（GUI Reload 后零位不变）；
  // 多机用 seed_index 区分机器人，不能 6 台完全一样。
  SimErrorOptions options = EnabledBaseOptions();
  options.joint_zero_offset_std_rad = 0.02;
  options.gyro_bias_std_rad_s = 0.01;
  options.mount_std_deg = 2.0;

  SimErrorModel a;
  a.Configure(options);
  a.Init(3, {0, 1, 2}, kDt, 0);

  SimErrorModel b;
  b.Configure(options);
  b.Init(3, {0, 1, 2}, kDt, 0);

  SimErrorModel c;
  c.Configure(options);
  c.Init(3, {0, 1, 2}, kDt, 1);

  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_DOUBLE_EQ(a.jointZeroOffset(i), b.jointZeroOffset(i));
  }
  EXPECT_TRUE(a.gyroBias().isApprox(b.gyroBias(), 0.0));
  EXPECT_TRUE(a.mountQuat().coeffs().isApprox(b.mountQuat().coeffs(), 0.0));

  bool any_different = false;
  for (std::size_t i = 0; i < 3; ++i) {
    any_different = any_different || (a.jointZeroOffset(i) != c.jointZeroOffset(i));
  }
  EXPECT_TRUE(any_different);
}

//------------------------------------------ 关节测量 ----------------------------------------------

TEST(ErrorModel, ZeroOffsetIsConstantAcrossSteps) {
  // 只给零位偏置、不给噪声：测量值与真值之差必须是常数（与时间无关）。
  SimErrorOptions options = EnabledBaseOptions();
  options.joint_zero_offset_std_rad = 0.02;

  SimErrorModel model;
  model.Configure(options);
  model.Init(1, {0}, kDt);

  const double offset = model.jointZeroOffset(0);
  EXPECT_NE(offset, 0.0);
  double first_delta = 0.0;
  for (int step = 0; step < 1000; ++step) {
    double q = 0.5;
    double dq = 0.0;
    model.PerturbJoint(0, step * kDt, q, dq);
    if (step == 0) {
      first_delta = q - 0.5;
      EXPECT_NEAR(first_delta, offset, 1e-12);
    } else {
      // 常数偏置与时间无关：每一步的偏差必须与第一步逐位相同。
      EXPECT_DOUBLE_EQ(q - 0.5, first_delta);
    }
    EXPECT_DOUBLE_EQ(dq, 0.0);
  }
}

TEST(ErrorModel, ExplicitZeroOffsetOverridesStd) {
  SimErrorOptions options = EnabledBaseOptions();
  options.joint_zero_offset_rad = {0.01, -0.02};
  options.joint_zero_offset_std_rad = 0.5;  // 被显式列表覆盖，不应生效

  SimErrorModel model;
  model.Configure(options);
  model.Init(2, {0, 1}, kDt);
  EXPECT_DOUBLE_EQ(model.jointZeroOffset(0), 0.01);
  EXPECT_DOUBLE_EQ(model.jointZeroOffset(1), -0.02);
}

TEST(ErrorModel, NoiseIsKeyedBySimTimeNotCallCount) {
  // 同一步内的重复读取必须返回同一组样本（PD 与发布读的是同一个编码器），
  // 且重复调用不能推进随机数序列。
  SimErrorOptions options = EnabledBaseOptions();
  options.joint_pos_noise_std_rad = 0.01;

  SimErrorModel once;
  once.Configure(options);
  once.Init(1, {0}, kDt);

  double q_first = 0.0;
  double dq = 0.0;
  once.PerturbJoint(0, 0.0, q_first, dq);       // 第 1 次（t=0）
  double q_next = 0.0;
  once.PerturbJoint(0, kDt, q_next, dq);        // 下一步

  SimErrorModel repeated;
  repeated.Configure(options);
  repeated.Init(1, {0}, kDt);
  for (int i = 0; i < 100; ++i) {
    double q = 0.0;
    double dq_local = 0.0;
    repeated.PerturbJoint(0, 0.0, q, dq_local);  // 同一时刻反复调用
    EXPECT_DOUBLE_EQ(q, q_first);
  }
  double q_after = 0.0;
  repeated.PerturbJoint(0, kDt, q_after, dq);
  EXPECT_DOUBLE_EQ(q_after, q_next);  // 随机数序列没有被重复调用推进
}

TEST(ErrorModel, NoiseStatisticsMatchConfiguredSigma) {
  SimErrorOptions options = EnabledBaseOptions();
  options.joint_pos_noise_std_rad = 0.01;
  options.joint_vel_noise_std_rad_s = 0.2;

  SimErrorModel model;
  model.Configure(options);
  model.Init(1, {0}, kDt);

  constexpr int kN = 20000;
  double sum_q = 0.0;
  double sum_sq = 0.0;
  double sum_dq = 0.0;
  double sum_dq_sq = 0.0;
  for (int i = 0; i < kN; ++i) {
    double q = 0.0;
    double dq = 0.0;
    model.PerturbJoint(0, i * kDt, q, dq);
    sum_q += q;
    sum_sq += q * q;
    sum_dq += dq;
    sum_dq_sq += dq * dq;
  }
  const double mean_q = sum_q / kN;
  const double std_q = std::sqrt(sum_sq / kN - mean_q * mean_q);
  const double mean_dq = sum_dq / kN;
  const double std_dq = std::sqrt(sum_dq_sq / kN - mean_dq * mean_dq);

  // 2e4 样本下样本标准差相对误差约 0.5%，5%/10% 容差足够稳。
  EXPECT_NEAR(mean_q, 0.0, 0.05 * options.joint_pos_noise_std_rad);
  EXPECT_NEAR(std_q, options.joint_pos_noise_std_rad, 0.1 * options.joint_pos_noise_std_rad);
  EXPECT_NEAR(mean_dq, 0.0, 0.05 * options.joint_vel_noise_std_rad_s);
  EXPECT_NEAR(std_dq, options.joint_vel_noise_std_rad_s, 0.1 * options.joint_vel_noise_std_rad_s);
}

//--------------------------------------------- IMU ------------------------------------------------

TEST(ErrorModel, MountErrorRotatesGyroAccAndAttitude) {
  // 安装角误差：陀螺/加计按 mount 旋转，姿态右乘同一个四元数。
  SimErrorOptions options = EnabledBaseOptions();
  options.mount_std_deg = 20.0;  // 只开安装角，其余全 0

  SimErrorModel model;
  model.Configure(options);
  model.Init(1, {0}, kDt);

  const Eigen::Vector3d gyro_true(0.1, -0.2, 0.3);
  const Eigen::Vector3d acc_true(1.0, 2.0, -9.8);
  double quat[4] = {0.0, 0.0, 0.0, 1.0};
  double gyro[3] = {gyro_true.x(), gyro_true.y(), gyro_true.z()};
  Eigen::Vector3d acc = acc_true;

  model.PerturbImu(0.0, quat, gyro, acc);

  const Eigen::Vector3d expected_gyro = model.mountQuat() * gyro_true;
  const Eigen::Vector3d expected_acc = model.mountQuat() * acc_true;
  EXPECT_TRUE(Eigen::Vector3d(gyro[0], gyro[1], gyro[2]).isApprox(expected_gyro, 1e-12));
  EXPECT_TRUE(acc.isApprox(expected_acc, 1e-12));

  // 姿态 = q_true ⊗ q_mount，且保持归一化与 (x,y,z,w) 顺序。
  const Eigen::Quaterniond expected_quat = Eigen::Quaterniond::Identity() * model.mountQuat();
  EXPECT_NEAR(quat[3], expected_quat.w(), 1e-12);
  EXPECT_NEAR(quat[0], expected_quat.x(), 1e-12);
  EXPECT_NEAR(quat[1], expected_quat.y(), 1e-12);
  EXPECT_NEAR(quat[2], expected_quat.z(), 1e-12);
}

TEST(ErrorModel, AttitudeNoiseStaysNormalizedAndOrdered) {
  SimErrorOptions options = EnabledBaseOptions();
  options.att_noise_std_deg = 3.0;
  options.att_bias_std_deg = 2.0;

  SimErrorModel model;
  model.Configure(options);
  model.Init(1, {0}, kDt);

  for (int step = 0; step < 200; ++step) {
    // 一个非平凡的真值姿态（绕 x 转 30°），顺序 (x,y,z,w)。
    const Eigen::Quaterniond q_true(Eigen::AngleAxisd(0.5236, Eigen::Vector3d::UnitX()));
    double quat[4] = {q_true.x(), q_true.y(), q_true.z(), q_true.w()};
    double gyro[3] = {0.0, 0.0, 0.0};
    Eigen::Vector3d acc(0.0, 0.0, -9.81);
    model.PerturbImu(step * kDt, quat, gyro, acc);

    const Eigen::Quaterniond q_pub(quat[3], quat[0], quat[1], quat[2]);
    EXPECT_NEAR(q_pub.norm(), 1.0, 1e-12);
    // 姿态误差是小角：三轴各 3° 噪声 + 2° 常值，合矢量量级 ~9°，
    // 20°（0.35 rad）是"没有退化/发散"的宽松上界。
    EXPECT_LT(q_true.angularDistance(q_pub), 0.35);
  }
}

TEST(ErrorModel, AttitudeBiasDoesNotTouchGyroAcc) {
  // 姿态估计误差只作用于姿态输出——这是与安装角误差的语义分界。
  SimErrorOptions options = EnabledBaseOptions();
  options.att_bias_std_deg = 5.0;
  options.att_noise_std_deg = 1.0;

  SimErrorModel model;
  model.Configure(options);
  model.Init(1, {0}, kDt);

  double quat[4] = {0.0, 0.0, 0.0, 1.0};
  double gyro[3] = {0.1, 0.2, 0.3};
  Eigen::Vector3d acc(1.0, 2.0, 3.0);
  model.PerturbImu(0.0, quat, gyro, acc);

  EXPECT_DOUBLE_EQ(gyro[0], 0.1);
  EXPECT_DOUBLE_EQ(gyro[1], 0.2);
  EXPECT_DOUBLE_EQ(gyro[2], 0.3);
  EXPECT_DOUBLE_EQ(acc[0], 1.0);
  EXPECT_DOUBLE_EQ(acc[2], 3.0);
  // 姿态确实被改动了（否则这条测试没有意义）。
  EXPECT_GT(std::abs(quat[0]) + std::abs(quat[1]) + std::abs(quat[2]), 1e-6);
}

//------------------------------------------- 执行延迟 ---------------------------------------------

TEST(TorqueDelay, ZeroStepsIsBypass) {
  double ctrl[1] = {1.5};
  const int ids[1] = {0};
  TorqueDelay delay;
  delay.Init(0, 0.0, 1);
  delay.Apply(1, ids, ctrl, 0.0);
  EXPECT_DOUBLE_EQ(ctrl[0], 1.5);
}

TEST(TorqueDelay, ShiftsByExactStepCount) {
  // 阶跃输入应当恰好晚 N 步出现：N=2 时，第 3 步推入的 1.0 在第 5 步才被施加。
  constexpr int kSteps = 2;
  const int ids[1] = {0};
  TorqueDelay delay;
  delay.Init(kSteps, 0.0, 1);

  std::vector<double> out;
  for (int step = 0; step < 7; ++step) {
    double ctrl[1] = {step >= 3 ? 1.0 : 0.0};
    delay.Apply(1, ids, ctrl, step * kDt);
    out.push_back(ctrl[0]);
  }

  EXPECT_DOUBLE_EQ(out[0], 0.0);  // 首帧用当前值填满环，不引入启动瞬态
  EXPECT_DOUBLE_EQ(out[4], 0.0);
  EXPECT_DOUBLE_EQ(out[5], 1.0);  // 第 3 步的值延迟 2 步
  EXPECT_DOUBLE_EQ(out[6], 1.0);
}

TEST(TorqueDelay, DoesNotAdvanceWhenTimeStandsStill) {
  // 暂停时每轮的 mj_forward 会用同一个 d->time 调用：不能推进环，也不能把
  // 新算出的力矩放行（否则暂停期间力矩会"漏"过去）。
  const int ids[1] = {0};
  TorqueDelay delay;
  delay.Init(2, 0.0, 1);

  double first[1] = {0.5};
  delay.Apply(1, ids, first, 0.0);  // 填环

  for (int i = 0; i < 10; ++i) {
    double ctrl[1] = {9.0};  // 时间不变但算出了新值
    delay.Apply(1, ids, ctrl, 0.0);
    EXPECT_DOUBLE_EQ(ctrl[0], 0.5);  // 施加值保持，不推进
  }
}

TEST(TorqueDelay, FlushesWhenTimeGoesBackwards) {
  // GUI Reset 会把 d->time 拨回 0：此时必须清环重填，不能吐出上一轮运行的力矩。
  const int ids[1] = {0};
  TorqueDelay delay;
  delay.Init(2, 0.0, 1);

  for (int step = 0; step < 6; ++step) {
    double ctrl[1] = {1.0};
    delay.Apply(1, ids, ctrl, step * kDt);
  }

  double after_reset[1] = {0.25};
  delay.Apply(1, ids, after_reset, 0.0);  // 时间倒退
  EXPECT_DOUBLE_EQ(after_reset[0], 0.25);

  // 倒退之后按新的时间线继续，不应残留 1.0。
  double next[1] = {0.5};
  delay.Apply(1, ids, next, kDt);
  EXPECT_DOUBLE_EQ(next[0], 0.25);
}

TEST(TorqueDelay, FullDropProbabilityHoldsPreviousValue) {
  const int ids[1] = {0};
  TorqueDelay delay;
  delay.Init(2, 1.0, 1);  // 每个仿真步都丢帧

  double first[1] = {0.5};
  delay.Apply(1, ids, first, 0.0);

  for (int step = 1; step < 10; ++step) {
    double ctrl[1] = {static_cast<double>(step)};
    delay.Apply(1, ids, ctrl, step * kDt);
    EXPECT_DOUBLE_EQ(ctrl[0], 0.5);  // 驱动器保持上一帧
  }
}

TEST(TorqueDelay, HandlesMultipleActuatorsIndependently) {
  const int ids[3] = {4, 7, 9};
  TorqueDelay delay;
  delay.Init(1, 0.0, 1);

  double ctrl[10] = {0.0};
  ctrl[4] = 1.0;
  ctrl[7] = 2.0;
  ctrl[9] = 3.0;
  delay.Apply(3, ids, ctrl, 0.0);  // 填环
  EXPECT_DOUBLE_EQ(ctrl[7], 2.0);

  // 下一帧改值，本帧应施加上一帧的值；未列出的执行器不受影响。
  ctrl[0] = 42.0;
  ctrl[4] = 10.0;
  ctrl[7] = 20.0;
  ctrl[9] = 30.0;
  delay.Apply(3, ids, ctrl, kDt);
  EXPECT_DOUBLE_EQ(ctrl[4], 1.0);
  EXPECT_DOUBLE_EQ(ctrl[7], 2.0);
  EXPECT_DOUBLE_EQ(ctrl[9], 3.0);
  EXPECT_DOUBLE_EQ(ctrl[0], 42.0);
}

//----------------------------------- 反馈上行采样保持 / 延迟 -------------------------------------

TEST(FeedbackDelay, BypassIsBitExact) {
  // 两个参数都为 0 = 旁路：只做拷贝，不做任何数值运算（-0.0 的位模式也要保住）。
  FeedbackDelay none;
  none.Init(0.0, 0.0);
  EXPECT_TRUE(none.bypass());

  Eigen::VectorXd q(2), dq(2), tau(2), qo(2), dqo(2), tauo(2);
  q << -0.0, 1.25;
  dq << 2.5, -3.75;
  tau << 0.0, -0.0;
  none.Apply(0.0, q, dq, tau, qo, dqo, tauo);
  for (Eigen::Index i = 0; i < 2; ++i) {
    EXPECT_TRUE(SameBits(qo[i], q[i]));
    EXPECT_TRUE(SameBits(dqo[i], dq[i]));
    EXPECT_TRUE(SameBits(tauo[i], tau[i]));
  }
}

TEST(FeedbackDelay, FirstCallReportsCurrentValueNotZero) {
  // 首帧不能吐零，也不能吐"delay 长的陈旧值"——语义与 TorqueDelay::Refill 一致：
  // 从当前值开始，不引入启动瞬态。
  FeedbackDelay hold;
  hold.Init(0.5, 0.02);

  Eigen::VectorXd q(1), dq(1), tau(1), qo(1), dqo(1), tauo(1);
  q[0] = 3.5;
  dq[0] = -1.25;
  tau[0] = 7.0;
  hold.Apply(0.0, q, dq, tau, qo, dqo, tauo);
  EXPECT_DOUBLE_EQ(qo[0], 3.5);
  EXPECT_DOUBLE_EQ(dqo[0], -1.25);
  EXPECT_DOUBLE_EQ(tauo[0], 7.0);
}

TEST(FeedbackDelay, DoesNotAdvanceWhenTimeStandsStill) {
  // 暂停时每轮的 mj_forward 会用同一个 d->time 调用：不能推进寄存器，
  // 也不能把真值压进历史环（否则同一时刻反复调用会灌满环）。
  FeedbackDelay hold;
  hold.Init(0.5, 0.0);

  Eigen::VectorXd q(1), dq(1), tau(1), qo(1), dqo(1), tauo(1);
  dq[0] = 0.0;
  tau[0] = 0.0;
  q[0] = 1.0;
  hold.Apply(0.0, q, dq, tau, qo, dqo, tauo);

  for (int i = 0; i < 10; ++i) {
    q[0] = 9.0;  // 时间不变但真值变了
    hold.Apply(0.0, q, dq, tau, qo, dqo, tauo);
    EXPECT_DOUBLE_EQ(qo[0], 1.0);
  }
}

TEST(FeedbackDelay, FlushesWhenTimeGoesBackwards) {
  // GUI Reset 把 d->time 拨回 0：必须重采当前值，不能吐上一轮运行的值。
  FeedbackDelay hold;
  hold.Init(0.5, 0.0);

  Eigen::VectorXd q(1), dq(1), tau(1), qo(1), dqo(1), tauo(1);
  dq[0] = 0.0;
  tau[0] = 0.0;

  q[0] = 1.0;
  hold.Apply(0.0, q, dq, tau, qo, dqo, tauo);
  q[0] = 2.0;
  hold.Apply(0.1, q, dq, tau, qo, dqo, tauo);  // 时间前进但未到保持周期
  EXPECT_DOUBLE_EQ(qo[0], 1.0);

  q[0] = 9.0;
  hold.Apply(0.0, q, dq, tau, qo, dqo, tauo);  // 时间倒退
  EXPECT_DOUBLE_EQ(qo[0], 9.0);
}

TEST(FeedbackDelay, PureDelayShiftsContentByConfiguredTime) {
  // hold=0 + delay=D：每个发布时刻上报的应当是 (t - D) 之前最新的一份真值。
  constexpr double kDelay = 0.010;
  const std::vector<double> times = SimPublishTimes(200);

  FeedbackDelay delay;
  delay.Init(0.0, kDelay);
  EXPECT_TRUE(delay.has_delay());
  EXPECT_FALSE(delay.has_hold());

  Eigen::VectorXd q(1), dq(1), tau(1), qo(1), dqo(1), tauo(1);
  dq[0] = 0.0;
  tau[0] = 0.0;

  std::vector<double> got;
  for (std::size_t i = 0; i < times.size(); ++i) {
    q[0] = static_cast<double>(i);
    delay.Apply(times[i], q, dq, tau, qo, dqo, tauo);
    got.push_back(qo[0]);

    // 期望值：环里 (t - D) 之前最新的一份（没有更早的则退回最旧一份 = 索引 0）。
    // 这里的 +1e-9 与实现里的 kTimeEps 一致：发布时刻是 2 ms 的整数倍，累加误差在
    // 1e-17 量级，1e-9 足够吃掉它又不会误纳下一拍。
    std::size_t expect = 0;
    for (std::size_t j = 0; j <= i; ++j) {
      if (times[j] <= times[i] - kDelay + 1e-9) {
        expect = j;
      }
    }
    EXPECT_DOUBLE_EQ(got[i], static_cast<double>(expect)) << "i=" << i;
  }
}

TEST(FeedbackDelay, RepeatRateMatchesMeasuredRealRobot) {
  // 真机实测：底层快照 11.52 ms 刷新一次（86.8 Hz）而发布固定 200 Hz，结果 56.6% 的帧
  // 与前一帧逐位完全相同，且是"整帧"保持（pos/vel/tor 一起变）。这里用仿真实际的
  // 6/4 ms 发布栅格喂同一组参数，检查重复率与整帧性。
  FeedbackDelay hold;
  hold.Init(0.01152, 0.0);

  const std::vector<double> times = SimPublishTimes(4000);
  Eigen::VectorXd q(1), dq(1), tau(1), qo(1), dqo(1), tauo(1);

  int repeats = 0;
  int partial = 0;
  bool have_prev = false;
  Eigen::Vector3d prev = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < times.size(); ++i) {
    // 每个发布时刻真值都不同：这样"内容不变"只可能来自保持，不会与"机器人静止"混淆。
    q[0] = static_cast<double>(i);
    dq[0] = static_cast<double>(i) * 2.0;
    tau[0] = static_cast<double>(i) * 3.0;
    hold.Apply(times[i], q, dq, tau, qo, dqo, tauo);

    const Eigen::Vector3d cur(qo[0], dqo[0], tauo[0]);
    if (have_prev) {
      const int same = (cur.array() == prev.array()).count();
      if (same == 3) {
        ++repeats;
      } else if (same != 0) {
        ++partial;  // 三路必须同帧变化，出现部分更新即为实现错误
      }
    }
    prev = cur;
    have_prev = true;
  }

  const double rate = static_cast<double>(repeats) / static_cast<double>(times.size() - 1);
  EXPECT_NEAR(rate, 0.565, 0.02);  // 实测 56.6%（= 1 - 发布周期/保持周期）
  EXPECT_EQ(partial, 0);
}

TEST(FeedbackDelay, HoldAndDelayCompose) {
  // 两者同时开：保持决定"何时"更新，延迟决定"更新成什么"。用均匀 10 ms 发布栅格，
  // 使延迟正好落在栅格点上，断言才不含量化误差。
  FeedbackDelay both;
  both.Init(0.03, 0.02);  // 保持 30 ms，延迟 20 ms

  Eigen::VectorXd q(1), dq(1), tau(1), qo(1), dqo(1), tauo(1);
  dq[0] = 0.0;
  tau[0] = 0.0;

  // 发布时刻 0,10,20,…,100 ms；真值取 1000+i（每个发布时刻都不同）。
  for (int i = 0; i <= 10; ++i) {
    q[0] = 1000.0 + i;
    both.Apply(i * 0.01, q, dq, tau, qo, dqo, tauo);

    // 期望：保持周期每 30 ms 到点刷新一次，刷新时上报 (t - 20 ms) 时刻的真值。
    // 发布时刻 t = i*10ms，刷新发生在 i = 0,3,6,9：
    //   t=0  首帧 = 当前值            -> 1000
    //   t=30ms 上报 t=10ms 的真值      -> 1001
    //   t=60ms 上报 t=40ms 的真值      -> 1004
    //   t=90ms 上报 t=70ms 的真值      -> 1007
    double expect = 1000.0;
    for (int k : {0, 3, 6, 9}) {
      if (i >= k) {
        expect = 1000.0 + (k == 0 ? 0 : (k - 2));
      }
    }
    EXPECT_DOUBLE_EQ(qo[0], expect) << "i=" << i;
  }
}

TEST(ErrorModel, ZeroSeedDisablesFeedbackTimingBitExactly) {
  // seed=0 仍然整体关闭：即使配了反馈时序，也必须逐位不改动任何数值。
  SimErrorOptions options;
  options.seed = 0;
  options.feedback_hold_ms = 11.5;
  options.feedback_delay_ms = 5.0;

  SimErrorModel model;
  model.Configure(options);
  model.Init(2, {0, 1}, kDt);
  EXPECT_FALSE(model.enabled());
  EXPECT_FALSE(model.hasFeedbackDelay());

  Eigen::VectorXd q(2), dq(2), tau(2), qo(2), dqo(2), tauo(2);
  q << -0.0, 0.5;
  dq << 0.25, -0.0;
  tau << 1.0, -0.0;
  model.ApplyFeedbackDelay(0.0, q, dq, tau, qo, dqo, tauo);
  for (Eigen::Index i = 0; i < 2; ++i) {
    EXPECT_TRUE(SameBits(qo[i], q[i]));
    EXPECT_TRUE(SameBits(dqo[i], dq[i]));
    EXPECT_TRUE(SameBits(tauo[i], tau[i]));
  }
}

TEST(ErrorModel, FeedbackTimingActiveWithSeedAndReportedInSummary) {
  SimErrorOptions options;
  options.seed = 1;
  options.feedback_hold_ms = 11.52;
  options.feedback_delay_ms = 0.0;

  SimErrorModel model;
  model.Configure(options);
  model.Init(1, {0}, kDt);
  EXPECT_TRUE(model.hasFeedbackDelay());

  const std::string summary = model.Summary();
  EXPECT_NE(summary.find("hold=11.52ms"), std::string::npos);
  EXPECT_NE(summary.find("86.8"), std::string::npos);  // 1 / 0.01152 s

  Eigen::VectorXd q(1), dq(1), tau(1), qo(1), dqo(1), tauo(1);
  dq[0] = 0.0;
  tau[0] = 0.0;
  q[0] = 1.0;
  model.ApplyFeedbackDelay(0.0, q, dq, tau, qo, dqo, tauo);
  EXPECT_DOUBLE_EQ(qo[0], 1.0);

  q[0] = 2.0;
  model.ApplyFeedbackDelay(0.005, q, dq, tau, qo, dqo, tauo);
  EXPECT_DOUBLE_EQ(qo[0], 1.0);  // 5 ms < 11.52 ms，保持中
  q[0] = 3.0;
  model.ApplyFeedbackDelay(0.015, q, dq, tau, qo, dqo, tauo);
  EXPECT_DOUBLE_EQ(qo[0], 3.0);  // 越过保持周期，刷新
}

//----------------------------------- 指令延迟（参考路径） ----------------------------------------

namespace {

// 构造一个各字段都不同的一帧命令：这样断言能同时覆盖 amount 与增益字段有没有被搬对。
sim_common::SimJointCommand Cmd(double pos) {
  sim_common::SimJointCommand c;
  c.position = pos;
  c.velocity = pos * 2.0;
  c.effort = pos * 3.0;
  c.kp = 100.0 + pos;
  c.kd = 5.0 + pos;
  c.enable = true;
  return c;
}

}  // namespace

TEST(CommandDelay, BypassCopiesInput) {
  CommandDelay d;
  d.Init(2, 0.0);
  EXPECT_TRUE(d.bypass());

  std::vector<SimJointCommand> fresh{Cmd(1.0), Cmd(2.0)};
  std::vector<SimJointCommand> out;
  d.Apply(0.0, fresh, out);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_DOUBLE_EQ(out[0].position, 1.0);
  EXPECT_DOUBLE_EQ(out[1].kp, 102.0);
  EXPECT_TRUE(out[1].enable);
}

TEST(CommandDelay, FirstCallReportsCurrentFrameNotStale) {
  // 启动/Reload 后第一次调用必须是当前帧：吐 Δ 长的陈旧参考会让机器人先按旧目标动一下。
  CommandDelay d;
  d.Init(1, 0.030);
  std::vector<SimJointCommand> fresh{Cmd(7.0)};
  std::vector<SimJointCommand> out;
  d.Apply(0.0, fresh, out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_DOUBLE_EQ(out[0].position, 7.0);
}

TEST(CommandDelay, ShiftsByConfiguredDelay) {
  // 指令每 10 ms 到一帧、延迟 30 ms：t 时刻的输出必须等于 (t−Δ) 之前最新的那一帧。
  constexpr double kPeriod = 0.01;
  constexpr double kDelay = 0.030;
  CommandDelay d;
  d.Init(1, kDelay);

  std::vector<SimJointCommand> fresh(1);
  std::vector<SimJointCommand> out;
  std::vector<double> stamps;
  for (int i = 0; i < 20; ++i) {
    const double t = i * kPeriod;
    fresh[0] = Cmd(static_cast<double>(i));
    d.Apply(t, fresh, out);
    stamps.push_back(t);

    // 期望：环里 (t − Δ) 之前最新的一帧；还没有更早的那就退回最旧一帧（首帧）。
    double expect = 0.0;
    for (int k = 0; k <= i; ++k) {
      if (stamps[static_cast<std::size_t>(k)] <= t - kDelay + 1e-9) {
        expect = static_cast<double>(k);
      }
    }
    EXPECT_DOUBLE_EQ(out[0].position, expect) << "i=" << i;
    EXPECT_DOUBLE_EQ(out[0].kp, 100.0 + expect);
  }
}

TEST(CommandDelay, DoesNotAdvanceWhenTimeStandsStill) {
  // 暂停时每轮的 mj_forward 都会走一遍指令写入路径（同一个 d->time）：
  // 不能推进环，也不能改动上一次给出的延迟参考。
  CommandDelay d;
  d.Init(1, 0.030);
  std::vector<SimJointCommand> fresh{Cmd(1.0)};
  std::vector<SimJointCommand> out;
  d.Apply(0.0, fresh, out);
  const double first = out[0].position;

  for (int i = 0; i < 10; ++i) {
    fresh[0] = Cmd(9.0);  // 同一时刻却给了新内容
    d.Apply(0.0, fresh, out);
    EXPECT_DOUBLE_EQ(out[0].position, first);
  }
}

TEST(CommandDelay, FlushesWhenTimeGoesBackwards) {
  // GUI Reset / /reset_* 把仿真时间拨回：必须重采当前帧，不能吐上一轮运行的参考。
  CommandDelay d;
  d.Init(1, 0.5);  // 延迟 0.5 s：这段时间内不会刷新
  std::vector<SimJointCommand> fresh{Cmd(1.0)};
  std::vector<SimJointCommand> out;
  d.Apply(0.0, fresh, out);

  fresh[0] = Cmd(2.0);
  d.Apply(0.1, fresh, out);
  EXPECT_DOUBLE_EQ(out[0].position, 1.0);  // 仍在延迟窗口内

  fresh[0] = Cmd(9.0);
  d.Apply(0.0, fresh, out);  // 时间倒退
  EXPECT_DOUBLE_EQ(out[0].position, 9.0);
}

TEST(ErrorModel, ZeroSeedDisablesCommandDelayBitExactly) {
  SimErrorOptions options;
  options.seed = 0;
  options.command_delay_ms = 50.0;

  SimErrorModel model;
  model.Configure(options);
  model.Init(2, {0, 1}, kDt);
  EXPECT_FALSE(model.hasCommandDelay());

  std::vector<SimJointCommand> fresh{Cmd(-0.0), Cmd(0.5)};
  std::vector<SimJointCommand> out;
  model.ApplyCommandDelay(1.0, fresh, out);
  ASSERT_EQ(out.size(), 2u);
  for (std::size_t i = 0; i < 2; ++i) {
    EXPECT_TRUE(SameBits(out[i].position, fresh[i].position));
    EXPECT_TRUE(SameBits(out[i].velocity, fresh[i].velocity));
    EXPECT_TRUE(SameBits(out[i].kp, fresh[i].kp));
    EXPECT_EQ(out[i].enable, fresh[i].enable);
  }
}

TEST(ErrorModel, CommandDelayActiveWithSeedAndReportedInSummary) {
  SimErrorOptions options;
  options.seed = 1;
  options.command_delay_ms = 30.0;

  SimErrorModel model;
  model.Configure(options);
  model.Init(1, {0}, kDt);
  EXPECT_TRUE(model.hasCommandDelay());
  EXPECT_NE(model.Summary().find("command_delay=30ms"), std::string::npos);
  EXPECT_NE(model.Summary().find("参考路径, 回路外"), std::string::npos);

  std::vector<SimJointCommand> fresh{Cmd(1.0)};
  std::vector<SimJointCommand> out;
  model.ApplyCommandDelay(0.0, fresh, out);
  EXPECT_DOUBLE_EQ(out[0].position, 1.0);  // 首帧 = 当前值

  // 只走 10 ms（< 30 ms），历史里还没有更早的帧 → 仍停在初始帧，不吐零值也不越界。
  for (int i = 1; i <= 5; ++i) {
    fresh[0] = Cmd(static_cast<double>(i));
    model.ApplyCommandDelay(i * kDt, fresh, out);
  }
  EXPECT_DOUBLE_EQ(out[0].position, 1.0);
}
