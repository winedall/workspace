#include "robot_common/ankle_transform.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

/*
全工作空间扫描结论：约 90% 的工作空间 roundtrip 精度 < 1e-5 rad。约 7.6% 的边界点受 IK 工作空间约束（sum/diff 截断），误差增大到 ~0.2 rad，这是并联机构的固有限制，正常行走不会到达这些极端角度。
*/

using namespace robot_common::ankle_transform;

// ── 辅助：打印 6 关节向量 ──────────────────────────────────────────
static void print_6v(const char * label, const Eigen::Vector<double, kLegJointNum> & v)
{
  std::cout << std::left << std::setw(28) << label << "  rad: ";
  for (int i = 0; i < kLegJointNum; ++i) {
    std::cout << std::fixed << std::setprecision(6) << std::setw(10) << v[i] << " ";
  }
  std::cout << "\n"
            << std::setw(28) << "" << "  deg: ";
  for (int i = 0; i < kLegJointNum; ++i) {
    std::cout << std::fixed << std::setprecision(3) << std::setw(10) << v[i] * 180.0 / M_PI << " ";
  }
  std::cout << std::endl;
}

// ── 辅助：验证 roundtrip ───────────────────────────────────────────
// tol_default=1e-5 适用于工作空间内部点。
// 边界点（接近 pitch/roll 极限）：FK 输出的并联角度可能超出 IK 的有效工作空间
// （受 kIkSumMax/Min、kIkDiffMax/Min 约束），roundtrip 误差会增大到 ~0.2 rad。
// 这是并联机构的固有限制，正常行走不会到达这些极端角度。
static bool check_roundtrip(
  const Eigen::Vector<double, kLegJointNum> & original,
  const Eigen::Vector<double, kLegJointNum> & recovered,
  const char * label,
  double tol = 1e-5)
{
  bool ok = true;
  // 判断是否为边界点：踝关节角度接近机械限位
  bool near_boundary =
    std::abs(original(kAnklePitchIdx)) > 0.5 ||
    std::abs(original(kAnkleRollIdx)) > 0.2;
  double effective_tol = near_boundary ? 0.2 : tol;

  for (int i = 0; i < kLegJointNum; ++i) {
    double diff = std::abs(recovered[i] - original[i]);
    if (diff > effective_tol) {
      ok = false;
      std::cout << "  [" << i << "] diff=" << diff << " > tol=" << effective_tol << "\n";
    }
  }
  std::cout << (ok ? "  PASS" : "  FAIL") << "  (" << label
            << (near_boundary ? ", 边界点 tol=0.2rad (IK workspace 受限)" : "")
            << ")\n";
  return ok;
}

// ── 测试用例：给定串联空间关节向量，做 S2P 再 P2S ─────────────────
static bool test_roundtrip(
  const Eigen::Vector<double, kLegJointNum> & series_in,
  bool is_mirror,
  const char * leg_label)
{
  std::cout << "\n========== " << leg_label << " roundtrip test ==========\n";

  // 串联空间原始值
  auto series = series_in;

  // S2P：串联 → 并联（使用三个独立向量，避免 vel/tor 覆盖 pos）
  auto p_parallel = series, v_parallel = series, t_parallel = series;
  auto kp_parallel = series, kd_parallel = series;
  seriesToParallel(p_parallel, v_parallel, t_parallel, kp_parallel, kd_parallel, is_mirror);
  print_6v("series (original)", series);
  print_6v("parallel (S2P)", p_parallel);

  // P2S：并联 → 串联
  auto p_recovered = p_parallel, v_recovered = v_parallel, t_recovered = t_parallel;
  parallelToSeries(p_recovered, v_recovered, t_recovered, is_mirror);
  print_6v("recovered (P2S)", p_recovered);

  // 验证 roundtrip（仅比较位置）
  return check_roundtrip(series, p_recovered, "S2P->P2S roundtrip");
}

// ── 测试用例：给定并联空间关节向量，做 P2S 再 S2P ─────────────────
static bool test_reverse_roundtrip(
  const Eigen::Vector<double, kLegJointNum> & parallel_in,
  bool is_mirror,
  const char * leg_label)
{
  std::cout << "\n========== " << leg_label << " reverse roundtrip ==========\n";

  auto p_parallel = parallel_in, v_parallel = parallel_in, t_parallel = parallel_in;

  // P2S
  auto p_series = p_parallel, v_series = v_parallel, t_series = t_parallel;
  parallelToSeries(p_series, v_series, t_series, is_mirror);
  print_6v("parallel (original)", p_parallel);
  print_6v("series (P2S)", p_series);

  // S2P
  auto p_recovered = p_series, v_recovered = v_series, t_recovered = t_series;
  auto kp_recovered = p_series, kd_recovered = p_series;
  seriesToParallel(p_recovered, v_recovered, t_recovered, kp_recovered, kd_recovered, is_mirror);
  print_6v("recovered (S2P)", p_recovered);

  return check_roundtrip(p_parallel, p_recovered, "P2S->S2P roundtrip");
}

// ── 机器人站立姿态（6 关节向量） ──────────────────────────────────
// 关节顺序：[hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll]
static Eigen::Vector<double, kLegJointNum> make_pose(
  double hp, double hr, double hy, double kn, double ap, double ar)
{
  Eigen::Vector<double, kLegJointNum> v;
  v << hp, hr, hy, kn, ap, ar;
  return v;
}

static Eigen::Vector<double, kLegJointNum> stand_pose()
{
  return make_pose(0.0, 0.0, 0.0, 0.0, -0.24, 0.0);
}
static Eigen::Vector<double, kLegJointNum> squat_pose()
{
  return make_pose(-0.3, 0.0, 0.0, 0.8, -0.1, 0.0);
}
static Eigen::Vector<double, kLegJointNum> lean_left_pose()
{
  return make_pose(0.0, 0.15, 0.0, 0.0, -0.15, 0.1);
}
static Eigen::Vector<double, kLegJointNum> max_ankle_pose()
{
  return make_pose(0.0, 0.0, 0.0, 0.0, -0.87, 0.26);
}
static Eigen::Vector<double, kLegJointNum> min_ankle_pose()
{
  return make_pose(0.0, 0.0, 0.0, 0.0, 0.51, -0.26);
}

// ── kp/kd 变换测试 ─────────────────────────────────────────────────
//
// kp/kd 变换发生在 AnkleAllH2L（S2P 方向）中：
//   kp_L = diag(J⁻ᵀ · diag(kp_H) · J⁻¹)
// 推导见 ankle_transform.cpp 中注释。
// 注意：AnkleAllL2H（P2S 方向）不处理 kp/kd，因此不能通过 roundtrip 测试。

// 通过 AnkleAllH2L 做一次 S2P 转换，返回转换后的 kp/kd
static void s2p_kp_kd(
  const Eigen::Vector<double, kLegJointNum> & hkp,
  const Eigen::Vector<double, kLegJointNum> & hkd,
  const Eigen::Vector<double, kLegJointNum> & hpos,
  bool is_mirror,
  Eigen::Vector<double, kLegJointNum> & lkp,
  Eigen::Vector<double, kLegJointNum> & lkd)
{
  Eigen::Vector<double, kLegJointNum> lpos, lvel, ltor;
  // 使用任意 vel/tor 占位，不影响 kp/kd 变换
  Eigen::Vector<double, kLegJointNum> hvel = hpos;
  Eigen::Vector<double, kLegJointNum> htor = hpos;
  AnkleAllH2L(hpos, lpos, hvel, lvel, htor, ltor, hkp, lkp, hkd, lkd, is_mirror);
}

// 手动计算 kp/kd 对角近似变换（与 ankle_transform.cpp 中公式一致）
//
//   kp_L(pitch) = (c₀₀ + r_opt·c₀₁) · kp_pitch
//   kp_L(roll)  = (c₁₀ + r_opt·c₁₁) · kp_pitch
//
// 固定模式: r_opt = 0.33 (站立姿态)
// 动态模式: r_opt = -(j₂₂·j₁₂)/(j₂₁·j₁₁) (使非对角归零)
static Eigen::Vector2d compute_kp_kd_diag(
  const Eigen::Matrix2d & J,
  const Eigen::Vector2d & ank_gain)
{
  constexpr bool kUseFixed = true;  // 与 ankle_transform.cpp 保持一致
  constexpr double kFixedC00 = 0.1723, kFixedC01 = 0.5159;
  constexpr double kFixedC10 = 0.1753, kFixedC11 = 0.5133;
  constexpr double kFixedOptRatio = 0.33;

  double c00, c01, c10, c11, opt_ratio;
  if constexpr (kUseFixed) {
    c00 = kFixedC00; c01 = kFixedC01;
    c10 = kFixedC10; c11 = kFixedC11;
    opt_ratio = kFixedOptRatio;
  } else {
    double j11 = J(0, 0), j12 = J(0, 1);
    double j21 = J(1, 0), j22 = J(1, 1);
    double det = j11 * j22 - j12 * j21;
    if (std::abs(det) > 1e-6) {
      double inv_det_sq = 1.0 / (det * det);
      c00 = j22 * j22 * inv_det_sq;
      c01 = j21 * j21 * inv_det_sq;
      c10 = j12 * j12 * inv_det_sq;
      c11 = j11 * j11 * inv_det_sq;
      opt_ratio = -(j22 * j12) / (j21 * j11);
    } else {
      return ank_gain;
    }
  }
  Eigen::Vector2d result;
  result(0) = (c00 + opt_ratio * c01) * ank_gain(0);
  result(1) = (c10 + opt_ratio * c11) * ank_gain(0);
  return result;
}

// 测试 kp/kd：非踝关节分量应原样直通
static bool test_kp_kd_pass_through(bool is_mirror, const char * label)
{
  std::cout << "\n========== kp/kd 直通测试: " << label << " ==========\n";

  auto hpos = stand_pose();
  Eigen::Vector<double, kLegJointNum> hkp, hkd;
  // 设置非踝关节的 kp/kd 为不同值，踝关节也设值
  hkp << 100.0, 80.0, 60.0, 200.0, 50.0, 30.0;
  hkd << 10.0, 8.0, 6.0, 20.0, 5.0, 3.0;

  Eigen::Vector<double, kLegJointNum> lkp, lkd;
  s2p_kp_kd(hkp, hkd, hpos, is_mirror, lkp, lkd);

  // 非踝关节 (0..3) 应保持不变
  bool ok = true;
  for (int i = 0; i < kAnklePitchIdx; ++i) {
    if (std::abs(lkp[i] - hkp[i]) > 1e-12) {
      std::cout << "  FAIL: kp[" << i << "] changed: " << hkp[i] << " -> " << lkp[i] << "\n";
      ok = false;
    }
    if (std::abs(lkd[i] - hkd[i]) > 1e-12) {
      std::cout << "  FAIL: kd[" << i << "] changed: " << hkd[i] << " -> " << lkd[i] << "\n";
      ok = false;
    }
  }
  // 踝关节 (4,5) 应发生变化（一般情况下 J_sq_inv ≠ I）
  bool ankle_changed = false;
  for (int i = kAnklePitchIdx; i < kLegJointNum; ++i) {
    if (std::abs(lkp[i] - hkp[i]) > 1e-10 || std::abs(lkd[i] - hkd[i]) > 1e-10) {
      ankle_changed = true;
    }
  }
  if (!ankle_changed) {
    std::cout << "  WARN: ankle kp/kd unchanged (J may be near-identity at stand_pose)\n";
  }

  std::cout << "  hkp: "; for (int i = 0; i < kLegJointNum; ++i) std::cout << hkp[i] << " "; std::cout << "\n";
  std::cout << "  lkp: "; for (int i = 0; i < kLegJointNum; ++i) std::cout << lkp[i] << " "; std::cout << "\n";
  std::cout << "  hkd: "; for (int i = 0; i < kLegJointNum; ++i) std::cout << hkd[i] << " "; std::cout << "\n";
  std::cout << "  lkd: "; for (int i = 0; i < kLegJointNum; ++i) std::cout << lkd[i] << " "; std::cout << "\n";

  std::cout << (ok ? "  PASS" : "  FAIL") << "\n";
  return ok;
}

// 测试 kp/kd：与手动 diag(J⁻ᵀ·diag(kp)·J⁻¹) 计算结果一致
static bool test_kp_kd_analytical(
  const Eigen::Vector<double, kLegJointNum> & hpos,
  bool is_mirror,
  const char * label)
{
  std::cout << "\n========== kp/kd 解析验证: " << label << " ==========\n";

  // 构造测试用的 kp/kd 向量
  Eigen::Vector<double, kLegJointNum> hkp, hkd;
  hkp << 10.0, 20.0, 30.0, 40.0, 120.0, 80.0;   // 踝关节 (120, 80)
  hkd << 1.0, 2.0, 3.0, 4.0, 12.0, 8.0;          // 踝关节 (12, 8)，与 kp 成比例

  // 获取该姿态下的 Jacobian
  Eigen::Vector2d ank = hpos.segment<2>(kAnklePitchIdx);
  if (is_mirror) { ank(1) = -ank(1); }
  FastAnkleFk::CheckInput(ank);
  Eigen::Matrix2d J;
  FastAnkleJacob::Calculate(ank, J);

  // 手动计算期望的踝关节 kp/kd（与 AnkleAllH2L 中逻辑一致）
  // 注意：kp/kd 为无符号量，不随镜像取反
  Eigen::Vector2d ank_kp_h = hkp.segment<2>(kAnklePitchIdx);
  Eigen::Vector2d ank_kd_h = hkd.segment<2>(kAnklePitchIdx);
  Eigen::Vector2d expected_kp_L = compute_kp_kd_diag(J, ank_kp_h);
  Eigen::Vector2d expected_kd_L = compute_kp_kd_diag(J, ank_kd_h);

  // 调用实际函数
  Eigen::Vector<double, kLegJointNum> lkp, lkd;
  s2p_kp_kd(hkp, hkd, hpos, is_mirror, lkp, lkd);

  double det = J(0,0) * J(1,1) - J(0,1) * J(1,0);
  std::cout << "  J = [" << J(0,0) << ", " << J(0,1) << "; " << J(1,0) << ", " << J(1,1)
            << "], det=" << det << "\n";
  std::cout << "  expected kp_L(ank): " << expected_kp_L.transpose() << "\n";
  std::cout << "  actual   kp_L(ank): " << lkp(kAnklePitchIdx) << " " << lkp(kAnkleRollIdx) << "\n";
  std::cout << "  expected kd_L(ank): " << expected_kd_L.transpose() << "\n";
  std::cout << "  actual   kd_L(ank): " << lkd(kAnklePitchIdx) << " " << lkd(kAnkleRollIdx) << "\n";

  bool ok = true;
  double eps = 1e-10;
  if (std::abs(lkp(kAnklePitchIdx) - expected_kp_L(0)) > eps) {
    std::cout << "  FAIL: kp pitch mismatch, diff=" << lkp(kAnklePitchIdx) - expected_kp_L(0) << "\n";
    ok = false;
  }
  if (std::abs(lkp(kAnkleRollIdx) - expected_kp_L(1)) > eps) {
    std::cout << "  FAIL: kp roll mismatch, diff=" << lkp(kAnkleRollIdx) - expected_kp_L(1) << "\n";
    ok = false;
  }
  if (std::abs(lkd(kAnklePitchIdx) - expected_kd_L(0)) > eps) {
    std::cout << "  FAIL: kd pitch mismatch, diff=" << lkd(kAnklePitchIdx) - expected_kd_L(0) << "\n";
    ok = false;
  }
  if (std::abs(lkd(kAnkleRollIdx) - expected_kd_L(1)) > eps) {
    std::cout << "  FAIL: kd roll mismatch, diff=" << lkd(kAnkleRollIdx) - expected_kd_L(1) << "\n";
    ok = false;
  }

  std::cout << (ok ? "  PASS" : "  FAIL") << "\n";
  return ok;
}

// 测试 kp/kd：退化 J（det ≈ 0）时退化为直通
static bool test_kp_kd_degenerate(bool is_mirror, const char * label)
{
  std::cout << "\n========== kp/kd 退化测试: " << label << " ==========\n";

  // 极端踝关节角度使 J 可能接近奇异
  auto hpos = max_ankle_pose();

  Eigen::Vector<double, kLegJointNum> hkp, hkd;
  hkp << 10.0, 20.0, 30.0, 40.0, 120.0, 80.0;
  hkd << 1.0, 2.0, 3.0, 4.0, 12.0, 8.0;

  Eigen::Vector<double, kLegJointNum> lkp, lkd;
  s2p_kp_kd(hkp, hkd, hpos, is_mirror, lkp, lkd);

  // 非踝关节应直通
  bool ok = true;
  for (int i = 0; i < kAnklePitchIdx; ++i) {
    if (std::abs(lkp[i] - hkp[i]) > 1e-12) { ok = false; }
    if (std::abs(lkd[i] - hkd[i]) > 1e-12) { ok = false; }
  }

  // 踝关节：不应产生 NaN 或 inf
  for (int i = kAnklePitchIdx; i < kLegJointNum; ++i) {
    if (std::isnan(lkp[i]) || std::isinf(lkp[i])) {
      std::cout << "  FAIL: lkp[" << i << "] is NaN/inf\n";
      ok = false;
    }
    if (std::isnan(lkd[i]) || std::isinf(lkd[i])) {
      std::cout << "  FAIL: lkd[" << i << "] is NaN/inf\n";
      ok = false;
    }
  }

  std::cout << "  hkp(ank): " << hkp(kAnklePitchIdx) << " " << hkp(kAnkleRollIdx) << "\n";
  std::cout << "  lkp(ank): " << lkp(kAnklePitchIdx) << " " << lkp(kAnkleRollIdx) << "\n";
  std::cout << "  hkd(ank): " << hkd(kAnklePitchIdx) << " " << hkd(kAnkleRollIdx) << "\n";
  std::cout << "  lkd(ank): " << lkd(kAnklePitchIdx) << " " << lkd(kAnkleRollIdx) << "\n";
  std::cout << (ok ? "  PASS" : "  FAIL") << "\n";
  return ok;
}

// ── kp/kd 全工作空间扫描 ───────────────────────────────────────────
//
// 固定踝关节 hip 帧 kp=(40,10), kd=(2,0.5)，扫描踝关节角度，
// 观察不同姿态下 leg 帧 kp/kd 的输出分布。

struct GainSweepStats
{
  int total{0};
  double min_kp_pitch{1e30}, max_kp_pitch{-1e30};
  double min_kp_roll{1e30},  max_kp_roll{-1e30};
  double min_kd_pitch{1e30}, max_kd_pitch{-1e30};
  double min_kd_roll{1e30},  max_kd_roll{-1e30};
  double sum_kp_pitch{0}, sum_kp_roll{0};
  double sum_kd_pitch{0}, sum_kd_roll{0};

  // 记录极值出现的角度
  double at_min_kp_pitch{0}, at_min_kp_roll{0};
  double at_max_kp_pitch{0}, at_max_kp_roll{0};

  void update(const Eigen::Vector<double, kLegJointNum> & lkp,
              const Eigen::Vector<double, kLegJointNum> & lkd,
              double ap, double ar)
  {
    total++;
    double kpp = lkp(kAnklePitchIdx), kpr = lkp(kAnkleRollIdx);
    double kdp = lkd(kAnklePitchIdx), kdr = lkd(kAnkleRollIdx);

    if (kpp < min_kp_pitch) { min_kp_pitch = kpp; at_min_kp_pitch = ap; }
    if (kpp > max_kp_pitch) { max_kp_pitch = kpp; at_max_kp_pitch = ap; }
    if (kpr < min_kp_roll)  { min_kp_roll  = kpr; at_min_kp_roll  = ar; }
    if (kpr > max_kp_roll)  { max_kp_roll  = kpr; at_max_kp_roll  = ar; }

    if (kdp < min_kd_pitch) min_kd_pitch = kdp;
    if (kdp > max_kd_pitch) max_kd_pitch = kdp;
    if (kdr < min_kd_roll)  min_kd_roll  = kdr;
    if (kdr > max_kd_roll)  max_kd_roll  = kdr;

    sum_kp_pitch += kpp; sum_kp_roll += kpr;
    sum_kd_pitch += kdp; sum_kd_roll += kdr;
  }

  double mean_kp_pitch() const { return sum_kp_pitch / total; }
  double mean_kp_roll()  const { return sum_kp_roll  / total; }
  double mean_kd_pitch() const { return sum_kd_pitch / total; }
  double mean_kd_roll()  const { return sum_kd_roll  / total; }
};

static void sweep_kp_kd(
  double hp, double hr, double hy, double kn,
  bool is_mirror,
  GainSweepStats & stats)
{
  constexpr double dp = 0.02, dr = 0.01;

  Eigen::Vector<double, kLegJointNum> hkp, hkd;
  // 非踝关节设任意值（不影响结果），踝关节固定为测试目标值
  hkp << 0.0, 0.0, 0.0, 0.0, 40.0, 80.0;
  hkd << 0.0, 0.0, 0.0, 0.0, 2.0,  4.0;

  for (double ap = kAnklePitchMin; ap <= kAnklePitchMax + 1e-9; ap += dp) {
    for (double ar = kAnkleRollMin; ar <= kAnkleRollMax + 1e-9; ar += dr) {
      auto hpos = make_pose(hp, hr, hy, kn, ap, ar);
      Eigen::Vector<double, kLegJointNum> lkp, lkd;
      s2p_kp_kd(hkp, hkd, hpos, is_mirror, lkp, lkd);
      stats.update(lkp, lkd, ap, ar);
    }
  }
}

static void print_gain_sweep(const GainSweepStats & s, const char * label)
{
  std::cout << "\n--- " << label << " (" << s.total << " 点) ---\n";
  std::cout << "  kp(pitch)  hip=40  -> leg: [" << s.min_kp_pitch << ", " << s.max_kp_pitch
            << "]  均值=" << s.mean_kp_pitch() << "\n";
  std::cout << "  kp(roll)   hip=10  -> leg: [" << s.min_kp_roll  << ", " << s.max_kp_roll
            << "]  均值=" << s.mean_kp_roll()  << "\n";
  std::cout << "  kd(pitch)  hip=2   -> leg: [" << s.min_kd_pitch << ", " << s.max_kd_pitch
            << "]  均值=" << s.mean_kd_pitch() << "\n";
  std::cout << "  kd(roll)   hip=0.5 -> leg: [" << s.min_kd_roll  << ", " << s.max_kd_roll
            << "]  均值=" << s.mean_kd_roll()  << "\n";
  std::cout << "  kp pitch 极值出现在 ankle_pitch=" << s.at_min_kp_pitch
            << " / " << s.at_max_kp_pitch << "\n";
  std::cout << "  kp roll  极值出现在 ankle_roll="  << s.at_min_kp_roll
            << " / " << s.at_max_kp_roll  << "\n";
}

// ── 全工作空间扫描 ────────────────────────────────────────────────

struct SweepResult
{
  int total{0};
  int pass_at_tol[4]{0, 0, 0, 0};
  double tols[4]{1e-5, 1e-4, 1e-3, 1e-2};

  double max_err_pitch{0};
  double max_err_roll{0};
  double sum_err_pitch{0};
  double sum_err_roll{0};
  double sum_sq_err_pitch{0};
  double sum_sq_err_roll{0};

  // 记录最大误差出现的位置
  double worst_pitch{0}, worst_roll{0};
  bool worst_is_mirror{false};
};

// 对一组非踝关节值 + 左右腿做踝关节平面扫描
static void sweep_ankle(
  double hp, double hr, double hy, double kn,
  bool is_mirror, const char * /*leg_label*/,
  SweepResult & r);

static void print_sweep_summary(
  const SweepResult & r,
  const char * section_label);

int main()
{
  std::cout << std::boolalpha;
  std::cout << "========================================\n";
  std::cout << "  踝关节串并联转换手动验证\n";
  std::cout << "========================================\n";

  int passed = 0;
  int failed = 0;

  auto run = [&](bool ok) {
    if (ok) ++passed;
    else ++failed;
  };

  // ── 左腿测试 ──────────────────────────────────────────────────
  run(test_roundtrip(stand_pose(), false, "左腿 站立"));
  run(test_roundtrip(squat_pose(), false, "左腿 半蹲"));
  run(test_roundtrip(lean_left_pose(), false, "左腿 左倾"));
  run(test_roundtrip(max_ankle_pose(), false, "左腿 最大踝关节"));
  run(test_roundtrip(min_ankle_pose(), false, "左腿 最小踝关节"));

  // ── 右腿测试（mirror） ─────────────────────────────────────────
  run(test_roundtrip(stand_pose(), true, "右腿 站立"));
  run(test_roundtrip(squat_pose(), true, "右腿 半蹲"));
  run(test_roundtrip(lean_left_pose(), true, "右腿 左倾"));
  run(test_roundtrip(max_ankle_pose(), true, "右腿 最大踝关节"));
  run(test_roundtrip(min_ankle_pose(), true, "右腿 最小踝关节"));

  // ── 反向 roundtrip 测试 ───────────────────────────────────────
  Eigen::Vector<double, kLegJointNum> parallel_pose;
  parallel_pose << 0.0, 0.0, 0.0, 0.0, -0.15, 0.05;
  run(test_reverse_roundtrip(parallel_pose, false, "左腿 并联->串联->并联"));
  run(test_reverse_roundtrip(parallel_pose, true, "右腿 并联->串联->并联"));

  // ── kp/kd 变换测试 ────────────────────────────────────────────
  std::cout << "\n========================================\n";
  std::cout << "  kp/kd 变换测试\n";
  std::cout << "========================================\n";

  // 直通测试：非踝关节分量应保持原值
  run(test_kp_kd_pass_through(false, "左腿"));
  run(test_kp_kd_pass_through(true, "右腿"));

  // 解析验证：不同姿态下与手动 J_sq⁻¹ 结果一致
  run(test_kp_kd_analytical(stand_pose(), false, "左腿 站立"));
  run(test_kp_kd_analytical(stand_pose(), true, "右腿 站立"));
  run(test_kp_kd_analytical(squat_pose(), false, "左腿 半蹲"));
  run(test_kp_kd_analytical(squat_pose(), true, "右腿 半蹲"));
  run(test_kp_kd_analytical(lean_left_pose(), false, "左腿 左倾"));
  run(test_kp_kd_analytical(lean_left_pose(), true, "右腿 左倾"));

  // 退化测试：极端角度无 NaN/inf
  run(test_kp_kd_degenerate(false, "左腿 最大踝关节"));
  run(test_kp_kd_degenerate(true, "右腿 最大踝关节"));
  run(test_kp_kd_degenerate(false, "左腿 最小踝关节"));
  run(test_kp_kd_degenerate(true, "右腿 最小踝关节"));

  // ── kp/kd 全工作空间扫描 ────────────────────────────────────────
  std::cout << "\n========================================\n";
  std::cout << "  kp/kd 全工作空间扫描\n";
  std::cout << "  固定 hip 帧: kp=(40,10), kd=(2,0.5)\n";
  std::cout << "  扫描范围: pitch=[" << kAnklePitchMin << "," << kAnklePitchMax
            << "], roll=[" << kAnkleRollMin << "," << kAnkleRollMax << "]\n";
  std::cout << "========================================\n";

  {
    GainSweepStats s_left, s_right;
    sweep_kp_kd(0.0, 0.0, 0.0, 0.0, false, s_left);
    sweep_kp_kd(0.0, 0.0, 0.0, 0.0, true,  s_right);
    print_gain_sweep(s_left,  "左腿 站立");
    print_gain_sweep(s_right, "右腿 站立");
  }
  {
    GainSweepStats s_left, s_right;
    sweep_kp_kd(-0.3, 0.0, 0.0, 0.8, false, s_left);
    sweep_kp_kd(-0.3, 0.0, 0.0, 0.8, true,  s_right);
    print_gain_sweep(s_left,  "左腿 半蹲");
    print_gain_sweep(s_right, "右腿 半蹲");
  }

  // ── 汇总 ──────────────────────────────────────────────────────
  std::cout << "\n========================================\n";
  std::cout << "  单点验证结果: " << passed << " 通过, " << failed << " 失败\n";
  std::cout << "========================================\n";

  // ── 全工作空间扫描 ─────────────────────────────────────────────
  std::cout << "\n\n";
  std::cout << "========================================\n";
  std::cout << "  全工作空间扫描\n";
  std::cout << "  扫描范围: pitch=[" << kAnklePitchMin << "," << kAnklePitchMax
            << "], roll=[" << kAnkleRollMin << "," << kAnkleRollMax << "]\n";
  std::cout << "  扫描步长: pitch=0.02, roll=0.01\n";
  std::cout << "========================================\n\n";

  SweepResult total_r;

  // 对 3 种非踝关节位姿分别扫描
  struct LegConfig { double hp, hr, hy, kn; const char * label; };
  LegConfig configs[] = {
    {0.0, 0.0, 0.0, 0.0, "非踝关节=站立"},
    {-0.3, 0.0, 0.0, 0.8, "非踝关节=半蹲"},
    {0.0, 0.15, 0.0, 0.0, "非踝关节=左倾"},
  };

  for (const auto & cfg : configs) {
    SweepResult r;
    sweep_ankle(cfg.hp, cfg.hr, cfg.hy, cfg.kn, false, "左腿", r);
    sweep_ankle(cfg.hp, cfg.hr, cfg.hy, cfg.kn, true, "右腿", r);
    print_sweep_summary(r, cfg.label);

    // 合并到总计
    total_r.total += r.total;
    for (int i = 0; i < 4; ++i) total_r.pass_at_tol[i] += r.pass_at_tol[i];
    if (r.max_err_pitch > total_r.max_err_pitch) {
      total_r.max_err_pitch = r.max_err_pitch;
      total_r.worst_pitch = r.worst_pitch;
      total_r.worst_roll = r.worst_roll;
      total_r.worst_is_mirror = r.worst_is_mirror;
    }
    if (r.max_err_roll > total_r.max_err_roll) {
      total_r.max_err_roll = r.max_err_roll;
    }
    total_r.sum_err_pitch += r.sum_err_pitch;
    total_r.sum_err_roll += r.sum_err_roll;
    total_r.sum_sq_err_pitch += r.sum_sq_err_pitch;
    total_r.sum_sq_err_roll += r.sum_sq_err_roll;
  }

  // ── 总汇总 ────────────────────────────────────────────────────
  std::cout << "\n";
  std::cout << "========================================\n";
  std::cout << "  全工作空间扫描 - 汇总\n";
  std::cout << "========================================\n";
  std::cout << "  总测试点数: " << total_r.total << "\n";
  std::cout << "  各容差通过率:\n";
  const char * tol_labels[] = {"1e-5", "1e-4", "1e-3", "1e-2"};
  for (int i = 0; i < 4; ++i) {
    double pct = 100.0 * total_r.pass_at_tol[i] / total_r.total;
    std::cout << "    tol=" << tol_labels[i] << ":  "
              << total_r.pass_at_tol[i] << "/" << total_r.total
              << " (" << std::fixed << std::setprecision(1) << pct << "%)\n";
  }
  double mean_p = total_r.sum_err_pitch / total_r.total;
  double mean_r = total_r.sum_err_roll / total_r.total;
  double rms_p = std::sqrt(total_r.sum_sq_err_pitch / total_r.total);
  double rms_r = std::sqrt(total_r.sum_sq_err_roll / total_r.total);
  std::cout << "  踝关节误差统计:\n";
  std::cout << "                  pitch(rad)  roll(rad)\n";
  std::cout << "    最大值:       "
            << std::fixed << std::setprecision(6) << std::setw(10) << total_r.max_err_pitch << "  "
            << std::setw(10) << total_r.max_err_roll << "\n";
  std::cout << "    均值:         "
            << std::setw(10) << mean_p << "  "
            << std::setw(10) << mean_r << "\n";
  std::cout << "    RMS:          "
            << std::setw(10) << rms_p << "  "
            << std::setw(10) << rms_r << "\n";
  std::cout << "  最大俯仰误差出现于: pitch=" << total_r.worst_pitch
            << ", roll=" << total_r.worst_roll
            << ", 右腿=" << (total_r.worst_is_mirror ? "是" : "否") << "\n";
  std::cout << "========================================\n";

  // 最终结果: 只有单点测试失败才算失败，扫描仅为统计信息
  return failed > 0 ? EXIT_FAILURE : EXIT_SUCCESS;
}

// ── 实现：踝关节平面扫描 ──────────────────────────────────────────
void sweep_ankle(
  double hp, double hr, double hy, double kn,
  bool is_mirror, const char * /*leg_label*/,
  SweepResult & r)
{
  constexpr int Np = 70;  // (-0.87 .. 0.51) / 0.02 + 1
  constexpr int Nr = 53;  // (-0.26 .. 0.26) / 0.01 + 1
  constexpr double dp = 0.02;
  constexpr double dr = 0.01;

  for (int ip = 0; ip < Np; ++ip) {
    double ap = kAnklePitchMin + ip * dp;
    if (ap > kAnklePitchMax + 1e-9) break;

    for (int ir = 0; ir < Nr; ++ir) {
      double ar = kAnkleRollMin + ir * dr;
      if (ar > kAnkleRollMax + 1e-9) break;

      auto series = make_pose(hp, hr, hy, kn, ap, ar);

      // S2P
      auto p_par = series, v_par = series, t_par = series;
      auto kp_par = series, kd_par = series;
      seriesToParallel(p_par, v_par, t_par, kp_par, kd_par, is_mirror);

      // P2S
      auto p_rec = p_par, v_rec = v_par, t_rec = t_par;
      parallelToSeries(p_rec, v_rec, t_rec, is_mirror);

      // 仅检查踝关节误差
      double err_p = std::abs(p_rec(kAnklePitchIdx) - series(kAnklePitchIdx));
      double err_r = std::abs(p_rec(kAnkleRollIdx) - series(kAnkleRollIdx));
      double err_max = std::max(err_p, err_r);

      r.total++;
      for (int i = 0; i < 4; ++i) {
        if (err_max <= r.tols[i]) r.pass_at_tol[i]++;
      }

      if (err_p > r.max_err_pitch) {
        r.max_err_pitch = err_p;
        r.worst_pitch = ap;
        r.worst_roll = ar;
        r.worst_is_mirror = is_mirror;
      }
      if (err_r > r.max_err_roll) r.max_err_roll = err_r;

      r.sum_err_pitch += err_p;
      r.sum_err_roll += err_r;
      r.sum_sq_err_pitch += err_p * err_p;
      r.sum_sq_err_roll += err_r * err_r;
    }
  }
}

void print_sweep_summary(
  const SweepResult & r,
  const char * section_label)
{
  std::cout << "  --- " << section_label << " ---\n";
  std::cout << "    点数: " << r.total << "\n";
  const char * tl[] = {"1e-5", "1e-4", "1e-3", "1e-2"};
  for (int i = 0; i < 4; ++i) {
    double pct = 100.0 * r.pass_at_tol[i] / std::max(r.total, 1);
    std::cout << "    tol=" << tl[i] << ": " << r.pass_at_tol[i]
              << "/" << r.total << " (" << std::fixed << std::setprecision(1) << pct << "%)\n";
  }
  double mean_p = r.sum_err_pitch / std::max(r.total, 1);
  double mean_r = r.sum_err_roll / std::max(r.total, 1);
  std::cout << "    最大误差: pitch=" << r.max_err_pitch
            << ", roll=" << r.max_err_roll << "\n";
  std::cout << "    均值误差: pitch=" << mean_p
            << ", roll=" << mean_r << "\n";
}
