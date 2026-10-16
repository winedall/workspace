#include "robot_common/ankle_transform.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace robot_common
{
namespace ankle_transform
{

// ── 辅助函数（替代 MATLAB 代码中的 rt_*） ────────────────────────────

// 安全 atan2 函数，处理 NaN 和无穷大
static double safe_atan2(double u0, double u1)
{
  if (std::isnan(u0) || std::isnan(u1)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (std::isinf(u0) && std::isinf(u1)) {
    int b_u0 = (u0 > 0.0) ? 1 : -1;
    int b_u1 = (u1 > 0.0) ? 1 : -1;
    return std::atan2(static_cast<double>(b_u0), static_cast<double>(b_u1));
  }
  if (u1 == 0.0) {
    if (u0 > 0.0) return M_PI / 2.0;
    if (u0 < 0.0) return -(M_PI / 2.0);
    return 0.0;
  }
  return std::atan2(u0, u1);
}

// 安全 pow 函数，处理 NaN 和无穷大
static double safe_pow(double u0, double u1)
{
  if (std::isnan(u0) || std::isnan(u1)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double d = std::abs(u0);
  double d1 = std::abs(u1);
  if (std::isinf(u1)) {
    if (d == 1.0) return 1.0;
    if (d > 1.0) return (u1 > 0.0) ? std::numeric_limits<double>::infinity() : 0.0;
    return (u1 > 0.0) ? 0.0 : std::numeric_limits<double>::infinity();
  }
  if (d1 == 0.0) return 1.0;
  if (d1 == 1.0) return (u1 > 0.0) ? u0 : 1.0 / u0;
  if (u1 == 2.0) return u0 * u0;
  if (u1 == 0.5 && u0 >= 0.0) return std::sqrt(u0);
  if (u0 < 0.0 && u1 > std::floor(u1)) return std::numeric_limits<double>::quiet_NaN();
  return std::pow(u0, u1);
}

// ── FastAnkleFk ────────────────────────────────────────────────────
// 检查输入角度是否超出范围
void FastAnkleFk::CheckInput(Eigen::Vector2d & f)
{
  if (f[0] > kAnklePitchMax) f[0] = kAnklePitchMax;
  else if (f[0] < kAnklePitchMin) f[0] = kAnklePitchMin;
  if (f[1] > kAnkleRollMax) f[1] = kAnkleRollMax;
  else if (f[1] < kAnkleRollMin) f[1] = kAnkleRollMin;
}

// 计算逆运动学
// 输入：f - 角度向量（pitch, roll）
// 输出：t - 位置向量（x, y, z）
void FastAnkleFk::Calculate(const Eigen::Vector2d & f, Eigen::Vector2d & t)
{
  double a_tmp;
  double a_tmp_tmp;
  double ai;
  double ar;
  double ar_tmp;
  double ar_tmp_tmp;
  double ar_tmp_tmp_tmp;
  double b_a_tmp;
  double b_a_tmp_tmp;
  double b_ar_tmp;
  double b_ar_tmp_tmp;
  double b_ar_tmp_tmp_tmp;
  double b_im;
  double b_re;
  double br_tmp;
  double c1c2;
  double c1s2;
  double c2s1;
  double c_a_tmp;
  double c_ar_tmp_tmp;
  double c_ar_tmp_tmp_tmp;
  double c_im;
  double c_re;
  double cf1;
  double cf2;
  double d_ar_tmp_tmp;
  double e_ar_tmp_tmp;
  double f_ar_tmp_tmp;
  double g_ar_tmp_tmp;
  double h_ar_tmp_tmp;
  double i_ar_tmp_tmp;
  double im;
  double re;
  double s1s2;
  double sf1;
  double sf2;
  sf1 = std::sin(f[0]);
  sf2 = std::sin(f[1]);
  cf1 = std::cos(f[0]);
  cf2 = std::cos(f[1]);
  s1s2 = sf1 * sf2;
  c1c2 = cf1 * cf2;
  c1s2 = cf1 * sf2;
  c2s1 = cf2 * sf1;
  a_tmp_tmp = 175.0 * cf1 + 230.0 * sf1;
  a_tmp = ((a_tmp_tmp - 147.0 * c1s2) - 120.0 * c1c2) + 3000.0;
  b_a_tmp = 49.0 * cf2 - 40.0 * sf2;
  b_a_tmp_tmp = (230.0 * cf1 - 175.0 * sf1) + 120.0 * c2s1;
  c_a_tmp = b_a_tmp_tmp + 147.0 * s1s2;
  ar_tmp_tmp = a_tmp * a_tmp;
  b_ar_tmp_tmp = b_a_tmp * b_a_tmp;
  c_ar_tmp_tmp = c_a_tmp * c_a_tmp;
  d_ar_tmp_tmp = 9.0 * b_ar_tmp_tmp;
  ar_tmp = ((((((483.0 * cf1 / 250000.0 - 147.0 * sf1 / 100000.0) + 63.0 * c2s1 / 62500.0) + 3087.0 * s1s2 / 2.5E+6) -
              ar_tmp_tmp / 1.0E+8) - d_ar_tmp_tmp / 1.0E+8) - c_ar_tmp_tmp / 1.0E+8) + 0.0505798527;
  ar = ar_tmp * -33.133333333333333;
  ai = ar_tmp * -3.7166666666666668;
  if (ai == 0.0) {
    re = ar / b_a_tmp;
    im = 0.0;
  } else if (ar == 0.0) {
    re = 0.0;
    im = ai / b_a_tmp;
  } else {
    re = ar / b_a_tmp;
    im = ai / b_a_tmp;
  }
  ar_tmp_tmp_tmp = 7.0 * cf1 / 400.0 + 23.0 * sf1 / 1000.0;
  b_ar_tmp_tmp_tmp = 147.0 * c1s2 / 10000.0;
  c_ar_tmp_tmp_tmp = 3.0 * c1c2 / 250.0;
  e_ar_tmp_tmp = (ar_tmp_tmp_tmp - b_ar_tmp_tmp_tmp) - c_ar_tmp_tmp_tmp;
  ar_tmp = safe_pow(a_tmp, 3.0);
  b_ar_tmp = 1.0376293541461623E+19 * b_ar_tmp_tmp;
  f_ar_tmp_tmp = (23.0 * cf1 / 500.0 - 7.0 * sf1 / 200.0) + 3.0 * c2s1 / 125.0;
  g_ar_tmp_tmp = 147.0 * s1s2 / 5000.0;
  h_ar_tmp_tmp = 147.0 * cf2 / 10000.0;
  i_ar_tmp_tmp = 3.0 * sf2 / 250.0;
  ar_tmp = (((((((((((((3.4288984728073892E+19 * c1s2 / 4.6116860184273879E+22 -
                        9.3911265264204033E+22 * sf1 / 7.2057594037927936E+25) -
                       5.0528299752151334E+23 * cf1 / 7.2057594037927936E+26) +
                      7497.0 * c2s1 / 7.8125E+7) +
                     367353.0 * s1s2 / 3.125E+9) -
                    119.0 * ar_tmp_tmp / 1.25E+11) +
                   ar_tmp / 1.0E+12) +
                  1071.0 * b_ar_tmp_tmp / 1.25E+11) -
                 119.0 * c_ar_tmp_tmp / 1.25E+11) +
                d_ar_tmp_tmp * (e_ar_tmp_tmp + 0.3) / 1.0E+8) +
               c_ar_tmp_tmp * (e_ar_tmp_tmp + 0.3) / 1.0E+8) -
              21.0 * (f_ar_tmp_tmp + g_ar_tmp_tmp) * (e_ar_tmp_tmp + 0.3) / 500.0) +
             6.9977519853212032E+17 * c1c2 / 1.152921504606847E+21) -
            1.4142135623730951 * (h_ar_tmp_tmp - i_ar_tmp_tmp) *
              std::sqrt(
                (((((((((((((((((((((1.0730744015857012E+30 * c1s2 - 1.128935157022907E+30 * sf1) -
                                       2.0003592510593277E+30 * cf1) -
                                      3.7715985671064229E+29 * c2s1) -
                                     4.6202082447053686E+29 * s1s2) +
                                    1.7441474113309633E+24 * ar_tmp_tmp) +
                                   2.1951625447714366E+21 * ar_tmp) -
                                  5.7646075230342349E+17 * safe_pow(a_tmp, 4.0)) +
                                 3.4505479385580342E+25 * b_ar_tmp_tmp) -
                                4.66933209365773E+19 * safe_pow(b_a_tmp, 4.0)) +
                               3.3349145383199653E+24 * c_ar_tmp_tmp) +
                              9.6845406386975146E+20 * safe_pow(c_a_tmp, 3.0)) -
                             5.7646075230342349E+17 * safe_pow(c_a_tmp, 4.0)) +
                            1.975646290294293E+22 * b_ar_tmp_tmp * a_tmp) +
                           2.1951625447714366E+21 * c_ar_tmp_tmp * a_tmp) +
                          9.6845406386975146E+20 * c_a_tmp * ar_tmp_tmp) +
                         8.7160865748277631E+21 * b_ar_tmp_tmp * c_a_tmp) -
                        b_ar_tmp * ar_tmp_tmp) -
                       1.152921504606847E+18 * c_ar_tmp_tmp * ar_tmp_tmp) -
                      b_ar_tmp * c_ar_tmp_tmp) -
                     1.8439365376080068E+24 * c_a_tmp * a_tmp) +
                    8.7597910333526627E+29 * c1c2) -
                   2.7887404321092932E+31) /
                 1.073741824E+17) -
              0.01035875383296;
  ar = ar_tmp * 6.2768919806476288E+19;
  ai = ar_tmp * -5.5957225370078413E+20;
  br_tmp = (((((1.5756293656318417E+20 * c1s2 - 2.4652704360226095E+20 * sf1) - 1.8757492447998116E+20 * cf1) +
              5.62949953421312E+14 * ar_tmp_tmp) + 5.066549580791808E+15 * b_ar_tmp_tmp) +
             1.2862280535770137E+20 * c1c2) - 2.7053663393569854E+21;
  if (ai == 0.0) {
    b_re = ar / br_tmp;
    b_im = 0.0;
  } else if (ar == 0.0) {
    b_re = 0.0;
    b_im = ai / br_tmp;
  } else {
    b_re = ar / br_tmp;
    b_im = ai / br_tmp;
  }
  ar_tmp *= e_ar_tmp_tmp + 0.2048;
  ar = ar_tmp * -1.8652408456692805E+24;
  ai = ar_tmp * -2.092297326882543E+23;
  br_tmp *= b_a_tmp;
  if (ai == 0.0) {
    c_re = ar / br_tmp;
    c_im = 0.0;
  } else if (ar == 0.0) {
    c_re = 0.0;
    c_im = ai / br_tmp;
  } else {
    c_re = ar / br_tmp;
    c_im = ai / br_tmp;
  }
  a_tmp = ((a_tmp_tmp + 147.0 * c1s2) - 120.0 * c1c2) + 3000.0;
  b_a_tmp = 49.0 * cf2 + 40.0 * sf2;
  c_a_tmp = b_a_tmp_tmp - 147.0 * s1s2;
  ar_tmp = b_a_tmp * b_a_tmp;
  b_ar_tmp = c_a_tmp * c_a_tmp;
  ar_tmp_tmp = (ar_tmp_tmp_tmp + b_ar_tmp_tmp_tmp) - c_ar_tmp_tmp_tmp;
  cf2 = a_tmp * a_tmp;
  c_ar_tmp_tmp = safe_pow(a_tmp, 3.0);
  br_tmp = 1.0376293541461623E+19 * ar_tmp;
  b_ar_tmp_tmp = 9.0 * ar_tmp;
  c_ar_tmp_tmp = (((((((((((((7.2570817045703674E+23 * cf1 / 1.4411518807585587E+27 +
                               1.7755655253732098E+23 * sf1 / 1.4411518807585587E+26) +
                              6.0303449724801335E+19 * c1s2 / 9.2233720368547758E+22) -
                             22401.0 * c2s1 / 1.5625E+8) +
                            1.097649E+6 * s1s2 / 6.25E+9) +
                           393.0 * cf2 / 2.5E+11) -
                          c_ar_tmp_tmp / 1.0E+12) -
                         3537.0 * ar_tmp / 2.5E+11) +
                        393.0 * b_ar_tmp / 2.5E+11) -
                       b_ar_tmp_tmp * (ar_tmp_tmp + 0.3) / 1.0E+8) -
                      b_ar_tmp * (ar_tmp_tmp + 0.3) / 1.0E+8) +
                     19.0 * (f_ar_tmp_tmp - g_ar_tmp_tmp) * (ar_tmp_tmp + 0.3) / 500.0) -
                    1.2306826474449252E+18 * c1c2 / 2.305843009213694E+21) +
                   1.4142135623730951 * (h_ar_tmp_tmp + i_ar_tmp_tmp) *
                     std::sqrt(
                       (((((((((((((((((((((3.6274877138309625E+29 * cf1 + 2.6988001115069934E+29 * sf1) +
                                              2.2096803736408614E+29 * c1s2) +
                                             5.2013000371251354E+28 * c2s1) -
                                            6.3715925454782912E+28 * s1s2) -
                                           6.1762742871551752E+24 * cf2) +
                                          3.6247852104839269E+21 * c_ar_tmp_tmp) -
                                         5.7646075230342349E+17 * safe_pow(a_tmp, 4.0)) -
                                        4.3030074264699756E+24 * ar_tmp) -
                                       4.66933209365773E+19 * safe_pow(b_a_tmp, 4.0)) -
                                      9.0328171705733629E+23 * b_ar_tmp) +
                                     8.762203435012037E+20 * safe_pow(c_a_tmp, 3.0)) -
                                    5.7646075230342349E+17 * safe_pow(c_a_tmp, 4.0)) +
                                   3.2623066894355342E+22 * ar_tmp * a_tmp) +
                                  3.6247852104839269E+21 * b_ar_tmp * a_tmp) +
                                 8.762203435012037E+20 * c_a_tmp * cf2) +
                                7.8859830915108333E+21 * ar_tmp * c_a_tmp) -
                               br_tmp * cf2) -
                              1.152921504606847E+18 * b_ar_tmp * cf2) -
                             br_tmp * b_ar_tmp) -
                            2.7548367599677844E+24 * c_a_tmp * a_tmp) -
                           1.8038207131762135E+29 * c1c2) +
                          4.59634972951194E+30) /
                        1.073741824E+17) +
                     0.0063513098879999993;
  ar = c_ar_tmp_tmp * 1.2159718993900339E+20;
  ai = c_ar_tmp_tmp * 1.1191445074015683E+21;
  br_tmp = (((((6.1947012874481172E+20 * cf1 + 8.1416074063603827E+20 * sf1) + 5.2035490814564185E+20 * c1s2) -
              1.125899906842624E+15 * cf2) - 1.0133099161583616E+16 * ar_tmp) - 4.2477951685358518E+20 * c1c2) +
            7.8371820859486466E+21;
  if (ai == 0.0) {
    e_ar_tmp_tmp = ar / br_tmp;
    sf2 = 0.0;
  } else if (ar == 0.0) {
    e_ar_tmp_tmp = 0.0;
    sf2 = ai / br_tmp;
  } else {
    e_ar_tmp_tmp = ar / br_tmp;
    sf2 = ai / br_tmp;
  }
  ar_tmp = ((((((133.0 * sf1 / 100000.0 - 437.0 * cf1 / 250000.0) - 57.0 * c2s1 / 62500.0) + 2793.0 * s1s2 / 2.5E+6) +
               cf2 / 1.0E+8) + b_ar_tmp_tmp / 1.0E+8) + b_ar_tmp / 1.0E+8) - 0.044476959999999996;
  ar = ar_tmp * 33.133333333333333;
  ai = ar_tmp * -3.6;
  if (ai == 0.0) {
    d_ar_tmp_tmp = ar / b_a_tmp;
    cf2 = 0.0;
  } else if (ar == 0.0) {
    d_ar_tmp_tmp = 0.0;
    cf2 = ai / b_a_tmp;
  } else {
    d_ar_tmp_tmp = ar / b_a_tmp;
    cf2 = ai / b_a_tmp;
  }
  ar_tmp = (ar_tmp_tmp + 0.1428) * c_ar_tmp_tmp;
  ar = ar_tmp * -3.730481691338561E+24;
  ai = ar_tmp * 4.0532396646334464E+23;
  br_tmp *= b_a_tmp;
  if (ai == 0.0) {
    c_ar_tmp_tmp = ar / br_tmp;
    br_tmp = 0.0;
  } else if (ar == 0.0) {
    c_ar_tmp_tmp = 0.0;
    br_tmp = ai / br_tmp;
  } else {
    c_ar_tmp_tmp = ar / br_tmp;
    br_tmp = ai / br_tmp;
  }
  t[0] = safe_atan2(((im + b_im) + c_im) - -0.001892576, ((re + b_re) + c_re) - 0.000212296);
  t[1] = safe_atan2(((sf2 + cf2) + br_tmp) - 0.003125136, ((e_ar_tmp_tmp + d_ar_tmp_tmp) + c_ar_tmp_tmp) - 0.000339552);
}

// ── FastAnkleIk ────────────────────────────────────────────────────
// 检查输入角度是否超出范围
void FastAnkleIk::CheckInput(Eigen::Vector2d & t)
{
  double sum = t[0] + t[1];
  double diff = t[0] - t[1];
  if (sum > kIkSumMax) { double coef = kIkSumMax / sum; t *= coef; }
  if (sum < kIkSumMin) { double coef = kIkSumMin / sum; t *= coef; }
  if (diff > kIkDiffMax) { double coef = kIkDiffMax / diff; t *= coef; }
  if (diff < kIkDiffMin) { double coef = kIkDiffMin / diff; t *= coef; }
}

// 计算正运动学
// 输入：t - 位置向量（x, y, z）
// 输出：f - 角度向量（pitch, roll）
// 输入：tol - 目标误差
// 输入：max_iter - 最大迭代次数
// 输入：damp - 阻尼系数
// 输出：f - 计算得到的角度向量
void FastAnkleIk::Calculate(
  const Eigen::Vector2d & t, Eigen::Vector2d & f,
  double tol, int max_iter, double damp)
{
  Eigen::Vector2d x = Eigen::Vector2d(0.0, (t[0] + t[1]) / 2);
  Eigen::Vector2d tx;
  Eigen::Matrix2d J;
  for (int iter = 0; iter < max_iter; ++iter) {
    FastAnkleFk::CheckInput(x);
    FastAnkleFk::Calculate(x, tx);
    if ((tx - t).norm() < tol) {
      f = x;
      return;
    }
    FastAnkleJacob::Calculate(x, J);
    Eigen::Vector2d dx = -J.inverse() * (tx - t);
    x += damp * dx;
  }
  f = x;
}

// ── FastAnkleJacob ─────────────────────────────────────────────────
// 计算雅可比矩阵
// 输入：f - 角度向量（pitch, roll）
// 输出：J - 雅可比矩阵（2x2）
void FastAnkleJacob::Calculate(const Eigen::Vector2d & f, Eigen::Matrix2d & J)
{
  Eigen::Vector4d J_temp = Eigen::Vector4d::Zero();
  double J_tmp;
  double J_tmp_tmp;
  double a_tmp;
  double a_tmp_tmp;
  double a_tmp_tmp_tmp;
  double ab_a_tmp_tmp;
  double ac_a_tmp_tmp;
  double ar_tmp;
  double ar_tmp_tmp;
  double b_J_tmp;
  double b_J_tmp_tmp;
  double b_a_tmp;
  double b_a_tmp_tmp;
  double b_a_tmp_tmp_tmp;
  double b_ar_tmp;
  double bb_a_tmp_tmp;
  double bc_a_tmp_tmp;
  double c1c2;
  double c1s2;
  double c2s1;
  double c_J_tmp;
  double c_a_tmp;
  double c_a_tmp_tmp;
  double c_a_tmp_tmp_tmp;
  double cb_a_tmp_tmp;
  double cf1;
  double cf2;
  double d_J_tmp;
  double d_a_tmp;
  double d_a_tmp_tmp;
  double d_a_tmp_tmp_tmp;
  double db_a_tmp_tmp;
  double e_J_tmp;
  double e_a_tmp;
  double e_a_tmp_tmp;
  double e_a_tmp_tmp_tmp;
  double eb_a_tmp_tmp;
  double f_J_tmp;
  double f_a_tmp;
  double f_a_tmp_tmp;
  double f_a_tmp_tmp_tmp;
  double fb_a_tmp_tmp;
  double g_J_tmp;
  double g_a_tmp;
  double g_a_tmp_tmp;
  double g_a_tmp_tmp_tmp;
  double gb_a_tmp_tmp;
  double h_J_tmp;
  double h_a_tmp;
  double h_a_tmp_tmp;
  double h_a_tmp_tmp_tmp;
  double hb_a_tmp_tmp;
  double i_J_tmp;
  double i_a_tmp;
  double i_a_tmp_tmp;
  double i_a_tmp_tmp_tmp;
  double ib_a_tmp_tmp;
  double im;
  double j_J_tmp;
  double j_a_tmp;
  double j_a_tmp_tmp;
  double j_a_tmp_tmp_tmp;
  double jb_a_tmp_tmp;
  double k_J_tmp;
  double k_a_tmp;
  double k_a_tmp_tmp;
  double k_a_tmp_tmp_tmp;
  double kb_a_tmp_tmp;
  double l_J_tmp;
  double l_a_tmp;
  double l_a_tmp_tmp;
  double l_a_tmp_tmp_tmp;
  double lb_a_tmp_tmp;
  double m_J_tmp;
  double m_a_tmp;
  double m_a_tmp_tmp;
  double mb_a_tmp_tmp;
  double n_a_tmp;
  double n_a_tmp_tmp;
  double nb_a_tmp_tmp;
  double o_a_tmp;
  double o_a_tmp_tmp;
  double ob_a_tmp_tmp;
  double p_a_tmp;
  double p_a_tmp_tmp;
  double pb_a_tmp_tmp;
  double q_a_tmp;
  double q_a_tmp_tmp;
  double qb_a_tmp_tmp;
  double r_a_tmp;
  double r_a_tmp_tmp;
  double rb_a_tmp_tmp;
  double re;
  double s1s2;
  double s_a_tmp;
  double s_a_tmp_tmp;
  double sb_a_tmp_tmp;
  double sf1;
  double sf2;
  double t_a_tmp;
  double t_a_tmp_tmp;
  double tb_a_tmp_tmp;
  double u_a_tmp;
  double u_a_tmp_tmp;
  double ub_a_tmp_tmp;
  double v_a_tmp_tmp;
  double vb_a_tmp_tmp;
  double w_a_tmp_tmp;
  double wb_a_tmp_tmp;
  double x_a_tmp_tmp;
  double xb_a_tmp_tmp;
  double y_a_tmp_tmp;
  double yb_a_tmp_tmp;
  sf1 = std::sin(f[0]);
  sf2 = std::sin(f[1]);
  cf1 = std::cos(f[0]);
  cf2 = std::cos(f[1]);
  s1s2 = sf1 * sf2;
  c1c2 = cf1 * cf2;
  c1s2 = cf1 * sf2;
  c2s1 = cf2 * sf1;
  a_tmp_tmp = 175.0 * cf1 + 230.0 * sf1;
  b_a_tmp_tmp = (a_tmp_tmp - 147.0 * c1s2) - 120.0 * c1c2;
  a_tmp = 49.0 * cf2 - 40.0 * sf2;
  c_a_tmp_tmp = (230.0 * cf1 - 175.0 * sf1) + 120.0 * c2s1;
  b_a_tmp = c_a_tmp_tmp + 147.0 * s1s2;
  d_a_tmp_tmp = (b_a_tmp_tmp + 3000.0) * (b_a_tmp_tmp + 3000.0);
  e_a_tmp_tmp = a_tmp * a_tmp;
  c_a_tmp = (((((1.5756293656318417E+20 * c1s2 - 2.4652704360226095E+20 * sf1) - 1.8757492447998116E+20 * cf1) +
               5.62949953421312E+14 * d_a_tmp_tmp) +
              5.066549580791808E+15 * e_a_tmp_tmp) +
             1.2862280535770137E+20 * c1c2) -
            2.7053663393569854E+21;
  d_a_tmp = b_a_tmp * b_a_tmp;
  e_a_tmp = 1.0376293541461623E+19 * e_a_tmp_tmp;
  f_a_tmp = safe_pow(b_a_tmp_tmp + 3000.0, 3.0);
  a_tmp_tmp_tmp = 7.0 * cf1 / 400.0 + 23.0 * sf1 / 1000.0;
  b_a_tmp_tmp_tmp = 147.0 * c1s2 / 10000.0;
  c_a_tmp_tmp_tmp = 3.0 * c1c2 / 250.0;
  f_a_tmp_tmp = (a_tmp_tmp_tmp - b_a_tmp_tmp_tmp) - c_a_tmp_tmp_tmp;
  g_a_tmp = ((483.0 * cf1 / 500.0 - 147.0 * sf1 / 200.0) + 63.0 * c2s1 / 125.0) + 3087.0 * s1s2 / 5000.0;
  g_a_tmp_tmp = ((4.2410540364984156E+26 * cf1 - 3.2268889408140119E+26 * sf1) + 2.2127238451296081E+26 * c2s1) +
                2.71058671028377E+26 * s1s2;
  h_a_tmp_tmp = safe_pow(b_a_tmp, 3.0);
  i_a_tmp_tmp = 8.7160865748277631E+21 * e_a_tmp_tmp;
  j_a_tmp_tmp = 1.975646290294293E+22 * e_a_tmp_tmp;
  k_a_tmp_tmp = ((2.2274443469004284E+23 * cf1 - 1.6947946117720651E+23 * sf1) + 1.1621448766437018E+23 * c2s1) +
                1.4236274738885346E+23 * s1s2;
  e_a_tmp = std::sqrt(
    (((((((((((((((((((((1.0730744015857012E+30 * c1s2 - 1.128935157022907E+30 * sf1) - 2.0003592510593277E+30 * cf1) -
                       3.7715985671064229E+29 * c2s1) -
                      4.6202082447053686E+29 * s1s2) +
                     1.7441474113309633E+24 * d_a_tmp_tmp) +
                    2.1951625447714366E+21 * f_a_tmp) -
                   5.7646075230342349E+17 * safe_pow(b_a_tmp_tmp + 3000.0, 4.0)) +
                  3.4505479385580342E+25 * e_a_tmp_tmp) -
                 4.66933209365773E+19 * safe_pow(a_tmp, 4.0)) +
                3.3349145383199653E+24 * d_a_tmp) +
               9.6845406386975146E+20 * h_a_tmp_tmp) -
              5.7646075230342349E+17 * safe_pow(b_a_tmp, 4.0)) +
             j_a_tmp_tmp * (b_a_tmp_tmp + 3000.0)) +
            k_a_tmp_tmp * d_a_tmp_tmp) +
           2.1951625447714366E+21 * d_a_tmp * (b_a_tmp_tmp + 3000.0)) +
          i_a_tmp_tmp * b_a_tmp) -
         e_a_tmp * d_a_tmp_tmp) -
        1.152921504606847E+18 * d_a_tmp * d_a_tmp_tmp) -
       e_a_tmp * d_a_tmp) -
      g_a_tmp_tmp * (b_a_tmp_tmp + 3000.0)) +
     8.7597910333526627E+29 * c1c2) -
    2.7887404321092932E+31);
  l_a_tmp_tmp = 147.0 * cf2 / 10000.0;
  m_a_tmp_tmp = 3.0 * sf2 / 250.0;
  h_a_tmp = l_a_tmp_tmp - m_a_tmp_tmp;
  d_a_tmp_tmp_tmp = 1.4142135623730951 * h_a_tmp;
  n_a_tmp_tmp = d_a_tmp_tmp_tmp * e_a_tmp;
  e_a_tmp_tmp_tmp = 9.0 * e_a_tmp_tmp;
  f_a_tmp_tmp_tmp = 1.4142135623730951 * e_a_tmp;
  o_a_tmp_tmp =
    (((((((((((((1.674266832425483E+15 * c1s2 / 2.251799813685248E+18 - 2.798773803240181E+15 * sf1 / 2.147483648E+18) -
                1.882325848644649E+15 * cf1 / 2.68435456E+18) +
               7497.0 * c2s1 / 7.8125E+7) +
              367353.0 * s1s2 / 3.125E+9) -
             119.0 * d_a_tmp_tmp / 1.25E+11) +
            f_a_tmp / 1.0E+12) +
           1071.0 * e_a_tmp_tmp / 1.25E+11) -
          119.0 * d_a_tmp / 1.25E+11) +
         e_a_tmp_tmp_tmp * (f_a_tmp_tmp + 0.3) / 1.0E+8) +
        d_a_tmp * (f_a_tmp_tmp + 0.3) / 1.0E+8) -
       g_a_tmp * (f_a_tmp_tmp + 0.3) / 500.0) +
      5.46699373853219E+14 * c1c2 / 9.007199254740992E+17) -
     f_a_tmp_tmp_tmp * h_a_tmp / 1.073741824E+17) -
    0.01035875383296;
  p_a_tmp_tmp = a_tmp * c_a_tmp;
  g_a_tmp_tmp_tmp = 0.0 * h_a_tmp;
  q_a_tmp_tmp = g_a_tmp_tmp_tmp * (f_a_tmp_tmp + 0.2048);
  r_a_tmp_tmp = 1.2553783961295258E+17 * g_a_tmp;
  s_a_tmp_tmp = 1.7653758695571456E+16 * e_a_tmp_tmp;
  t_a_tmp_tmp = 1.961528743952384E+15 * d_a_tmp;
  u_a_tmp_tmp = (((((((((((((3.7336150363088269E+17 * c1s2 / 8.0 - 1.2782111910350037E+21 * sf1 / 15625.0) +
                            n_a_tmp_tmp * -584.58112) -
                           3.4386629775176232E+21 * cf1 / 78125.0) +
                          4.7057859178915273E+20 * c2s1 / 78125.0) +
                         2.8822938747085605E+21 * s1s2 / 390625.0) -
                        1.1671096026516685E+17 * d_a_tmp_tmp / 1.953125E+6) +
                       1.22595546497024E+14 * f_a_tmp / 1.953125E+6) +
                      1.0503986423865016E+18 * e_a_tmp_tmp / 1.953125E+6) -
                     1.1671096026516685E+17 * d_a_tmp / 1.953125E+6) +
                    s_a_tmp_tmp * (f_a_tmp_tmp + 0.3) / 3125.0) +
                   t_a_tmp_tmp * (f_a_tmp_tmp + 0.3) / 3125.0) -
                  r_a_tmp_tmp * (f_a_tmp_tmp + 0.3)) +
                 6.095698018463392E+17 * c1c2 / 16.0) -
                6.502077886360951E+17;
  v_a_tmp_tmp = 1.8652408456692805E+24 * (f_a_tmp_tmp + 0.2048);
  h_a_tmp_tmp_tmp = 3.2E+9 * a_tmp;
  w_a_tmp_tmp = h_a_tmp_tmp_tmp * c_a_tmp;
  x_a_tmp_tmp = ((((((80017.0 * cf1 / 1.25E+6 - 24353.0 * sf1 / 500000.0) + 10437.0 * c2s1 / 312500.0) +
                    511413.0 * s1s2 / 1.25E+7) -
                   497.0 * d_a_tmp_tmp / 1.5E+9) -
                  1491.0 * e_a_tmp_tmp / 5.0E+8) -
                 497.0 * d_a_tmp / 1.5E+9) +
                1.6758791194599998;
  i_a_tmp = (((x_a_tmp_tmp / a_tmp - u_a_tmp_tmp / c_a_tmp) + v_a_tmp_tmp * o_a_tmp_tmp / p_a_tmp_tmp) +
             q_a_tmp_tmp / w_a_tmp_tmp) +
            0.000212296;
  y_a_tmp_tmp = 1.1191445074015683E+18 * g_a_tmp;
  ab_a_tmp_tmp = 1.5737969635334554E+17 * e_a_tmp_tmp;
  bb_a_tmp_tmp = 1.7486632928149504E+16 * d_a_tmp;
  n_a_tmp_tmp = (((((((((((((8.3211061571546509E+17 * c1s2 / 2.0 - 1.1394994833083351E+22 * sf1 / 15625.0) -
                            n_a_tmp_tmp * 5211.42272) -
                           3.0654986543968765E+22 * cf1 / 78125.0) +
                          4.1951131859947786E+21 * c2s1 / 78125.0) +
                         2.5695068264218019E+22 * s1s2 / 390625.0) -
                        1.0404546592248955E+18 * d_a_tmp_tmp / 1.953125E+6) +
                       1.092914558009344E+15 * f_a_tmp / 1.953125E+6) +
                      9.36409193302406E+18 * e_a_tmp_tmp / 1.953125E+6) -
                     1.0404546592248955E+18 * d_a_tmp / 1.953125E+6) +
                    ab_a_tmp_tmp * (f_a_tmp_tmp + 0.3) / 3125.0) +
                   bb_a_tmp_tmp * (f_a_tmp_tmp + 0.3) / 3125.0) -
                  y_a_tmp_tmp * (f_a_tmp_tmp + 0.3)) +
                 1.3585479440252493E+18 * c1c2 / 4.0) -
                5.7964712278410629E+18;
  cb_a_tmp_tmp = 2.092297326882543E+23 * (f_a_tmp_tmp + 0.2048);
  i_a_tmp_tmp_tmp = 4.0E+8 * a_tmp;
  db_a_tmp_tmp = i_a_tmp_tmp_tmp * c_a_tmp;
  eb_a_tmp_tmp =
    ((((((35903.0 * cf1 / 5.0E+6 - 10927.0 * sf1 / 2.0E+6) + 4683.0 * c2s1 / 1.25E+6) + 229467.0 * s1s2 / 5.0E+7) -
       223.0 * d_a_tmp_tmp / 6.0E+9) -
      669.0 * e_a_tmp_tmp / 2.0E+9) -
     223.0 * d_a_tmp / 6.0E+9) +
    0.187988452535;
  j_a_tmp = (((n_a_tmp_tmp / c_a_tmp + eb_a_tmp_tmp / a_tmp) + cb_a_tmp_tmp * o_a_tmp_tmp / p_a_tmp_tmp) -
             q_a_tmp_tmp / db_a_tmp_tmp) -
            0.001892576;
  a_tmp_tmp = (a_tmp_tmp + 147.0 * c1s2) - 120.0 * c1c2;
  k_a_tmp = 49.0 * cf2 + 40.0 * sf2;
  l_a_tmp = c_a_tmp_tmp - 147.0 * s1s2;
  c_a_tmp_tmp = (a_tmp_tmp + 3000.0) * (a_tmp_tmp + 3000.0);
  fb_a_tmp_tmp = k_a_tmp * k_a_tmp;
  m_a_tmp = (((((6.1947012874481172E+20 * cf1 + 8.1416074063603827E+20 * sf1) + 5.2035490814564185E+20 * c1s2) -
               1.125899906842624E+15 * c_a_tmp_tmp) -
              1.0133099161583616E+16 * fb_a_tmp_tmp) -
             4.2477951685358518E+20 * c1c2) +
            7.8371820859486466E+21;
  n_a_tmp = (a_tmp_tmp_tmp + b_a_tmp_tmp_tmp) - c_a_tmp_tmp_tmp;
  o_a_tmp = l_a_tmp * l_a_tmp;
  p_a_tmp = safe_pow(a_tmp_tmp + 3000.0, 3.0);
  q_a_tmp = 1.0376293541461623E+19 * fb_a_tmp_tmp;
  r_a_tmp = l_a_tmp_tmp + m_a_tmp_tmp;
  l_a_tmp_tmp = ((6.3361245479259042E+26 * cf1 - 4.8209643299436228E+26 * sf1) + 3.3058041119613413E+26 * c2s1) -
                4.0496100371526431E+26 * s1s2;
  m_a_tmp_tmp = safe_pow(l_a_tmp, 3.0);
  gb_a_tmp_tmp = 7.8859830915108333E+21 * fb_a_tmp_tmp;
  hb_a_tmp_tmp = 3.2623066894355342E+22 * fb_a_tmp_tmp;
  ib_a_tmp_tmp = ((2.0153067900527685E+23 * cf1 - 1.5333856011271065E+23 * sf1) + 1.0514644122014444E+23 * c2s1) -
                 1.2880439049467694E+23 * s1s2;
  q_a_tmp = std::sqrt(
    (((((((((((((((((((((3.6274877138309625E+29 * cf1 + 2.6988001115069934E+29 * sf1) + 2.2096803736408614E+29 * c1s2) +
                       5.2013000371251354E+28 * c2s1) -
                      6.3715925454782912E+28 * s1s2) -
                     6.1762742871551752E+24 * c_a_tmp_tmp) +
                    3.6247852104839269E+21 * p_a_tmp) -
                   5.7646075230342349E+17 * safe_pow(a_tmp_tmp + 3000.0, 4.0)) -
                  4.3030074264699756E+24 * fb_a_tmp_tmp) -
                 4.66933209365773E+19 * safe_pow(k_a_tmp, 4.0)) -
                9.0328171705733629E+23 * o_a_tmp) +
               8.762203435012037E+20 * m_a_tmp_tmp) -
              5.7646075230342349E+17 * safe_pow(l_a_tmp, 4.0)) +
             hb_a_tmp_tmp * (a_tmp_tmp + 3000.0)) +
            ib_a_tmp_tmp * c_a_tmp_tmp) +
           3.6247852104839269E+21 * o_a_tmp * (a_tmp_tmp + 3000.0)) +
          gb_a_tmp_tmp * l_a_tmp) -
         q_a_tmp * c_a_tmp_tmp) -
        1.152921504606847E+18 * o_a_tmp * c_a_tmp_tmp) -
       q_a_tmp * o_a_tmp) -
      l_a_tmp_tmp * (a_tmp_tmp + 3000.0)) -
     1.8038207131762135E+29 * c1c2) +
    4.59634972951194E+30);
  s_a_tmp = ((437.0 * cf1 / 500.0 - 133.0 * sf1 / 200.0) + 57.0 * c2s1 / 125.0) - 2793.0 * s1s2 / 5000.0;
  a_tmp_tmp_tmp = 1.4142135623730951 * r_a_tmp;
  jb_a_tmp_tmp = a_tmp_tmp_tmp * q_a_tmp;
  b_a_tmp_tmp_tmp = 9.0 * fb_a_tmp_tmp;
  c_a_tmp_tmp_tmp = 1.4142135623730951 * q_a_tmp;
  kb_a_tmp_tmp = (((((((((((((1.258903E+6 * cf1 / 2.5E+9 + 1.058319524153E+12 * sf1 / 8.589934592E+14) +
                             3.680630476367269E+15 * c1s2 / 5.62949953421312E+18) -
                            22401.0 * c2s1 / 1.5625E+8) +
                           1.097649E+6 * s1s2 / 6.25E+9) +
                          393.0 * c_a_tmp_tmp / 2.5E+11) -
                         p_a_tmp / 1.0E+12) -
                        3537.0 * fb_a_tmp_tmp / 2.5E+11) +
                       393.0 * o_a_tmp / 2.5E+11) -
                      b_a_tmp_tmp_tmp * (n_a_tmp + 0.3) / 1.0E+8) -
                     o_a_tmp * (n_a_tmp + 0.3) / 1.0E+8) +
                    s_a_tmp * (n_a_tmp + 0.3) / 500.0) -
                   4.807354091581739E+15 * c1c2 / 9.007199254740992E+18) +
                  c_a_tmp_tmp_tmp * r_a_tmp / 1.073741824E+17) +
                 0.0063513098879999993;
  lb_a_tmp_tmp = k_a_tmp * m_a_tmp;
  j_a_tmp_tmp_tmp = 0.0 * r_a_tmp;
  mb_a_tmp_tmp = j_a_tmp_tmp_tmp * (n_a_tmp + 0.1428);
  nb_a_tmp_tmp = 2.4319437987800678E+17 * s_a_tmp;
  ob_a_tmp_tmp = 3.4199209670344704E+16 * fb_a_tmp_tmp;
  pb_a_tmp_tmp = 3.799912185593856E+15 * o_a_tmp;
  qb_a_tmp_tmp = (((((((((((((jb_a_tmp_tmp * 1132.46208 + 4.7837208501806621E+21 * cf1 / 78125.0) +
                             3.7453335300841144E+18 * sf1 / 25.0) +
                            3.97508091447665E+17 * c1s2 / 5.0) -
                           1.3619493259118075E+21 * c2s1 / 78125.0) +
                          8.3419396212098209E+21 * s1s2 / 390625.0) +
                         3.7334137223459635E+17 * c_a_tmp_tmp / 1.953125E+6) -
                        2.37494511599616E+14 * p_a_tmp / 1.953125E+6) -
                       3.3600723501113672E+18 * fb_a_tmp_tmp / 1.953125E+6) +
                      3.7334137223459635E+17 * o_a_tmp / 1.953125E+6) -
                     ob_a_tmp_tmp * (n_a_tmp + 0.3) / 3125.0) -
                    pb_a_tmp_tmp * (n_a_tmp + 0.3) / 3125.0) +
                   nb_a_tmp_tmp * (n_a_tmp + 0.3)) -
                  1.2979856047270696E+17 * c1c2 / 2.0) +
                 7.7230143481260634E+17;
  rb_a_tmp_tmp = 3.730481691338561E+24 * (n_a_tmp + 0.1428);
  k_a_tmp_tmp_tmp = 5.0 * k_a_tmp;
  sb_a_tmp_tmp = k_a_tmp_tmp_tmp * m_a_tmp;
  tb_a_tmp_tmp = mb_a_tmp_tmp / sb_a_tmp_tmp;
  ub_a_tmp_tmp =
    ((((((66101.0 * sf1 / 1.5E+6 - 217189.0 * cf1 / 3.75E+6) - 9443.0 * c2s1 / 312500.0) + 462707.0 * s1s2 / 1.25E+7) +
       497.0 * c_a_tmp_tmp / 1.5E+9) +
      1491.0 * fb_a_tmp_tmp / 5.0E+8) +
     497.0 * o_a_tmp / 1.5E+9) -
    1.4736699413333332;
  t_a_tmp =
    (((rb_a_tmp_tmp * kb_a_tmp_tmp / lb_a_tmp_tmp - ub_a_tmp_tmp / k_a_tmp) - qb_a_tmp_tmp / m_a_tmp) + tb_a_tmp_tmp) +
    0.000339552;
  vb_a_tmp_tmp = 2.2382890148031365E+18 * s_a_tmp;
  wb_a_tmp_tmp = 3.1475939270669107E+17 * fb_a_tmp_tmp;
  xb_a_tmp_tmp = 3.4973265856299008E+16 * o_a_tmp;
  jb_a_tmp_tmp = (((((((((((((jb_a_tmp_tmp * 10422.84544 + 4.402794930629239E+22 * cf1 / 78125.0) +
                             3.4470940082440831E+19 * sf1 / 25.0) +
                            3.6585466935090652E+18 * c1s2 / 5.0) -
                           1.2534978055151265E+22 * c2s1 / 78125.0) +
                          7.67767405878015E+22 * s1s2 / 390625.0) +
                         3.4361233703813775E+18 * c_a_tmp_tmp / 1.953125E+6) -
                        2.185829116018688E+15 * p_a_tmp / 1.953125E+6) -
                       3.0925110333432398E+19 * fb_a_tmp_tmp / 1.953125E+6) +
                      3.4361233703813775E+18 * o_a_tmp / 1.953125E+6) -
                     wb_a_tmp_tmp * (n_a_tmp + 0.3) / 3125.0) -
                    xb_a_tmp_tmp * (n_a_tmp + 0.3) / 3125.0) +
                   vb_a_tmp_tmp * (n_a_tmp + 0.3)) -
                  2.3892549835161242E+18 * c1c2 / 4.0) +
                 7.1080335759604685E+18;
  yb_a_tmp_tmp = 4.0532396646334464E+23 * (n_a_tmp + 0.1428);
  l_a_tmp_tmp_tmp = 2.0E+8 * k_a_tmp;
  ac_a_tmp_tmp = l_a_tmp_tmp_tmp * m_a_tmp;
  bc_a_tmp_tmp =
    ((((((1197.0 * sf1 / 250000.0 - 3933.0 * cf1 / 625000.0) - 513.0 * c2s1 / 156250.0) + 25137.0 * s1s2 / 6.25E+6) +
       9.0 * c_a_tmp_tmp / 2.5E+8) +
      81.0 * fb_a_tmp_tmp / 2.5E+8) +
     9.0 * o_a_tmp / 2.5E+8) -
    0.16011705599999998;
  u_a_tmp = (((bc_a_tmp_tmp / k_a_tmp - jb_a_tmp_tmp / m_a_tmp) - yb_a_tmp_tmp * kb_a_tmp_tmp / lb_a_tmp_tmp) +
             mb_a_tmp_tmp / ac_a_tmp_tmp) +
            0.003125136;
  ar_tmp = 2.0752587082923246E+19 * e_a_tmp_tmp;
  b_ar_tmp = 2.305843009213694E+18 * b_a_tmp_tmp;
  ar_tmp_tmp =
    ((((((((((((((((((((((1.128935157022907E+30 * cf1 - 2.0003592510593277E+30 * sf1) + 4.6202082447053686E+29 * c1s2) +
                        8.7597910333526627E+29 * c2s1) +
                       g_a_tmp_tmp * b_a_tmp) +
                      1.0730744015857012E+30 * s1s2) +
                     (((1.6947946117720651E+23 * cf1 + 2.2274443469004284E+23 * sf1) - 1.4236274738885346E+23 * c1s2) -
                      1.1621448766437018E+23 * c1c2) *
                       d_a_tmp_tmp) -
                    2.1951625447714366E+21 * h_a_tmp_tmp) +
                   i_a_tmp_tmp * b_a_tmp_tmp) +
                  2.9053621916092544E+21 * b_a_tmp_tmp * d_a_tmp) -
                 b_ar_tmp * h_a_tmp_tmp) -
                6.58548763431431E+21 * b_a_tmp * d_a_tmp_tmp) +
               2.305843009213694E+18 * h_a_tmp_tmp * (b_a_tmp_tmp + 3000.0)) +
              2.305843009213694E+18 * b_a_tmp * f_a_tmp) -
             j_a_tmp_tmp * b_a_tmp) -
            (((3.2268889408140119E+26 * cf1 + 4.2410540364984156E+26 * sf1) - 2.71058671028377E+26 * c1s2) -
             2.2127238451296081E+26 * c1c2) *
              (b_a_tmp_tmp + 3000.0)) +
           6.6698290766399307E+24 * b_a_tmp_tmp * b_a_tmp) -
          3.4882948226619266E+24 * b_a_tmp * (b_a_tmp_tmp + 3000.0)) +
         3.7715985671064229E+29 * c1c2) -
        ar_tmp * b_a_tmp_tmp * b_a_tmp) +
       ar_tmp * b_a_tmp * (b_a_tmp_tmp + 3000.0)) +
      4.3903250895428733E+21 * b_a_tmp_tmp * b_a_tmp * (b_a_tmp_tmp + 3000.0)) -
     2.0 * k_a_tmp_tmp * b_a_tmp * (b_a_tmp_tmp + 3000.0)) -
    b_ar_tmp * b_a_tmp * d_a_tmp_tmp;
  ar_tmp = d_a_tmp_tmp_tmp * ar_tmp_tmp;
  b_ar_tmp = ar_tmp * 292.29056;
  i_a_tmp_tmp = ar_tmp * -2605.71136;
  if (i_a_tmp_tmp == 0.0) {
    im = 0.0;
    re = b_ar_tmp / e_a_tmp;
  } else {
    im = i_a_tmp_tmp / e_a_tmp;
    if (b_ar_tmp == 0.0) {
      re = 0.0;
    } else {
      re = b_ar_tmp / e_a_tmp;
    }
  }
  J_tmp_tmp = (23.0 * cf1 / 1000.0 - 7.0 * sf1 / 400.0) + 3.0 * c2s1 / 250.0;
  b_J_tmp_tmp = 147.0 * s1s2 / 10000.0;
  J_tmp = J_tmp_tmp + b_J_tmp_tmp;
  b_J_tmp = ((147.0 * cf1 / 200.0 + 483.0 * sf1 / 500.0) - 3087.0 * c1s2 / 5000.0) - 63.0 * c1c2 / 125.0;
  c_J_tmp = (((2.4652704360226095E+20 * cf1 - 1.8757492447998116E+20 * sf1) + 1.2862280535770137E+20 * c2s1) +
             1.5756293656318417E+20 * s1s2) -
            1.125899906842624E+15 * b_a_tmp * (b_a_tmp_tmp + 3000.0);
  d_J_tmp = c_a_tmp * c_a_tmp;
  e_J_tmp = (((((((((((((1.882325848644649E+15 * sf1 / 2.68435456E+18 - 2.798773803240181E+15 * cf1 / 2.147483648E+18) +
                        367353.0 * c1s2 / 3.125E+9) -
                       5.46699373853219E+14 * c2s1 / 9.007199254740992E+17) -
                      g_a_tmp * J_tmp / 500.0) -
                     1.674266832425483E+15 * s1s2 / 2.251799813685248E+18) +
                    3.0 * b_a_tmp * d_a_tmp_tmp / 1.0E+12) +
                   e_a_tmp_tmp_tmp * J_tmp / 1.0E+8) +
                  d_a_tmp * J_tmp / 1.0E+8) +
                 b_J_tmp * (f_a_tmp_tmp + 0.3) / 500.0) +
                119.0 * b_a_tmp_tmp * b_a_tmp / 6.25E+10) -
               119.0 * b_a_tmp * (b_a_tmp_tmp + 3000.0) / 6.25E+10) +
              7497.0 * c1c2 / 7.8125E+7) +
             1.4142135623730951 * (ar_tmp_tmp / e_a_tmp) * h_a_tmp / 2.147483648E+17) -
            b_a_tmp_tmp * b_a_tmp * (f_a_tmp_tmp + 0.3) / 5.0E+7;
  f_J_tmp = a_tmp * d_J_tmp;
  g_J_tmp = g_a_tmp_tmp_tmp * J_tmp;
  ar_tmp_tmp = q_a_tmp_tmp * c_J_tmp;
  h_J_tmp = i_a_tmp * i_a_tmp;
  i_J_tmp = q_a_tmp_tmp / (8.0E+8 * a_tmp * c_a_tmp);
  j_J_tmp = i_a_tmp_tmp_tmp * d_J_tmp;
  k_J_tmp = q_a_tmp_tmp / (6.4E+9 * a_tmp * c_a_tmp);
  l_J_tmp = h_a_tmp_tmp_tmp * d_J_tmp;
  m_J_tmp = j_a_tmp * j_a_tmp + h_J_tmp;
  J_temp[0] =
    (((((((((2.092297326882543E+23 * J_tmp * o_a_tmp_tmp / p_a_tmp_tmp +
             ((((((((((((((3.0654986543968765E+22 * sf1 / 78125.0 - 1.1394994833083351E+22 * cf1 / 15625.0) - im) +
                         2.5695068264218019E+22 * c1s2 / 390625.0) -
                        1.3585479440252493E+18 * c2s1 / 4.0) -
                       y_a_tmp_tmp * J_tmp) -
                      8.3211061571546509E+17 * s1s2 / 2.0) +
                     3.278743674028032E+15 * b_a_tmp * d_a_tmp_tmp / 1.953125E+6) +
                    ab_a_tmp_tmp * J_tmp / 3125.0) +
                   bb_a_tmp_tmp * J_tmp / 3125.0) +
                  1.1191445074015683E+18 * b_J_tmp * (f_a_tmp_tmp + 0.3)) +
                 2.080909318449791E+18 * b_a_tmp_tmp * b_a_tmp / 1.953125E+6) -
                2.080909318449791E+18 * b_a_tmp * (b_a_tmp_tmp + 3000.0) / 1.953125E+6) +
               4.1951131859947786E+21 * c1c2 / 78125.0) -
              3.4973265856299008E+16 * b_a_tmp_tmp * b_a_tmp * (f_a_tmp_tmp + 0.3) / 3125.0) /
               c_a_tmp) +
            c_J_tmp * n_a_tmp_tmp / d_J_tmp) -
           (((((10927.0 * cf1 / 2.0E+6 + 35903.0 * sf1 / 5.0E+6) - 229467.0 * c1s2 / 5.0E+7) -
              223.0 * b_a_tmp_tmp * b_a_tmp / 3.0E+9) +
             223.0 * b_a_tmp * (b_a_tmp_tmp + 3000.0) / 3.0E+9) -
            4683.0 * c1c2 / 1.25E+6) /
             a_tmp) +
          cb_a_tmp_tmp * e_J_tmp / p_a_tmp_tmp) +
         cb_a_tmp_tmp * c_J_tmp * o_a_tmp_tmp / f_J_tmp) +
        i_J_tmp) -
       g_J_tmp / db_a_tmp_tmp) -
      ar_tmp_tmp / j_J_tmp) /
       i_a_tmp -
     j_a_tmp *
       ((((((((1.8652408456692805E+24 * J_tmp * o_a_tmp_tmp / p_a_tmp_tmp -
               ((((((((((((((3.4386629775176232E+21 * sf1 / 78125.0 - 1.2782111910350037E+21 * cf1 / 15625.0) + re) +
                           2.8822938747085605E+21 * c1s2 / 390625.0) -
                          6.095698018463392E+17 * c2s1 / 16.0) -
                         r_a_tmp_tmp * J_tmp) -
                        3.7336150363088269E+17 * s1s2 / 8.0) +
                       3.67786639491072E+14 * b_a_tmp * d_a_tmp_tmp / 1.953125E+6) +
                      s_a_tmp_tmp * J_tmp / 3125.0) +
                     t_a_tmp_tmp * J_tmp / 3125.0) +
                    1.2553783961295258E+17 * b_J_tmp * (f_a_tmp_tmp + 0.3)) +
                   2.334219205303337E+17 * b_a_tmp_tmp * b_a_tmp / 1.953125E+6) -
                  2.334219205303337E+17 * b_a_tmp * (b_a_tmp_tmp + 3000.0) / 1.953125E+6) +
                 4.7057859178915273E+20 * c1c2 / 78125.0) -
                3.923057487904768E+15 * b_a_tmp_tmp * b_a_tmp * (f_a_tmp_tmp + 0.3) / 3125.0) /
                 c_a_tmp) -
              c_J_tmp * u_a_tmp_tmp / d_J_tmp) -
             (((((24353.0 * cf1 / 500000.0 + 80017.0 * sf1 / 1.25E+6) - 511413.0 * c1s2 / 1.25E+7) -
                497.0 * b_a_tmp_tmp * b_a_tmp / 7.5E+8) +
               497.0 * b_a_tmp * (b_a_tmp_tmp + 3000.0) / 7.5E+8) -
              10437.0 * c1c2 / 312500.0) /
               a_tmp) +
            v_a_tmp_tmp * e_J_tmp / p_a_tmp_tmp) +
           v_a_tmp_tmp * c_J_tmp * o_a_tmp_tmp / f_J_tmp) -
          k_J_tmp) +
         g_J_tmp / w_a_tmp_tmp) +
        ar_tmp_tmp / l_J_tmp) /
       h_J_tmp) *
    h_J_tmp / m_J_tmp;
  ar_tmp = 147.0 * c2s1 - 120.0 * s1s2;
  b_ar_tmp = 120.0 * c1s2 - 147.0 * c1c2;
  t_a_tmp_tmp = 40.0 * cf2 + 49.0 * sf2;
  j_a_tmp_tmp = 2.305843009213694E+18 * b_ar_tmp;
  i_a_tmp_tmp_tmp = 2.305843009213694E+18 * ar_tmp;
  i_a_tmp_tmp = 2.0752587082923246E+19 * t_a_tmp_tmp * a_tmp;
  ar_tmp_tmp =
    ((((((((((((((((((((((((((2.9053621916092544E+21 * ar_tmp * d_a_tmp - 4.6202082447053686E+29 * c2s1) -
                             8.7597910333526627E+29 * c1s2) -
                            i_a_tmp_tmp_tmp * h_a_tmp_tmp) +
                           3.7715985671064229E+29 * s1s2) +
                          1.8677328374630921E+20 * t_a_tmp_tmp * safe_pow(a_tmp, 3.0)) +
                         3.4882948226619266E+24 * b_ar_tmp * (b_a_tmp_tmp + 3000.0)) -
                        b_ar_tmp * g_a_tmp_tmp) -
                       (2.71058671028377E+26 * c2s1 - 2.2127238451296081E+26 * s1s2) * (b_a_tmp_tmp + 3000.0)) +
                      6.6698290766399307E+24 * ar_tmp * b_a_tmp) +
                     6.58548763431431E+21 * b_ar_tmp * d_a_tmp_tmp) -
                    j_a_tmp_tmp * f_a_tmp) +
                   1.975646290294293E+22 * b_ar_tmp * e_a_tmp_tmp) -
                  6.9010958771160684E+25 * t_a_tmp_tmp * a_tmp) +
                 2.1951625447714366E+21 * b_ar_tmp * d_a_tmp) +
                (1.4236274738885346E+23 * c2s1 - 1.1621448766437018E+23 * s1s2) * d_a_tmp_tmp) +
               1.0730744015857012E+30 * c1c2) +
              8.7160865748277631E+21 * ar_tmp * e_a_tmp_tmp) +
             i_a_tmp_tmp * d_a_tmp) +
            2.0 * b_ar_tmp * k_a_tmp_tmp * (b_a_tmp_tmp + 3000.0)) +
           4.3903250895428733E+21 * ar_tmp * b_a_tmp * (b_a_tmp_tmp + 3000.0)) -
          2.0752587082923246E+19 * b_ar_tmp * e_a_tmp_tmp * (b_a_tmp_tmp + 3000.0)) -
         3.951292580588586E+22 * t_a_tmp_tmp * a_tmp * (b_a_tmp_tmp + 3000.0)) -
        j_a_tmp_tmp * d_a_tmp * (b_a_tmp_tmp + 3000.0)) -
       1.7432173149655526E+22 * t_a_tmp_tmp * a_tmp * b_a_tmp) -
      i_a_tmp_tmp_tmp * b_a_tmp * d_a_tmp_tmp) -
     2.0752587082923246E+19 * ar_tmp * e_a_tmp_tmp * b_a_tmp) +
    i_a_tmp_tmp * d_a_tmp_tmp;
  j_a_tmp_tmp = d_a_tmp_tmp_tmp * ar_tmp_tmp;
  i_a_tmp_tmp_tmp = j_a_tmp_tmp * -292.29056;
  i_a_tmp_tmp = j_a_tmp_tmp * 2605.71136;
  if (i_a_tmp_tmp == 0.0) {
    im = 0.0;
    re = i_a_tmp_tmp_tmp / e_a_tmp;
  } else {
    im = i_a_tmp_tmp / e_a_tmp;
    if (i_a_tmp_tmp_tmp == 0.0) {
      re = 0.0;
    } else {
      re = i_a_tmp_tmp_tmp / e_a_tmp;
    }
  }
  q_a_tmp_tmp = 3.0 * c1s2 / 250.0;
  h_a_tmp_tmp_tmp = 147.0 * c1c2 / 10000.0;
  J_tmp = q_a_tmp_tmp - h_a_tmp_tmp_tmp;
  b_J_tmp = 3087.0 * c2s1 / 5000.0 - 63.0 * s1s2 / 125.0;
  r_a_tmp_tmp = 3.0 * cf2 / 250.0;
  s_a_tmp_tmp = 147.0 * sf2 / 10000.0;
  c_J_tmp = r_a_tmp_tmp + s_a_tmp_tmp;
  e_J_tmp = ((1.2862280535770137E+20 * c1s2 - 1.125899906842624E+15 * b_ar_tmp * (b_a_tmp_tmp + 3000.0)) +
             1.0133099161583616E+16 * t_a_tmp_tmp * a_tmp) -
            1.5756293656318417E+20 * c1c2;
  g_J_tmp = 1.4142135623730951 * c_J_tmp * e_a_tmp;
  ar_tmp_tmp = ((((((((((((((5.46699373853219E+14 * c1s2 / 9.007199254740992E+17 - 367353.0 * c2s1 / 3.125E+9) +
                            7497.0 * s1s2 / 7.8125E+7) +
                           119.0 * b_ar_tmp * (b_a_tmp_tmp + 3000.0) / 6.25E+10) +
                          J_tmp * g_a_tmp / 500.0) +
                         b_J_tmp * (f_a_tmp_tmp + 0.3) / 500.0) +
                        119.0 * ar_tmp * b_a_tmp / 6.25E+10) -
                       3.0 * b_ar_tmp * d_a_tmp_tmp / 1.0E+12) -
                      9.0 * J_tmp * e_a_tmp_tmp / 1.0E+8) +
                     1071.0 * t_a_tmp_tmp * a_tmp / 6.25E+10) -
                    J_tmp * d_a_tmp / 1.0E+8) -
                   1.674266832425483E+15 * c1c2 / 2.251799813685248E+18) +
                  1.4142135623730951 * (ar_tmp_tmp / e_a_tmp) * h_a_tmp / 2.147483648E+17) -
                 ar_tmp * b_a_tmp * (f_a_tmp_tmp + 0.3) / 5.0E+7) +
                9.0 * t_a_tmp_tmp * a_tmp * (f_a_tmp_tmp + 0.3) / 5.0E+7) -
               f_a_tmp_tmp_tmp * c_J_tmp / 1.073741824E+17;
  i_a_tmp_tmp_tmp = e_a_tmp_tmp * c_a_tmp;
  e_a_tmp_tmp_tmp = 0.0 * J_tmp * h_a_tmp;
  c_J_tmp = 0.0 * c_J_tmp * (f_a_tmp_tmp + 0.2048);
  j_a_tmp_tmp = g_a_tmp_tmp_tmp * e_J_tmp * (f_a_tmp_tmp + 0.2048);
  i_a_tmp_tmp = 0.0 * t_a_tmp_tmp * h_a_tmp * (f_a_tmp_tmp + 0.2048);
  J_temp[2] =
    -(((((((((((((((((((((((((((((im + g_J_tmp * -5211.42272) + 1.3585479440252493E+18 * c1s2 / 4.0) -
                                2.5695068264218019E+22 * c2s1 / 390625.0) +
                               4.1951131859947786E+21 * s1s2 / 78125.0) +
                              2.080909318449791E+18 * b_ar_tmp * (b_a_tmp_tmp + 3000.0) / 1.953125E+6) +
                             1.1191445074015683E+18 * J_tmp * g_a_tmp) +
                            1.1191445074015683E+18 * b_J_tmp * (f_a_tmp_tmp + 0.3)) +
                           2.080909318449791E+18 * ar_tmp * b_a_tmp / 1.953125E+6) -
                          3.278743674028032E+15 * b_ar_tmp * d_a_tmp_tmp / 1.953125E+6) -
                         1.5737969635334554E+17 * J_tmp * e_a_tmp_tmp / 3125.0) +
                        1.8728183866048119E+19 * t_a_tmp_tmp * a_tmp / 1.953125E+6) -
                       1.7486632928149504E+16 * J_tmp * d_a_tmp / 3125.0) -
                      8.3211061571546509E+17 * c1c2 / 2.0) -
                     3.4973265856299008E+16 * ar_tmp * b_a_tmp * (f_a_tmp_tmp + 0.3) / 3125.0) +
                    3.1475939270669107E+17 * t_a_tmp_tmp * a_tmp * (f_a_tmp_tmp + 0.3) / 3125.0) /
                     c_a_tmp +
                   ((((4683.0 * s1s2 / 1.25E+6 - 229467.0 * c2s1 / 5.0E+7) +
                      223.0 * b_ar_tmp * (b_a_tmp_tmp + 3000.0) / 3.0E+9) +
                     223.0 * ar_tmp * b_a_tmp / 3.0E+9) -
                    669.0 * t_a_tmp_tmp * a_tmp / 1.0E+9) /
                     a_tmp) -
                  t_a_tmp_tmp * eb_a_tmp_tmp / e_a_tmp_tmp) -
                 e_J_tmp * n_a_tmp_tmp / d_J_tmp) +
                cb_a_tmp_tmp * ar_tmp_tmp / p_a_tmp_tmp) -
               2.092297326882543E+23 * J_tmp * o_a_tmp_tmp / p_a_tmp_tmp) -
              2.092297326882543E+23 * e_J_tmp * (f_a_tmp_tmp + 0.2048) * o_a_tmp_tmp / f_J_tmp) -
             2.092297326882543E+23 * t_a_tmp_tmp * (f_a_tmp_tmp + 0.2048) * o_a_tmp_tmp / i_a_tmp_tmp_tmp) +
            i_J_tmp) +
           e_a_tmp_tmp_tmp / db_a_tmp_tmp) -
          c_J_tmp / db_a_tmp_tmp) +
         j_a_tmp_tmp / j_J_tmp) +
        i_a_tmp_tmp / (4.0E+8 * e_a_tmp_tmp * c_a_tmp)) /
         i_a_tmp +
       j_a_tmp *
         ((((((((((((t_a_tmp_tmp * x_a_tmp_tmp / e_a_tmp_tmp -
                     (((((((((((((((re + g_J_tmp * 584.58112) - 6.095698018463392E+17 * c1s2 / 16.0) +
                                  2.8822938747085605E+21 * c2s1 / 390625.0) -
                                 4.7057859178915273E+20 * s1s2 / 78125.0) -
                                2.334219205303337E+17 * b_ar_tmp * (b_a_tmp_tmp + 3000.0) / 1.953125E+6) -
                               1.2553783961295258E+17 * J_tmp * g_a_tmp) -
                              1.2553783961295258E+17 * b_J_tmp * (f_a_tmp_tmp + 0.3)) -
                             2.334219205303337E+17 * ar_tmp * b_a_tmp / 1.953125E+6) +
                            3.67786639491072E+14 * b_ar_tmp * d_a_tmp_tmp / 1.953125E+6) +
                           1.7653758695571456E+16 * J_tmp * e_a_tmp_tmp / 3125.0) -
                          2.1007972847730033E+18 * t_a_tmp_tmp * a_tmp / 1.953125E+6) +
                         1.961528743952384E+15 * J_tmp * d_a_tmp / 3125.0) +
                        3.7336150363088269E+17 * c1c2 / 8.0) +
                       3.923057487904768E+15 * ar_tmp * b_a_tmp * (f_a_tmp_tmp + 0.3) / 3125.0) -
                      3.5307517391142912E+16 * t_a_tmp_tmp * a_tmp * (f_a_tmp_tmp + 0.3) / 3125.0) /
                       c_a_tmp) -
                    ((((10437.0 * s1s2 / 312500.0 - 511413.0 * c2s1 / 1.25E+7) +
                       497.0 * b_ar_tmp * (b_a_tmp_tmp + 3000.0) / 7.5E+8) +
                      497.0 * ar_tmp * b_a_tmp / 7.5E+8) -
                     1491.0 * t_a_tmp_tmp * a_tmp / 2.5E+8) /
                      a_tmp) -
                   e_J_tmp * u_a_tmp_tmp / d_J_tmp) -
                  v_a_tmp_tmp * ar_tmp_tmp / p_a_tmp_tmp) +
                 1.8652408456692805E+24 * J_tmp * o_a_tmp_tmp / p_a_tmp_tmp) +
                1.8652408456692805E+24 * e_J_tmp * (f_a_tmp_tmp + 0.2048) * o_a_tmp_tmp / f_J_tmp) +
               1.8652408456692805E+24 * t_a_tmp_tmp * (f_a_tmp_tmp + 0.2048) * o_a_tmp_tmp / i_a_tmp_tmp_tmp) +
              k_J_tmp) +
             e_a_tmp_tmp_tmp / w_a_tmp_tmp) -
            c_J_tmp / w_a_tmp_tmp) +
           j_a_tmp_tmp / l_J_tmp) +
          i_a_tmp_tmp / (3.2E+9 * e_a_tmp_tmp * c_a_tmp)) /
         h_J_tmp) *
      h_J_tmp) /
    m_J_tmp;
  ar_tmp = 2.0752587082923246E+19 * fb_a_tmp_tmp;
  b_ar_tmp = 2.305843009213694E+18 * a_tmp_tmp;
  ar_tmp_tmp =
    ((((((((((((((((((((((2.6988001115069934E+29 * cf1 - 3.6274877138309625E+29 * sf1) -
                         6.3715925454782912E+28 * c1s2) +
                        1.8038207131762135E+29 * c2s1) -
                       l_a_tmp_tmp * l_a_tmp) -
                      2.2096803736408614E+29 * s1s2) -
                     (((1.5333856011271065E+23 * cf1 + 2.0153067900527685E+23 * sf1) + 1.2880439049467694E+23 * c1s2) -
                      1.0514644122014444E+23 * c1c2) *
                       c_a_tmp_tmp) +
                    3.6247852104839269E+21 * m_a_tmp_tmp) -
                   gb_a_tmp_tmp * a_tmp_tmp) -
                  2.6286610305036111E+21 * a_tmp_tmp * o_a_tmp) +
                 b_ar_tmp * m_a_tmp_tmp) +
                1.0874355631451781E+22 * l_a_tmp * c_a_tmp_tmp) -
               2.305843009213694E+18 * m_a_tmp_tmp * (a_tmp_tmp + 3000.0)) -
              2.305843009213694E+18 * l_a_tmp * p_a_tmp) +
             hb_a_tmp_tmp * l_a_tmp) +
            (((4.8209643299436228E+26 * cf1 + 6.3361245479259042E+26 * sf1) + 4.0496100371526431E+26 * c1s2) -
             3.3058041119613413E+26 * c1c2) *
              (a_tmp_tmp + 3000.0)) +
           1.8065634341146726E+24 * a_tmp_tmp * l_a_tmp) -
          1.235254857431035E+25 * l_a_tmp * (a_tmp_tmp + 3000.0)) +
         5.2013000371251354E+28 * c1c2) +
        ar_tmp * a_tmp_tmp * l_a_tmp) -
       ar_tmp * l_a_tmp * (a_tmp_tmp + 3000.0)) -
      7.2495704209678538E+21 * a_tmp_tmp * l_a_tmp * (a_tmp_tmp + 3000.0)) +
     2.0 * ib_a_tmp_tmp * l_a_tmp * (a_tmp_tmp + 3000.0)) +
    b_ar_tmp * l_a_tmp * c_a_tmp_tmp;
  ar_tmp = a_tmp_tmp_tmp * ar_tmp_tmp;
  b_ar_tmp = ar_tmp * 566.23104;
  i_a_tmp_tmp = ar_tmp * 5211.42272;
  if (i_a_tmp_tmp == 0.0) {
    im = 0.0;
    re = b_ar_tmp / q_a_tmp;
  } else {
    im = i_a_tmp_tmp / q_a_tmp;
    if (b_ar_tmp == 0.0) {
      re = 0.0;
    } else {
      re = b_ar_tmp / q_a_tmp;
    }
  }
  J_tmp = J_tmp_tmp - b_J_tmp_tmp;
  b_J_tmp = ((133.0 * cf1 / 200.0 + 437.0 * sf1 / 500.0) + 2793.0 * c1s2 / 5000.0) - 57.0 * c1c2 / 125.0;
  c_J_tmp = (((6.1947012874481172E+20 * sf1 - 8.1416074063603827E+20 * cf1) - 4.2477951685358518E+20 * c2s1) +
             5.2035490814564185E+20 * s1s2) +
            2.251799813685248E+15 * l_a_tmp * (a_tmp_tmp + 3000.0);
  d_J_tmp = m_a_tmp * m_a_tmp;
  e_J_tmp = (((((((((((((1.258903E+6 * sf1 / 2.5E+9 - 1.058319524153E+12 * cf1 / 8.589934592E+14) -
                        1.097649E+6 * c1s2 / 6.25E+9) -
                       4.807354091581739E+15 * c2s1 / 9.007199254740992E+18) -
                      s_a_tmp * J_tmp / 500.0) +
                     3.680630476367269E+15 * s1s2 / 5.62949953421312E+18) +
                    3.0 * l_a_tmp * c_a_tmp_tmp / 1.0E+12) +
                   b_a_tmp_tmp_tmp * J_tmp / 1.0E+8) +
                  o_a_tmp * J_tmp / 1.0E+8) +
                 b_J_tmp * (n_a_tmp + 0.3) / 500.0) +
                393.0 * a_tmp_tmp * l_a_tmp / 1.25E+11) -
               393.0 * l_a_tmp * (a_tmp_tmp + 3000.0) / 1.25E+11) +
              22401.0 * c1c2 / 1.5625E+8) -
             a_tmp_tmp * l_a_tmp * (n_a_tmp + 0.3) / 5.0E+7) -
            1.4142135623730951 * (ar_tmp_tmp / q_a_tmp) * r_a_tmp / 2.147483648E+17;
  f_J_tmp = k_a_tmp * d_J_tmp;
  g_J_tmp = j_a_tmp_tmp_tmp * J_tmp;
  ar_tmp_tmp = mb_a_tmp_tmp * c_J_tmp;
  h_J_tmp = t_a_tmp * t_a_tmp;
  i_J_tmp = mb_a_tmp_tmp / (4.0E+8 * k_a_tmp * m_a_tmp);
  j_J_tmp = l_a_tmp_tmp_tmp * d_J_tmp;
  k_J_tmp = k_a_tmp_tmp_tmp * d_J_tmp;
  l_J_tmp = h_J_tmp + u_a_tmp * u_a_tmp;
  J_temp[1] =
    ((((((((((((((1197.0 * cf1 / 250000.0 + 3933.0 * sf1 / 625000.0) + 25137.0 * c1s2 / 6.25E+6) -
                9.0 * a_tmp_tmp * l_a_tmp / 1.25E+8) +
               9.0 * l_a_tmp * (a_tmp_tmp + 3000.0) / 1.25E+8) -
              513.0 * c1c2 / 156250.0) /
               k_a_tmp +
             ((((((((((((((4.402794930629239E+22 * sf1 / 78125.0 - 3.4470940082440831E+19 * cf1 / 25.0) - im) -
                         7.67767405878015E+22 * c1s2 / 390625.0) -
                        2.3892549835161242E+18 * c2s1 / 4.0) -
                       vb_a_tmp_tmp * J_tmp) +
                      3.6585466935090652E+18 * s1s2 / 5.0) +
                     6.557487348056064E+15 * l_a_tmp * c_a_tmp_tmp / 1.953125E+6) +
                    wb_a_tmp_tmp * J_tmp / 3125.0) +
                   xb_a_tmp_tmp * J_tmp / 3125.0) +
                  2.2382890148031365E+18 * b_J_tmp * (n_a_tmp + 0.3)) +
                 6.8722467407627551E+18 * a_tmp_tmp * l_a_tmp / 1.953125E+6) -
                6.8722467407627551E+18 * l_a_tmp * (a_tmp_tmp + 3000.0) / 1.953125E+6) +
               1.2534978055151265E+22 * c1c2 / 78125.0) -
              6.9946531712598016E+16 * a_tmp_tmp * l_a_tmp * (n_a_tmp + 0.3) / 3125.0) /
               m_a_tmp) -
            c_J_tmp * jb_a_tmp_tmp / d_J_tmp) -
           4.0532396646334464E+23 * J_tmp * kb_a_tmp_tmp / lb_a_tmp_tmp) +
          yb_a_tmp_tmp * e_J_tmp / lb_a_tmp_tmp) -
         yb_a_tmp_tmp * c_J_tmp * kb_a_tmp_tmp / f_J_tmp) +
        g_J_tmp / ac_a_tmp_tmp) +
       i_J_tmp) +
      ar_tmp_tmp / j_J_tmp) /
       t_a_tmp -
     u_a_tmp *
       ((((((((((((((((((((((4.7837208501806621E+21 * sf1 / 78125.0 - 3.7453335300841144E+18 * cf1 / 25.0) - re) -
                           8.3419396212098209E+21 * c1s2 / 390625.0) -
                          1.2979856047270696E+17 * c2s1 / 2.0) -
                         nb_a_tmp_tmp * J_tmp) +
                        3.97508091447665E+17 * s1s2 / 5.0) +
                       7.12483534798848E+14 * l_a_tmp * c_a_tmp_tmp / 1.953125E+6) +
                      ob_a_tmp_tmp * J_tmp / 3125.0) +
                     pb_a_tmp_tmp * J_tmp / 3125.0) +
                    2.4319437987800678E+17 * b_J_tmp * (n_a_tmp + 0.3)) +
                   7.466827444691927E+17 * a_tmp_tmp * l_a_tmp / 1.953125E+6) -
                  7.466827444691927E+17 * l_a_tmp * (a_tmp_tmp + 3000.0) / 1.953125E+6) +
                 1.3619493259118075E+21 * c1c2 / 78125.0) -
                7.599824371187712E+15 * a_tmp_tmp * l_a_tmp * (n_a_tmp + 0.3) / 3125.0) /
                 m_a_tmp -
               (((((66101.0 * cf1 / 1.5E+6 + 217189.0 * sf1 / 3.75E+6) + 462707.0 * c1s2 / 1.25E+7) -
                  497.0 * a_tmp_tmp * l_a_tmp / 7.5E+8) +
                 497.0 * l_a_tmp * (a_tmp_tmp + 3000.0) / 7.5E+8) -
                9443.0 * c1c2 / 312500.0) /
                 k_a_tmp) -
              c_J_tmp * qb_a_tmp_tmp / d_J_tmp) +
             3.730481691338561E+24 * J_tmp * kb_a_tmp_tmp / lb_a_tmp_tmp) -
            rb_a_tmp_tmp * e_J_tmp / lb_a_tmp_tmp) +
           rb_a_tmp_tmp * c_J_tmp * kb_a_tmp_tmp / f_J_tmp) +
          g_J_tmp / sb_a_tmp_tmp) +
         tb_a_tmp_tmp) +
        ar_tmp_tmp / k_J_tmp) /
       h_J_tmp) *
    h_J_tmp / l_J_tmp;
  ar_tmp = 147.0 * c2s1 + 120.0 * s1s2;
  b_ar_tmp = 120.0 * c1s2 + 147.0 * c1c2;
  t_a_tmp_tmp = 40.0 * cf2 - 49.0 * sf2;
  j_a_tmp_tmp = 2.305843009213694E+18 * b_ar_tmp;
  i_a_tmp_tmp_tmp = 2.305843009213694E+18 * ar_tmp;
  i_a_tmp_tmp = 2.0752587082923246E+19 * t_a_tmp_tmp * k_a_tmp;
  ar_tmp_tmp =
    ((((((((((((((((((((((((((6.3715925454782912E+28 * c2s1 - 1.8038207131762135E+29 * c1s2) +
                             2.6286610305036111E+21 * ar_tmp * o_a_tmp) -
                            i_a_tmp_tmp_tmp * m_a_tmp_tmp) +
                           5.2013000371251354E+28 * s1s2) +
                          1.8677328374630921E+20 * t_a_tmp_tmp * safe_pow(k_a_tmp, 3.0)) +
                         1.235254857431035E+25 * b_ar_tmp * (a_tmp_tmp + 3000.0)) +
                        b_ar_tmp * l_a_tmp_tmp) -
                       (4.0496100371526431E+26 * c2s1 + 3.3058041119613413E+26 * s1s2) * (a_tmp_tmp + 3000.0)) -
                      1.8065634341146726E+24 * ar_tmp * l_a_tmp) -
                     1.0874355631451781E+22 * b_ar_tmp * c_a_tmp_tmp) +
                    j_a_tmp_tmp * p_a_tmp) -
                   3.2623066894355342E+22 * b_ar_tmp * fb_a_tmp_tmp) +
                  8.6060148529399511E+24 * t_a_tmp_tmp * k_a_tmp) -
                 3.6247852104839269E+21 * b_ar_tmp * o_a_tmp) +
                (1.2880439049467694E+23 * c2s1 + 1.0514644122014444E+23 * s1s2) * c_a_tmp_tmp) -
               2.2096803736408614E+29 * c1c2) +
              7.8859830915108333E+21 * ar_tmp * fb_a_tmp_tmp) +
             i_a_tmp_tmp * o_a_tmp) -
            2.0 * b_ar_tmp * ib_a_tmp_tmp * (a_tmp_tmp + 3000.0)) +
           7.2495704209678538E+21 * ar_tmp * l_a_tmp * (a_tmp_tmp + 3000.0)) +
          2.0752587082923246E+19 * b_ar_tmp * fb_a_tmp_tmp * (a_tmp_tmp + 3000.0)) -
         6.5246133788710684E+22 * t_a_tmp_tmp * k_a_tmp * (a_tmp_tmp + 3000.0)) +
        j_a_tmp_tmp * o_a_tmp * (a_tmp_tmp + 3000.0)) -
       1.5771966183021667E+22 * t_a_tmp_tmp * k_a_tmp * l_a_tmp) -
      i_a_tmp_tmp_tmp * l_a_tmp * c_a_tmp_tmp) -
     2.0752587082923246E+19 * ar_tmp * fb_a_tmp_tmp * l_a_tmp) +
    i_a_tmp_tmp * c_a_tmp_tmp;
  j_a_tmp_tmp = a_tmp_tmp_tmp * ar_tmp_tmp;
  i_a_tmp_tmp_tmp = j_a_tmp_tmp * -566.23104;
  i_a_tmp_tmp = j_a_tmp_tmp * -5211.42272;
  if (i_a_tmp_tmp == 0.0) {
    im = 0.0;
    re = i_a_tmp_tmp_tmp / q_a_tmp;
  } else {
    im = i_a_tmp_tmp / q_a_tmp;
    if (i_a_tmp_tmp_tmp == 0.0) {
      re = 0.0;
    } else {
      re = i_a_tmp_tmp_tmp / q_a_tmp;
    }
  }
  J_tmp = q_a_tmp_tmp + h_a_tmp_tmp_tmp;
  b_J_tmp = 2793.0 * c2s1 / 5000.0 + 57.0 * s1s2 / 125.0;
  c_J_tmp = r_a_tmp_tmp - s_a_tmp_tmp;
  e_J_tmp = ((4.2477951685358518E+20 * c1s2 - 2.251799813685248E+15 * b_ar_tmp * (a_tmp_tmp + 3000.0)) -
             2.0266198323167232E+16 * t_a_tmp_tmp * k_a_tmp) +
            5.2035490814564185E+20 * c1c2;
  g_J_tmp = 1.4142135623730951 * c_J_tmp * q_a_tmp;
  ar_tmp_tmp = ((((((((((((((4.807354091581739E+15 * c1s2 / 9.007199254740992E+18 + 1.097649E+6 * c2s1 / 6.25E+9) +
                            22401.0 * s1s2 / 1.5625E+8) +
                           393.0 * b_ar_tmp * (a_tmp_tmp + 3000.0) / 1.25E+11) +
                          J_tmp * s_a_tmp / 500.0) -
                         b_J_tmp * (n_a_tmp + 0.3) / 500.0) -
                        393.0 * ar_tmp * l_a_tmp / 1.25E+11) -
                       3.0 * b_ar_tmp * c_a_tmp_tmp / 1.0E+12) -
                      9.0 * J_tmp * fb_a_tmp_tmp / 1.0E+8) -
                     3537.0 * t_a_tmp_tmp * k_a_tmp / 1.25E+11) -
                    J_tmp * o_a_tmp / 1.0E+8) +
                   3.680630476367269E+15 * c1c2 / 5.62949953421312E+18) +
                  c_a_tmp_tmp_tmp * c_J_tmp / 1.073741824E+17) +
                 ar_tmp * l_a_tmp * (n_a_tmp + 0.3) / 5.0E+7) -
                9.0 * t_a_tmp_tmp * k_a_tmp * (n_a_tmp + 0.3) / 5.0E+7) -
               1.4142135623730951 * (ar_tmp_tmp / q_a_tmp) * r_a_tmp / 2.147483648E+17;
  m_J_tmp = fb_a_tmp_tmp * m_a_tmp;
  i_a_tmp_tmp_tmp = 0.0 * J_tmp * r_a_tmp;
  c_J_tmp = 0.0 * c_J_tmp * (n_a_tmp + 0.1428);
  e_a_tmp_tmp_tmp = j_a_tmp_tmp_tmp * e_J_tmp * (n_a_tmp + 0.1428);
  j_a_tmp_tmp = 0.0 * t_a_tmp_tmp * r_a_tmp * (n_a_tmp + 0.1428);
  J_temp[3] = -(((((((((((((((((((((((((((((g_J_tmp * 10422.84544 + im) + 2.3892549835161242E+18 * c1s2 / 4.0) +
                                          7.67767405878015E+22 * c2s1 / 390625.0) +
                                         1.2534978055151265E+22 * s1s2 / 78125.0) +
                                        6.8722467407627551E+18 * b_ar_tmp * (a_tmp_tmp + 3000.0) / 1.953125E+6) +
                                       2.2382890148031365E+18 * J_tmp * s_a_tmp) -
                                      2.2382890148031365E+18 * b_J_tmp * (n_a_tmp + 0.3)) -
                                     6.8722467407627551E+18 * ar_tmp * l_a_tmp / 1.953125E+6) -
                                    6.557487348056064E+15 * b_ar_tmp * c_a_tmp_tmp / 1.953125E+6) -
                                   3.1475939270669107E+17 * J_tmp * fb_a_tmp_tmp / 3125.0) -
                                  6.1850220666864796E+19 * t_a_tmp_tmp * k_a_tmp / 1.953125E+6) -
                                 3.4973265856299008E+16 * J_tmp * o_a_tmp / 3125.0) +
                                3.6585466935090652E+18 * c1c2 / 5.0) +
                               6.9946531712598016E+16 * ar_tmp * l_a_tmp * (n_a_tmp + 0.3) / 3125.0) -
                              6.2951878541338214E+17 * t_a_tmp_tmp * k_a_tmp * (n_a_tmp + 0.3) / 3125.0) /
                               m_a_tmp -
                             ((((25137.0 * c2s1 / 6.25E+6 + 513.0 * s1s2 / 156250.0) +
                                9.0 * b_ar_tmp * (a_tmp_tmp + 3000.0) / 1.25E+8) -
                               9.0 * ar_tmp * l_a_tmp / 1.25E+8) +
                              81.0 * t_a_tmp_tmp * k_a_tmp / 1.25E+8) /
                               k_a_tmp) -
                            e_J_tmp * jb_a_tmp_tmp / d_J_tmp) +
                           t_a_tmp_tmp * bc_a_tmp_tmp / fb_a_tmp_tmp) +
                          4.0532396646334464E+23 * J_tmp * kb_a_tmp_tmp / lb_a_tmp_tmp) +
                         yb_a_tmp_tmp * ar_tmp_tmp / lb_a_tmp_tmp) -
                        4.0532396646334464E+23 * e_J_tmp * (n_a_tmp + 0.1428) * kb_a_tmp_tmp / f_J_tmp) -
                       4.0532396646334464E+23 * t_a_tmp_tmp * (n_a_tmp + 0.1428) * kb_a_tmp_tmp / m_J_tmp) -
                      i_a_tmp_tmp_tmp / ac_a_tmp_tmp) +
                     i_J_tmp) -
                    c_J_tmp / ac_a_tmp_tmp) +
                   e_a_tmp_tmp_tmp / j_J_tmp) +
                  j_a_tmp_tmp / (2.0E+8 * fb_a_tmp_tmp * m_a_tmp)) /
                   t_a_tmp -
                 u_a_tmp *
                   (((((((((((((((((((((((((((g_J_tmp * 1132.46208 + re) + 1.2979856047270696E+17 * c1s2 / 2.0) +
                                            8.3419396212098209E+21 * c2s1 / 390625.0) +
                                           1.3619493259118075E+21 * s1s2 / 78125.0) +
                                          7.466827444691927E+17 * b_ar_tmp * (a_tmp_tmp + 3000.0) / 1.953125E+6) +
                                         2.4319437987800678E+17 * J_tmp * s_a_tmp) -
                                        2.4319437987800678E+17 * b_J_tmp * (n_a_tmp + 0.3)) -
                                       7.466827444691927E+17 * ar_tmp * l_a_tmp / 1.953125E+6) -
                                      7.12483534798848E+14 * b_ar_tmp * c_a_tmp_tmp / 1.953125E+6) -
                                     3.4199209670344704E+16 * J_tmp * fb_a_tmp_tmp / 3125.0) -
                                    6.7201447002227343E+18 * t_a_tmp_tmp * k_a_tmp / 1.953125E+6) -
                                   3.799912185593856E+15 * J_tmp * o_a_tmp / 3125.0) +
                                  3.97508091447665E+17 * c1c2 / 5.0) +
                                 7.599824371187712E+15 * ar_tmp * l_a_tmp * (n_a_tmp + 0.3) / 3125.0) -
                                6.8398419340689408E+16 * t_a_tmp_tmp * k_a_tmp * (n_a_tmp + 0.3) / 3125.0) /
                                 m_a_tmp +
                               ((((462707.0 * c2s1 / 1.25E+7 + 9443.0 * s1s2 / 312500.0) +
                                  497.0 * b_ar_tmp * (a_tmp_tmp + 3000.0) / 7.5E+8) -
                                 497.0 * ar_tmp * l_a_tmp / 7.5E+8) +
                                1491.0 * t_a_tmp_tmp * k_a_tmp / 2.5E+8) /
                                 k_a_tmp) -
                              t_a_tmp_tmp * ub_a_tmp_tmp / fb_a_tmp_tmp) -
                             e_J_tmp * qb_a_tmp_tmp / d_J_tmp) -
                            3.730481691338561E+24 * J_tmp * kb_a_tmp_tmp / lb_a_tmp_tmp) -
                           rb_a_tmp_tmp * ar_tmp_tmp / lb_a_tmp_tmp) +
                          3.730481691338561E+24 * e_J_tmp * (n_a_tmp + 0.1428) * kb_a_tmp_tmp / f_J_tmp) +
                         3.730481691338561E+24 * t_a_tmp_tmp * (n_a_tmp + 0.1428) * kb_a_tmp_tmp / m_J_tmp) -
                        i_a_tmp_tmp_tmp / sb_a_tmp_tmp) +
                       tb_a_tmp_tmp) -
                      c_J_tmp / sb_a_tmp_tmp) +
                     e_a_tmp_tmp_tmp / k_J_tmp) +
                    j_a_tmp_tmp / (5.0 * fb_a_tmp_tmp * m_a_tmp)) /
                   h_J_tmp) *
                h_J_tmp) /
              l_J_tmp;

  J(0, 0) = J_temp[0];
  J(0, 1) = J_temp[2];
  J(1, 0) = J_temp[1];
  J(1, 1) = J_temp[3];
}

// ── 底层单腿转换函数 ─────────────────────────────────────────────────
// 关节位置转换（H2L）
// 输入：hpos - 高层关节位置向量（pitch, roll）
// 输出：lpos - 低层关节位置向量（pitch, roll）
// 输入：is_mirror - 是否镜像
void AnklePosH2L(
  const Eigen::Vector<double, kLegJointNum> & hpos,
  Eigen::Vector<double, kLegJointNum> & lpos,
  bool is_mirror)
{
  lpos = hpos;
  Eigen::Vector2d ank = hpos.segment<2>(kAnklePitchIdx);
  if (is_mirror) {
    ank(1) = -ank(1);
  }
  Eigen::Vector2d ank_ans;
  FastAnkleFk::CheckInput(ank);
  FastAnkleFk::Calculate(ank, ank_ans);
  lpos(kAnklePitchIdx) = ank_ans(0);
  lpos(kAnkleRollIdx) = ank_ans(1);
  if (is_mirror) {
    lpos(kAnklePitchIdx) = -ank_ans(0);
    lpos(kAnkleRollIdx) = -ank_ans(1);
  }
}

// 关节位置转换（L2H）
// 输入：hpos - 高层关节位置向量（pitch, roll）
// 输出：lpos - 低层关节位置向量（pitch, roll）
// 输入：is_mirror - 是否镜像
void AnklePosL2H(
  Eigen::Vector<double, kLegJointNum> & hpos,
  const Eigen::Vector<double, kLegJointNum> & lpos,
  bool is_mirror)
{
  hpos = lpos;
  Eigen::Vector2d ank_ans = {lpos(kAnklePitchIdx), lpos(kAnkleRollIdx)};
  if (is_mirror) {
    ank_ans(0) = -ank_ans(0);
    ank_ans(1) = -ank_ans(1);
  }
  Eigen::Vector2d ank;
  FastAnkleIk::CheckInput(ank_ans);
  FastAnkleIk::Calculate(ank_ans, ank);
  if (is_mirror) {
    ank(1) = -ank(1);
  }
  hpos.segment<2>(kAnklePitchIdx) = ank;
}

// 关节位置转换（H2L）
// 输入：hpos - 高层关节位置向量（pitch, roll）
// 输出：lpos - 低层关节位置向量（pitch, roll）
// 输入：hvel - 高层关节速度向量（pitch, roll）
// 输出：lvel - 低层关节速度向量（pitch, roll）
// 输入：htor - 高层关节力矩向量（pitch, roll）
// 输出：ltor - 低层关节力矩向量（pitch, roll）
// 输入：is_mirror - 是否镜像
void AnkleAllH2L(
  const Eigen::Vector<double, kLegJointNum> & hpos,
  Eigen::Vector<double, kLegJointNum> & lpos,
  const Eigen::Vector<double, kLegJointNum> & hvel,
  Eigen::Vector<double, kLegJointNum> & lvel,
  const Eigen::Vector<double, kLegJointNum> & htor,
  Eigen::Vector<double, kLegJointNum> & ltor,
  const Eigen::Vector<double, kLegJointNum> & hkp,
  Eigen::Vector<double, kLegJointNum> & lkp,
  const Eigen::Vector<double, kLegJointNum> & hkd,
  Eigen::Vector<double, kLegJointNum> & lkd,
  bool is_mirror)
{
  AnklePosH2L(hpos, lpos, is_mirror);

  Eigen::Matrix2d J;
  Eigen::Vector2d ank = hpos.segment<2>(kAnklePitchIdx);
  if (is_mirror) {
    ank(1) = -ank(1);
  }
  FastAnkleFk::CheckInput(ank);
  FastAnkleJacob::Calculate(ank, J);

  // 速度变换
  lvel = hvel;
  Eigen::Vector2d ank_v = hvel.segment<2>(kAnklePitchIdx);
  if (is_mirror) {
    ank_v(1) = -ank_v(1);
  }
  Eigen::Vector2d ank_v_ans = J * ank_v;
  lvel(kAnklePitchIdx) = ank_v_ans(0);
  lvel(kAnkleRollIdx) = ank_v_ans(1);
  if (is_mirror) {
    lvel(kAnklePitchIdx) = -ank_v_ans(0);
    lvel(kAnkleRollIdx) = -ank_v_ans(1);
  }

  // 力矩变换
  ltor = htor;
  Eigen::Vector2d ank_t = htor.segment<2>(kAnklePitchIdx);
  if (is_mirror) {
    ank_t(1) = -ank_t(1);
  }
  Eigen::Matrix2d J_inv_T = PseudoInverse(J.transpose());
  Eigen::Vector2d ank_t_ans = J_inv_T * ank_t;
  ltor(kAnklePitchIdx) = ank_t_ans(0);
  ltor(kAnkleRollIdx) = ank_t_ans(1);
  if (is_mirror) {
    ltor(kAnklePitchIdx) = -ank_t_ans(0);
    ltor(kAnkleRollIdx) = -ank_t_ans(1);
  }

  // ── kp/kd 变换 ────────────────────────────────────────────────────
  //
  // kp_L = diag(J⁻ᵀ · diag(kp_H) · J⁻¹)
  //
  // 推导：
  //   速度：v_L = J · v_H                      (1)
  //   力矩：τ_L = J⁻ᵀ · τ_H                   (2) 功率守恒
  //   刚度定义：τ = kp · Δq  →  kp = ∂τ/∂q
  //   链式法则：kp_L = ∂τ_L/∂q_L = J⁻ᵀ · (∂τ_H/∂q_H) · (∂q_H/∂q_L) = J⁻ᵀ · kp_H · J⁻¹
  //   阻尼同理（力/速度关系与刚度力/位移一致）。
  //
  // 对角近似与简化：
  //   完整 kp_L 是 2×2 矩阵。对角近似丢弃非对角耦合项，仅保留对角元：
  //     kp_L(pitch) = (k₁·j₂₂² + k₂·j₂₁²) / det(J)²
  //     kp_L(roll)  = (k₁·j₁₂² + k₂·j₁₁²) / det(J)²
  //   非对角元为零的条件：kp₂/kp₁ = -(j₂₂·j₁₂)/(j₂₁·j₁₁) ≜ r_opt。
  //   将 hip 帧 kp_roll 锁定在 r_opt，仅以 kp_pitch 为输入：
  //     kp_L(pitch) = (c₀₀ + r_opt·c₀₁) · hip_kp_pitch
  //     kp_L(roll)  = (c₁₀ + r_opt·c₁₁) · hip_kp_pitch
  //   kd 同理。
  //
  // 注意：kp/kd 是无符号物理量（刚度/阻尼恒正），与 pos/vel/tor 不同，
  // 不需要镜像符号翻转。

  constexpr bool kUseFixedGainCoeffs = true;

  // 固定系数：由站立姿态 (ankle_pitch=-0.2, ankle_roll=0.0) 的 J 预计算。
  //   J ≈ [-1.198, 0.700; 1.201, 0.694], det ≈ -1.673
  constexpr double kFixedC00 = 0.1723, kFixedC01 = 0.5159;
  constexpr double kFixedC10 = 0.1753, kFixedC11 = 0.5133;
  constexpr double kFixedOptRatio = 0.3376;  // 站立姿态: -(j₂₂·j₁₂)/(j₂₁·j₁₁) ≈ 0.3376

  double c00 = kFixedC00, c01 = kFixedC01, c10 = kFixedC10, c11 = kFixedC11;
  double opt_ratio = kFixedOptRatio;

  if constexpr (!kUseFixedGainCoeffs) {
    // 动态解算：根据当前姿态的 J 实时计算系数及最优对角比
    double j11 = J(0, 0), j12 = J(0, 1);
    double j21 = J(1, 0), j22 = J(1, 1);
    double det = j11 * j22 - j12 * j21;
    if (std::abs(det) > 1e-6) {
      double inv_det_sq = 1.0 / (det * det);
      c00 = j22 * j22 * inv_det_sq;
      c01 = j21 * j21 * inv_det_sq;
      c10 = j12 * j12 * inv_det_sq;
      c11 = j11 * j11 * inv_det_sq;
      opt_ratio = -(j22 * j12) / (j21 * j11);  // 使非对角元为零的最优比
    }
    // 退化时保持固定系数默认值
  }

  auto transformAnkleGain = [&](const Eigen::Vector<double, kLegJointNum> & src,
                                 Eigen::Vector<double, kLegJointNum> & dst) {
    dst = src;
    double kp_pitch = src(kAnklePitchIdx);
    dst(kAnklePitchIdx) = (c00 + opt_ratio * c01) * kp_pitch;
    dst(kAnkleRollIdx)  = (c10 + opt_ratio * c11) * kp_pitch;
    // kp/kd 是无符号量，不随镜像取反
  };

  transformAnkleGain(hkp, lkp);
  transformAnkleGain(hkd, lkd);
}

// 关节全量转换（H2L）：pos, vel, tor（无 kp/kd，用于状态反馈路径）
void AnkleAllH2L(
  const Eigen::Vector<double, kLegJointNum> & hpos,
  Eigen::Vector<double, kLegJointNum> & lpos,
  const Eigen::Vector<double, kLegJointNum> & hvel,
  Eigen::Vector<double, kLegJointNum> & lvel,
  const Eigen::Vector<double, kLegJointNum> & htor,
  Eigen::Vector<double, kLegJointNum> & ltor,
  bool is_mirror)
{
  AnklePosH2L(hpos, lpos, is_mirror);

  Eigen::Matrix2d J;
  Eigen::Vector2d ank = hpos.segment<2>(kAnklePitchIdx);
  if (is_mirror) {
    ank(1) = -ank(1);
  }
  FastAnkleFk::CheckInput(ank);
  FastAnkleJacob::Calculate(ank, J);

  // 速度变换
  lvel = hvel;
  Eigen::Vector2d ank_v = hvel.segment<2>(kAnklePitchIdx);
  if (is_mirror) {
    ank_v(1) = -ank_v(1);
  }
  Eigen::Vector2d ank_v_ans = J * ank_v;
  lvel(kAnklePitchIdx) = ank_v_ans(0);
  lvel(kAnkleRollIdx) = ank_v_ans(1);
  if (is_mirror) {
    lvel(kAnklePitchIdx) = -ank_v_ans(0);
    lvel(kAnkleRollIdx) = -ank_v_ans(1);
  }

  // 力矩变换
  ltor = htor;
  Eigen::Vector2d ank_t = htor.segment<2>(kAnklePitchIdx);
  if (is_mirror) {
    ank_t(1) = -ank_t(1);
  }
  Eigen::Matrix2d J_inv_T = PseudoInverse(J.transpose());
  Eigen::Vector2d ank_t_ans = J_inv_T * ank_t;
  ltor(kAnklePitchIdx) = ank_t_ans(0);
  ltor(kAnkleRollIdx) = ank_t_ans(1);
  if (is_mirror) {
    ltor(kAnklePitchIdx) = -ank_t_ans(0);
    ltor(kAnkleRollIdx) = -ank_t_ans(1);
  }
}

// 关节位置转换（L2H）
// 输入：hpos - 高层关节位置向量（pitch, roll）
// 输出：lpos - 低层关节位置向量（pitch, roll）
// 输入：hvel - 高层关节速度向量（pitch, roll）
// 输出：hvel - 高层关节速度向量（pitch, roll）
// 输入：htor - 高层关节力矩向量（pitch, roll）
// 输出：htor - 高层关节力矩向量（pitch, roll）
// 输入：is_mirror - 是否镜像
void AnkleAllL2H(
  Eigen::Vector<double, kLegJointNum> & hpos,
  const Eigen::Vector<double, kLegJointNum> & lpos,
  Eigen::Vector<double, kLegJointNum> & hvel,
  const Eigen::Vector<double, kLegJointNum> & lvel,
  Eigen::Vector<double, kLegJointNum> & htor,
  const Eigen::Vector<double, kLegJointNum> & ltor,
  bool is_mirror)
{
  AnklePosL2H(hpos, lpos, is_mirror);

  Eigen::Matrix2d J;
  Eigen::Vector2d ank = hpos.segment<2>(kAnklePitchIdx);
  if (is_mirror) {
    ank(1) = -ank(1);
  }
  FastAnkleFk::CheckInput(ank);
  FastAnkleJacob::Calculate(ank, J);

  // 速度变换
  hvel = lvel;
  Eigen::Vector2d ank_v_ans = {lvel(kAnklePitchIdx), lvel(kAnkleRollIdx)};
  if (is_mirror) {
    ank_v_ans(0) = -ank_v_ans(0);
    ank_v_ans(1) = -ank_v_ans(1);
  }
  Eigen::Vector2d ank_v = PseudoInverse(J) * ank_v_ans;
  if (is_mirror) {
    ank_v(1) = -ank_v(1);
  }
  hvel.segment<2>(kAnklePitchIdx) = ank_v;

  // 力矩变换
  htor = ltor;
  Eigen::Vector2d ank_t_ans = {ltor(kAnklePitchIdx), ltor(kAnkleRollIdx)};
  if (is_mirror) {
    ank_t_ans(0) = -ank_t_ans(0);
    ank_t_ans(1) = -ank_t_ans(1);
  }
  Eigen::Vector2d ank_t = J.transpose() * ank_t_ans;
  if (is_mirror) {
    ank_t(1) = -ank_t(1);
  }
  htor.segment<2>(kAnklePitchIdx) = ank_t;
}

// 伪逆矩阵（用于逆运动学）
// 输入：mat - 矩阵
// 输出：伪逆矩阵
Eigen::MatrixXd PseudoInverse(const Eigen::MatrixXd & mat)
{
  typedef typename Eigen::MatrixXd::Scalar Scalar;
  auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto & singular_val = svd.singularValues();
  Eigen::MatrixXd singular_val_inv;
  double max_sg_val{0};
  for (int i = 0; i < singular_val.size(); ++i) {
    if (singular_val(i) > max_sg_val) max_sg_val = singular_val(i);
  }
  double tolerance = std::numeric_limits<double>::epsilon() * std::max(mat.rows(), mat.cols()) * max_sg_val;
  singular_val_inv = Eigen::MatrixXd::Zero(mat.cols(), mat.rows());
  for (int i = 0; i < singular_val.size(); ++i) {
    if (singular_val(i) > tolerance) singular_val_inv(i, i) = Scalar{1} / singular_val(i);
    else singular_val_inv(i, i) = Scalar{0};
  }
  return svd.matrixV() * singular_val_inv * svd.matrixU().adjoint();
}

// ── 力矩前馈模式（H2L） ──────────────────────────────────────────────
// 在串联空间算 PD 力矩，再映射成并联电机力矩。
//
// 镜像约定与 AnkleAllH2L 保持一致：右腿内部统一用 (pitch, -roll) 计算，
// 角/角速度输入镜像 roll，力矩输出镜像两项。
void AnkleTorqueH2L(
  const Eigen::Vector<double, kLegJointNum> & hpos_cmd,
  const Eigen::Vector<double, kLegJointNum> & hvel_cmd,
  const Eigen::Vector<double, kLegJointNum> & hkp,
  const Eigen::Vector<double, kLegJointNum> & hkd,
  const Eigen::Vector<double, kLegJointNum> & htor_ff,
  const Eigen::Vector<double, kLegJointNum> & hpos_meas,
  const Eigen::Vector<double, kLegJointNum> & hvel_meas,
  Eigen::Vector2d & ltor,
  bool is_mirror)
{
  // 进入内部计算坐标系（右腿 roll 取反）。
  auto toInternal = [is_mirror](Eigen::Vector2d v) {
      if (is_mirror) v(1) = -v(1);
      return v;
    };

  Eigen::Vector2d q_des = toInternal(hpos_cmd.segment<2>(kAnklePitchIdx));
  Eigen::Vector2d dq_des = toInternal(hvel_cmd.segment<2>(kAnklePitchIdx));
  Eigen::Vector2d q_meas = toInternal(hpos_meas.segment<2>(kAnklePitchIdx));
  Eigen::Vector2d dq_meas = toInternal(hvel_meas.segment<2>(kAnklePitchIdx));
  Eigen::Vector2d tau_ff = toInternal(htor_ff.segment<2>(kAnklePitchIdx));
  const Eigen::Vector2d kp(hkp(kAnklePitchIdx), hkp(kAnkleRollIdx));
  const Eigen::Vector2d kd(hkd(kAnklePitchIdx), hkd(kAnkleRollIdx));

  // 目标角先按串联限位钳位；实测角同样钳位，保证 J 在工作空间内求值。
  FastAnkleFk::CheckInput(q_des);
  FastAnkleFk::CheckInput(q_meas);

  // J 取实测姿态：τ_L = J⁻ᵀ·τ_H 是当前构型下的瞬时映射。
  Eigen::Matrix2d J;
  FastAnkleJacob::Calculate(q_meas, J);

  // 串联空间 PD（模型自带增益，无任何等效近似）。
  const Eigen::Vector2d tau_h = tau_ff +
    kp.cwiseProduct(q_des - q_meas) + kd.cwiseProduct(dq_des - dq_meas);

  // 映射到并联电机力矩。
  Eigen::Vector2d tau_l = PseudoInverse(J.transpose()) * tau_h;
  if (is_mirror) {
    tau_l(0) = -tau_l(0);
    tau_l(1) = -tau_l(1);
  }
  ltor = tau_l;
}

// ── 高层向量 API ────────────────────────────────────────────────────
// 串联→并联（S2P）
// 输入：pos - 关节位置向量（pitch, roll）
// 输入：vel - 关节速度向量（pitch, roll）
// 输入：tor - 关节力矩向量（pitch, roll）
// 输入：is_mirror - 是否镜像
// 输出：pos - 关节位置向量（pitch, roll）
// 输出：vel - 关节速度向量（pitch, roll）
// 输出：tor - 关节力矩向量（pitch, roll）
void seriesToParallel(
  Eigen::Vector<double, kLegJointNum> & pos,
  Eigen::Vector<double, kLegJointNum> & vel,
  Eigen::Vector<double, kLegJointNum> & tor,
  Eigen::Vector<double, kLegJointNum> & kp,
  Eigen::Vector<double, kLegJointNum> & kd,
  bool is_mirror)
{
  Eigen::Vector<double, kLegJointNum> lpos, lvel, ltor, lkp, lkd;
  AnkleAllH2L(pos, lpos, vel, lvel, tor, ltor, kp, lkp, kd, lkd, is_mirror);
  pos = lpos;
  vel = lvel;
  tor = ltor;
  kp = lkp;
  kd = lkd;
}

// 串联→并联（S2P）：pos, vel, tor（无 kp/kd，用于状态反馈路径）
void seriesToParallel(
  Eigen::Vector<double, kLegJointNum> & pos,
  Eigen::Vector<double, kLegJointNum> & vel,
  Eigen::Vector<double, kLegJointNum> & tor,
  bool is_mirror)
{
  Eigen::Vector<double, kLegJointNum> lpos, lvel, ltor;
  AnkleAllH2L(pos, lpos, vel, lvel, tor, ltor, is_mirror);
  pos = lpos;
  vel = lvel;
  tor = ltor;
}

// 并联→串联（P2S）
// 输入：pos - 关节位置向量（pitch, roll）
// 输入：vel - 关节速度向量（pitch, roll）
// 输入：tor - 关节力矩向量（pitch, roll）
// 输入：is_mirror - 是否镜像
// 输出：pos - 关节位置向量（pitch, roll）
// 输出：vel - 关节速度向量（pitch, roll）
// 输出：tor - 关节力矩向量（pitch, roll）
void parallelToSeries(
  Eigen::Vector<double, kLegJointNum> & pos,
  Eigen::Vector<double, kLegJointNum> & vel,
  Eigen::Vector<double, kLegJointNum> & tor,
  bool is_mirror)
{
  Eigen::Vector<double, kLegJointNum> hpos, hvel, htor;
  AnkleAllL2H(hpos, pos, hvel, vel, htor, tor, is_mirror);
  pos = hpos;
  vel = hvel;
  tor = htor;
}

// ── 命名关节辅助函数 ─────────────────────────────────────────────────
// 提取关节位置
// 输入：leg_joint_names - 关节名称数组
// 输入：positions - 关节位置数组
// 输入：joint_names - 关节名称数组
// 输出：out - 关节位置向量（pitch, roll）
// 返回：是否成功提取
bool extractLegJoints(
  const char * const * leg_joint_names,
  const std::vector<float> & positions,
  const std::vector<std::string> & joint_names,
  Eigen::Vector<double, kLegJointNum> & out)
{
  for (int i = 0; i < kLegJointNum; ++i) {
    bool found = false;
    for (std::size_t j = 0; j < joint_names.size(); ++j) {
      if (joint_names[j] == leg_joint_names[i]) {
        out[i] = static_cast<double>(positions[j]);
        found = true;
        break;
      }
    }
    if (!found) return false;
  }
  return true;
}

// 插入关节位置
// 输入：leg_joint_names - 关节名称数组
// 输入：values - 关节位置向量（pitch, roll）
// 输入：positions - 关节位置数组
// 输入：joint_names - 关节名称数组
void insertLegJoints(
  const char * const * leg_joint_names,
  const Eigen::Vector<double, kLegJointNum> & values,
  std::vector<float> & positions,
  const std::vector<std::string> & joint_names)
{
  for (int i = 0; i < kLegJointNum; ++i) {
    for (std::size_t j = 0; j < joint_names.size(); ++j) {
      if (joint_names[j] == leg_joint_names[i]) {
        positions[j] = static_cast<float>(values[i]);
        break;
      }
    }
  }
}

// ── 双腿平面数组辅助函数 ─────────────────────────────────────────────
// 串联→并联（S2P）
// 输入：positions - 关节位置数组
// 输入：velocities - 关节速度数组
// 输入：efforts - 关节力矩数组
// 输入：names - 关节名称数组
void seriesToParallelBothLegs(
  std::vector<float> & positions,
  std::vector<float> & velocities,
  std::vector<float> & efforts,
  std::vector<float> & kps,
  std::vector<float> & kds,
  const std::vector<std::string> & names)
{
  auto convert_one_leg = [&](const char * const * leg_names, bool mirror) {
    Eigen::Vector<double, kLegJointNum> pos, vel, tor, kp, kd;
    if (extractLegJoints(leg_names, positions, names, pos) &&
        extractLegJoints(leg_names, velocities, names, vel) &&
        extractLegJoints(leg_names, efforts, names, tor) &&
        extractLegJoints(leg_names, kps, names, kp) &&
        extractLegJoints(leg_names, kds, names, kd))
    {
      seriesToParallel(pos, vel, tor, kp, kd, mirror);
      insertLegJoints(leg_names, pos, positions, names);
      insertLegJoints(leg_names, vel, velocities, names);
      insertLegJoints(leg_names, tor, efforts, names);
      insertLegJoints(leg_names, kp, kps, names);
      insertLegJoints(leg_names, kd, kds, names);
    }
  };
  convert_one_leg(kLeftLegJointNames, false);
  convert_one_leg(kRightLegJointNames, true);
}

// 串联→并联（S2P）：pos, vel, tor（无 kp/kd，用于状态反馈路径）
void seriesToParallelBothLegs(
  std::vector<float> & positions,
  std::vector<float> & velocities,
  std::vector<float> & efforts,
  const std::vector<std::string> & names)
{
  auto convert_one_leg = [&](const char * const * leg_names, bool mirror) {
    Eigen::Vector<double, kLegJointNum> pos, vel, tor;
    if (extractLegJoints(leg_names, positions, names, pos) &&
        extractLegJoints(leg_names, velocities, names, vel) &&
        extractLegJoints(leg_names, efforts, names, tor))
    {
      seriesToParallel(pos, vel, tor, mirror);
      insertLegJoints(leg_names, pos, positions, names);
      insertLegJoints(leg_names, vel, velocities, names);
      insertLegJoints(leg_names, tor, efforts, names);
    }
  };
  convert_one_leg(kLeftLegJointNames, false);
  convert_one_leg(kRightLegJointNames, true);
}

// 并联→串联（P2S）
// 输入：positions - 关节位置数组
// 输入：velocities - 关节速度数组
// 输入：efforts - 关节力矩数组
// 输入：names - 关节名称数组
void parallelToSeriesBothLegs(
  std::vector<float> & positions,
  std::vector<float> & velocities,
  std::vector<float> & efforts,
  const std::vector<std::string> & names)
{
  auto convert_one_leg = [&](const char * const * leg_names, bool mirror) {
    Eigen::Vector<double, kLegJointNum> pos, vel, tor;
    if (extractLegJoints(leg_names, positions, names, pos) &&
        extractLegJoints(leg_names, velocities, names, vel) &&
        extractLegJoints(leg_names, efforts, names, tor))
    {
      parallelToSeries(pos, vel, tor, mirror);
      insertLegJoints(leg_names, pos, positions, names);
      insertLegJoints(leg_names, vel, velocities, names);
      insertLegJoints(leg_names, tor, efforts, names);
    }
  };
  convert_one_leg(kLeftLegJointNames, false);
  convert_one_leg(kRightLegJointNames, true);
}

}  // namespace ankle_transform
}  // namespace robot_common
