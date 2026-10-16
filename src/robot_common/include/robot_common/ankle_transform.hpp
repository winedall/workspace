#ifndef ROBOT_COMMON__ANKLE_TRANSFORM_HPP_
#define ROBOT_COMMON__ANKLE_TRANSFORM_HPP_

#include <Eigen/Dense>

#include <string>
#include <type_traits>
#include <vector>

namespace robot_common
{
namespace ankle_transform
{

constexpr int kLegJointNum = 6;
constexpr int kAnklePitchIdx = 4;
constexpr int kAnkleRollIdx = 5;

constexpr double kAnklePitchMax = 0.51;
constexpr double kAnklePitchMin = -0.87;
// constexpr double kAnklePitchMin = -0.9;
constexpr double kAnkleRollMax = 0.26;
constexpr double kAnkleRollMin = -0.26;

constexpr double kIkSumMax = 0.36;
constexpr double kIkSumMin = -0.36;
constexpr double kIkDiffMax = 2.2;
constexpr double kIkDiffMin = -1.2;

// 新框架腿部关节命名规则。
// 每条腿 6 个关节，顺序：hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll。
constexpr const char * kLeftLegJointNames[kLegJointNum] = {
  "left_hip_pitch", "left_hip_roll", "left_hip_yaw",
  "left_knee", "left_ankle_pitch", "left_ankle_roll"};
constexpr const char * kRightLegJointNames[kLegJointNum] = {
  "right_hip_pitch", "right_hip_roll", "right_hip_yaw",
  "right_knee", "right_ankle_pitch", "right_ankle_roll"};

// ── 类型萃取：检测 T 是否含有 kp/kd 字段 ─────────────────────────────
template<typename, typename = void>
struct has_kp_kd : std::false_type {};

template<typename T>
struct has_kp_kd<T, std::void_t<decltype(std::declval<T>().kp), decltype(std::declval<T>().kd)>>
  : std::true_type {};

template<typename T>
inline constexpr bool has_kp_kd_v = has_kp_kd<T>::value;

// ── 正运动学（MATLAB 生成，闭式解） ──────────────────────────────────
class FastAnkleFk
{
public:
  static void Calculate(const Eigen::Vector2d & f, Eigen::Vector2d & t);
  static void CheckInput(Eigen::Vector2d & f);
};

// ── 逆运动学（Newton 迭代） ──────────────────────────────────────────
class FastAnkleIk
{
public:
  static void Calculate(
    const Eigen::Vector2d & t, Eigen::Vector2d & f,
    double tol = 1e-5, int max_iter = 20, double damp = 0.9);
  static void CheckInput(Eigen::Vector2d & t);
};

// ── 雅可比矩阵（MATLAB 生成，闭式解） ────────────────────────────────
class FastAnkleJacob
{
public:
  static void Calculate(const Eigen::Vector2d & f, Eigen::Matrix2d & J);
};

// ── 底层单腿转换函数 ─────────────────────────────────────────────────
//
// 操作单腿 6 关节数据向量。
// 关节顺序：[hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll]
// is_mirror: false = 左腿, true = 右腿

// 关节位置转换（H2L）
void AnklePosH2L(
  const Eigen::Vector<double, kLegJointNum> & hpos,
  Eigen::Vector<double, kLegJointNum> & lpos,
  bool is_mirror);

// 关节位置转换（L2H）
void AnklePosL2H(
  Eigen::Vector<double, kLegJointNum> & hpos,
  const Eigen::Vector<double, kLegJointNum> & lpos,
  bool is_mirror);

// 关节全量转换（H2L）：pos, vel, tor, kp, kd
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
  bool is_mirror);

// 关节全量转换（H2L）：pos, vel, tor（无 kp/kd，用于状态反馈路径）
void AnkleAllH2L(
  const Eigen::Vector<double, kLegJointNum> & hpos,
  Eigen::Vector<double, kLegJointNum> & lpos,
  const Eigen::Vector<double, kLegJointNum> & hvel,
  Eigen::Vector<double, kLegJointNum> & lvel,
  const Eigen::Vector<double, kLegJointNum> & htor,
  Eigen::Vector<double, kLegJointNum> & ltor,
  bool is_mirror);

// 关节全量转换（L2H）：pos, vel, tor
void AnkleAllL2H(
  Eigen::Vector<double, kLegJointNum> & hpos,
  const Eigen::Vector<double, kLegJointNum> & lpos,
  Eigen::Vector<double, kLegJointNum> & hvel,
  const Eigen::Vector<double, kLegJointNum> & lvel,
  Eigen::Vector<double, kLegJointNum> & htor,
  const Eigen::Vector<double, kLegJointNum> & ltor,
  bool is_mirror);

// 伪逆矩阵（用于逆运动学）
Eigen::MatrixXd PseudoInverse(const Eigen::MatrixXd & mat);

// ── 力矩前馈模式（H2L） ──────────────────────────────────────────────
//
// 与 AnkleAllH2L 的增益等效方案并列的另一种转换：在串联空间用模型自带的
// kp/kd 计算 PD 力矩，再用 J⁻ᵀ 映射成并联电机力矩，由电机侧以纯力矩
// （kp=kd=0）执行。
//
//   τ_H = effort_ff + kp_H ⊙ (q*_H − q_H) + kd_H ⊙ (q̇*_H − q̇_H)
//   τ_L = J(q_H,meas)⁻ᵀ · τ_H
//
// 与增益等效方案的区别：不引入对角近似、不锁定 r_opt 比例、不丢弃 roll
// 增益，结果精确等于训练时串联空间的 PD。J 取**实测姿态**（力矩映射是当前
// 构型下的瞬时关系），实测角先按串联限位钳位。
//
// 输入（均为单腿 6 关节向量，关节顺序见 kLeftLegJointNames）：
//   hpos_cmd, hvel_cmd, hkp, hkd, htor_ff  命令（串联空间）
//   hpos_meas, hvel_meas                   实测（串联空间）
// 输出：
//   ltor  并联电机力矩（2 维：pitch 电机, roll 电机）
void AnkleTorqueH2L(
  const Eigen::Vector<double, kLegJointNum> & hpos_cmd,
  const Eigen::Vector<double, kLegJointNum> & hvel_cmd,
  const Eigen::Vector<double, kLegJointNum> & hkp,
  const Eigen::Vector<double, kLegJointNum> & hkd,
  const Eigen::Vector<double, kLegJointNum> & htor_ff,
  const Eigen::Vector<double, kLegJointNum> & hpos_meas,
  const Eigen::Vector<double, kLegJointNum> & hvel_meas,
  Eigen::Vector2d & ltor,
  bool is_mirror);

// ── 高层消息转换函数 ─────────────────────────────────────────────────
//
// seriesToParallel: 串联→并联（S2P），命令路径
// parallelToSeries: 并联→串联（P2S），状态反馈路径

// 向量 API（操作关节顺序数组中的腿部数据）
// 串联→并联（S2P）
void seriesToParallel(
  Eigen::Vector<double, kLegJointNum> & pos,
  Eigen::Vector<double, kLegJointNum> & vel,
  Eigen::Vector<double, kLegJointNum> & tor,
  Eigen::Vector<double, kLegJointNum> & kp,
  Eigen::Vector<double, kLegJointNum> & kd,
  bool is_mirror);

// 串联→并联（S2P）：pos, vel, tor（无 kp/kd，用于状态反馈路径）
void seriesToParallel(
  Eigen::Vector<double, kLegJointNum> & pos,
  Eigen::Vector<double, kLegJointNum> & vel,
  Eigen::Vector<double, kLegJointNum> & tor,
  bool is_mirror);

// 并联→串联（P2S）
void parallelToSeries(
  Eigen::Vector<double, kLegJointNum> & pos,
  Eigen::Vector<double, kLegJointNum> & vel,
  Eigen::Vector<double, kLegJointNum> & tor,
  bool is_mirror);

// 在平面命名数组上原地对双腿执行 S2P 或 P2S 转换。
// 组合 extractLegJoints → seriesToParallel/parallelToSeries → insertLegJoints，
// 作用于左右双腿。positions/velocities/efforts 被原地修改。
// 串联→并联（S2P）
void seriesToParallelBothLegs(
  std::vector<float> & positions,
  std::vector<float> & velocities,
  std::vector<float> & efforts,
  std::vector<float> & kps,
  std::vector<float> & kds,
  const std::vector<std::string> & names);

// 串联→并联（S2P）：pos, vel, tor（无 kp/kd，用于状态反馈路径）
void seriesToParallelBothLegs(
  std::vector<float> & positions,
  std::vector<float> & velocities,
  std::vector<float> & efforts,
  const std::vector<std::string> & names);

// 并联→串联（P2S）
void parallelToSeriesBothLegs(
  std::vector<float> & positions,
  std::vector<float> & velocities,
  std::vector<float> & efforts,
  const std::vector<std::string> & names);

// 从带名称的关节列表中提取腿部 6 关节值到 6 维向量。
// 若全部 6 个关节均找到则返回 true。
bool extractLegJoints(
  const char * const * leg_joint_names,
  const std::vector<float> & positions,
  const std::vector<std::string> & joint_names,
  Eigen::Vector<double, kLegJointNum> & out);

// 按名称将 6 维向量写回带名称的关节列表中。
void insertLegJoints(
  const char * const * leg_joint_names,
  const Eigen::Vector<double, kLegJointNum> & values,
  std::vector<float> & positions,
  const std::vector<std::string> & joint_names);

// ── 拆包 → 转换 → 写回辅助模板 ──────────────────────────────────────
//
// 将结构体向量（包含 .joint_name, .position, .velocity, .effort 字段）拆包，
// 对双腿原地执行踝关节转换，再写回。
// 适用于任何含有这些字段的结构体类型（JointCommand, JointState 等），
// 位于头文件中，不依赖 ROS 包。
//
// convertToSeries  — P2S：状态反馈（driver 并联 → 串联）
template<typename T>
inline void convertToSeries(std::vector<T> & items)
{
  std::vector<std::string> names;
  std::vector<float> positions, velocities, efforts;
  names.reserve(items.size());
  positions.reserve(items.size());
  velocities.reserve(items.size());
  efforts.reserve(items.size());
  for (const auto & item : items) {
    names.push_back(item.joint_name);
    positions.push_back(item.position);
    velocities.push_back(item.velocity);
    efforts.push_back(item.effort);
  }

  parallelToSeriesBothLegs(positions, velocities, efforts, names);

  for (std::size_t i = 0; i < items.size(); ++i) {
    items[i].position = positions[i];
    items[i].velocity = velocities[i];
    items[i].effort = efforts[i];
  }
}

// convertToParallel — S2P：命令路径（control 串联 → driver 并联）
// 通过 if constexpr 自动适配：有 kp/kd 的类型（如 JointCommand）转换全部字段，
// 无 kp/kd 的类型（如 JointState）仅转换 pos/vel/effort。
template<typename T>
inline void convertToParallel(std::vector<T> & items)
{
  std::vector<std::string> names;
  std::vector<float> positions, velocities, efforts, kps, kds;
  names.reserve(items.size());
  positions.reserve(items.size());
  velocities.reserve(items.size());
  efforts.reserve(items.size());

  if constexpr (has_kp_kd_v<T>) {
    kps.reserve(items.size());
    kds.reserve(items.size());
  }

  for (const auto & item : items) {
    names.push_back(item.joint_name);
    positions.push_back(item.position);
    velocities.push_back(item.velocity);
    efforts.push_back(item.effort);
    if constexpr (has_kp_kd_v<T>) {
      kps.push_back(item.kp);
      kds.push_back(item.kd);
    }
  }

  if constexpr (has_kp_kd_v<T>) {
    seriesToParallelBothLegs(positions, velocities, efforts, kps, kds, names);
  } else {
    seriesToParallelBothLegs(positions, velocities, efforts, names);
  }

  for (std::size_t i = 0; i < items.size(); ++i) {
    items[i].position = positions[i];
    items[i].velocity = velocities[i];
    items[i].effort = efforts[i];
    if constexpr (has_kp_kd_v<T>) {
      items[i].kp = kps[i];
      items[i].kd = kds[i];
    }
  }
}

}  // namespace ankle_transform
}  // namespace robot_common

#endif  // ROBOT_COMMON__ANKLE_TRANSFORM_HPP_