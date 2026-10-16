#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace motor_spec {

// 电机物理规格，描述单台电机的 TN 曲线（扭矩-速度曲线）参数。
//
// TN 曲线将电机工作区间分为三段：
//   1. |ω| ≤ v1 → 全扭矩可用（恒扭矩区）
//   2. v1 < |ω| ≤ v2 → 扭矩线性下降至 0（恒功率过渡区）
//   3. |ω| > v2 → 扭矩为 0（速度限制区）
//
// drive_coef / brake_coef 区分两种工况下的扭矩能力差异：
//   - 驱动（力矩与速度同向）：反电动势削弱可用扭矩，系数较小
//   - 制动（力矩与速度反向）：反电动势辅助，享有最大扭矩能力
struct MotorSpec {
    double v1;              // 最大扭矩开始衰减的临界速度 (rad/s)，对应 max_torque 点
    double v2;              // 扭矩降为零的速度截距 (rad/s)
    double max_torque;      // 峰值扭矩 (Nm)，对称取绝对值；同时作为执行器力矩指令限幅
    double drive_coef;      // 驱动系数（力矩与速度同向，电机输出机械功）
    double brake_coef;      // 制动系数（力矩与速度反向，电机被反拖/发电）
    const char* name;       // 电机型号标识
};

// 根据 TN 曲线计算当前速度下允许的最大扭矩幅值。
//
// current_speed  — 关节当前速度 (rad/s)
// required_torque — 需求的关节力矩 (Nm)，用于判断驱动/制动工况
// 返回值 — 当前速度下允许的扭矩上限（正数），下限为 -返回值
inline double computeTorqueLimit(const MotorSpec& spec,
                                 double current_speed,
                                 double required_torque) {
    const double abs_speed = std::fabs(current_speed);

    // 力矩与速度同向 → 驱动工况，扭矩能力受反电动势削弱
    // 力矩与速度反向 → 制动工况，享有最大扭矩能力
    const bool is_driving = (required_torque > 0.0 && current_speed > 0.0) ||
                            (required_torque < 0.0 && current_speed < 0.0);
    const double coef = is_driving ? spec.drive_coef : spec.brake_coef;

    if (abs_speed > spec.v2) {
        // 第三段：超过 v2 速度，扭矩降为 0
        return 0.0;
    } else if (abs_speed > spec.v1) {
        // 第二段：恒功率过渡区，扭矩从 max_torque 线性下降到 0
        return spec.max_torque * coef * (spec.v2 - abs_speed) / (spec.v2 - spec.v1);
    } else {
        // 第一段：恒扭矩区，全扭矩可用
        return spec.max_torque * coef;
    }
}

// 对需求力矩施加 TN 曲线物理限制。
inline double clampTorque(const MotorSpec& spec,
                          double current_speed,
                          double required_torque) {
    const double limit = computeTorqueLimit(spec, current_speed, required_torque);
    return std::clamp(required_torque, -limit, limit);
}

// ============================================================
//  预定义电机类型
// ============================================================

// --- 通用参数 ---
// 驱动工况扭矩系数（力矩与速度同向，反电动势削弱扭矩）
constexpr double kDefaultDriveCoef = 0.92;
// 制动工况扭矩系数（力矩与速度反向，享有最大扭矩）
constexpr double kDefaultBrakeCoef = 1.0;

// --- encos 机器人电机型号 ---

// encos4310：手臂 1-5 关节
// TN 曲线：w = (-2.4156 * T + 173.22) / 60 * 2 * pi
//   v1 (T=36 Nm): w = 9.03 rad/s
//   v2 (T=0 Nm):  w = 18.14 rad/s
inline constexpr MotorSpec kMotorEncos4310 = {
    9.03,                 // v1 — 36 Nm 开始衰减的速度
    18.14,                // v2 — 扭矩降为 0 的速度截距
    36.0,                 // max_torque (Nm)
    kDefaultDriveCoef,
    kDefaultBrakeCoef,
    "encos4310"
};

// encos4310 并联：踝关节pitch
inline constexpr MotorSpec kMotorEncos4310ParallelPitch = {
    4.51,                 // v1
    9.07,                // v2
    72.0,                 // max_torque (Nm) — 双电机并联
    kDefaultDriveCoef,
    kDefaultBrakeCoef,
    "encos4310_parallel"
};

// encos6416：腰 yaw、大腿 roll/yaw 关节
// TN 曲线：w = (-0.6681 * T + 108.75) / 60 * 2 * pi
//   v1 (T=120 Nm): w = 2.99 rad/s
//   v2 (T=0 Nm):   w = 11.39 rad/s
inline constexpr MotorSpec kMotorEncos6416 = {
    2.99,
    11.39,
    120.0,
    kDefaultDriveCoef,
    kDefaultBrakeCoef,
    "encos6416"
};

// encos8116：大腿 pitch、膝关节
// TN 曲线：w = (-0.612 * T + 141.48) / 60 * 2 * pi
//   v1 (T=113 Nm): w = 7.57 rad/s
//   v2 (T=0 Nm):   w = 14.82 rad/s
inline constexpr MotorSpec kMotorEncos8116 = {
    7.57,
    14.82,
    113.0,
    kDefaultDriveCoef,
    kDefaultBrakeCoef,
    "encos8116"
};

// encos2806：头部关节、手腕末端关节（臂 6-7）
// TN 曲线：w = (-18.879 * T + 230.336) / 60 * 2 * pi
//   v1 (T=12 Nm): w = 0.40 rad/s
//   v2 (T=0 Nm):  w = 24.12 rad/s
inline constexpr MotorSpec kMotorEncos2806 = {
    0.40,
    24.12,
    12.0,
    kDefaultDriveCoef,
    kDefaultBrakeCoef,
    "encos2806"
};

// --- tienkung 机器人电机型号 ---

// tienkungHRA58P：手臂 1-5 关节
inline constexpr MotorSpec kMotorTienKungHRA58P = {
    8.90,                 // v1 — 25 Nm 开始衰减的速度
    25.65,                // v2 — 扭矩降为 0 的速度截距
    25.0,                 // max_torque (Nm)
    kDefaultDriveCoef,
    kDefaultBrakeCoef,
    "tienkungHRA58P"
};

// tienkungHRA58P 并联：踝关节pitch
inline constexpr MotorSpec kMotorTienKungHRA58PParallelPitch = {
    4.45,                 // v1
    12.82,                // v2
    50.0,                 // max_torque (Nm) — 双电机并联
    kDefaultDriveCoef,
    kDefaultBrakeCoef,
    "tienkungHRA58P_parallel"
};

// tienkungHRA58P：腰 yaw、大腿 1-4 关节
inline constexpr MotorSpec kMotorTienKungHRA88P = {
    4.71,
    20.94,
    140.0,
    kDefaultDriveCoef,
    kDefaultBrakeCoef,
    "tienkungHRA88P"
};

// --- 无限制电机：用于仿真中未指定电机限制的场景 ---
// v1=inf, v2=inf 保证所有速度下扭矩不衰减，max_torque=inf 使指令限幅也无效
inline constexpr MotorSpec kMotorUnlimited = {
    std::numeric_limits<double>::max(),   // v1
    std::numeric_limits<double>::max(),   // v2
    std::numeric_limits<double>::max(),   // max_torque
    1.0,
    1.0,
    "Unlimited"
};

}  // namespace motor_spec
