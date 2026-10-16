#ifndef ROBOT_RECORDER__JOINT_RECORD_SAMPLE_HPP_
#define ROBOT_RECORDER__JOINT_RECORD_SAMPLE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "robot_interfaces/msg/joint_commands.hpp"
#include "robot_interfaces/msg/joint_states.hpp"

namespace robot_recorder
{

// ============================================================================
// 详细记录 —— 常态下使用，每关节数值展开 + IMU
// ============================================================================

/// IMU 采样数据，通过 imu/nav_all 回调实时缓存
struct ImuSample
{
  float angular_vel_x{0.0f};  // 陀螺仪 X 轴角速度 (rad/s)
  float angular_vel_y{0.0f};  // 陀螺仪 Y 轴角速度 (rad/s)
  float angular_vel_z{0.0f};  // 陀螺仪 Z 轴角速度 (rad/s)
  float euler_roll{0.0f};     // 欧拉角 Roll (rad)
  float euler_pitch{0.0f};    // 欧拉角 Pitch (rad)
  float euler_yaw{0.0f};      // 欧拉角 Yaw (rad)
};

/// 单个关节的详细状态数值
/// 不保存 joint_name，关节名称由 JointStatesFrame 外部 joint_names 顺序确定
struct JointStateValues
{
  float position{0.0f};
  float velocity{0.0f};
  float effort{0.0f};
  uint8_t joint_error{0};
  uint8_t motor_error{0};
  float motor_temp{0.0f};
  float mos_temp{0.0f};
};

/// 单个关节的详细命令数值
/// 不保存 joint_name，关节名称由 JointCommandsFrame 外部 joint_names 顺序确定
struct JointCommandValues
{
  float position{0.0f};
  float velocity{0.0f};
  float effort{0.0f};
  float kp{0.0f};
  float kd{0.0f};
  bool enable{false};
};

/// 关节状态详细帧
/// 用于常态持续记录，joints 按固定 joint_names 顺序展开成宽表列
/// recorder_time_ms 统一放在外层 JointStatesRecord，避免 envelope 和 payload 重复保存
struct JointStatesFrame
{
  int64_t msg_stamp_ms{0};
  uint32_t sequence{0};
  uint64_t last_command_trace_id{0};
  uint32_t last_command_sequence{0};
  std::vector<JointStateValues> joints;  // 按固定顺序排列的关节状态
  ImuSample imu;                          // 关联的最新 IMU 数据
};

/// 关节命令详细帧
/// 用于常态持续记录，joints 按固定 joint_names 顺序展开成宽表列
struct JointCommandsFrame
{
  int64_t msg_stamp_ms{0};
  uint64_t trace_id{0};
  uint32_t sequence{0};
  uint32_t parent_sequence{0};  // 直接上游命令序号
  std::string source;
  std::string parent_source;    // 直接上游来源
  std::string stage;            // 当前控制链路阶段
  uint8_t type{0};              // JointCommands.type
  std::vector<JointCommandValues> joints;  // 按固定顺序排列的关节命令
};

/// 关节状态记录。
/// recorder_time_ms 放在外层，供 TimeWindowBuffer 按时间窗口淘汰旧数据。
struct JointStatesRecord
{
  int64_t recorder_time_ms{0};
  JointStatesFrame frame;
};

/// 关节命令记录。
/// 与状态记录分开建模，避免 variant/type 分流，让 buffer 类型直接表达内容。
struct JointCommandsRecord
{
  int64_t recorder_time_ms{0};
  JointCommandsFrame frame;
};

// ============================================================================
// 工具函数
// ============================================================================

/// 将 ROS Time 转换成毫秒，统一 recorder 内部的时间戳单位
int64_t toMillis(const builtin_interfaces::msg::Time & stamp);

/// 从 JointCommands 消息生成详细记录
/// joint_names 决定宽表列顺序；消息中缺失的关节保留默认值，保证列数稳定
JointCommandsRecord makeJointCommandsRecord(
  const robot_interfaces::msg::JointCommands & msg,
  const std::vector<std::string> & joint_names,
  int64_t recorder_time_ms,
  int64_t msg_stamp_ms);

/// 从 JointStates 消息生成详细记录
/// joint_names 决定宽表列顺序；imu 填充关联的 IMU 采样数据
JointStatesRecord makeJointStatesRecord(
  const robot_interfaces::msg::JointStates & msg,
  const std::vector<std::string> & joint_names,
  int64_t recorder_time_ms,
  int64_t msg_stamp_ms,
  const ImuSample & imu = ImuSample{});

/// 生成详细命令 CSV 的关节数值列头。
/// joint_names 应传入已经完成展示名转换的关节名。
std::vector<std::string> commandValueHeaders(const std::vector<std::string> & joint_names);

/// 按 commandValueHeaders() 的字段顺序追加详细命令 CSV 数值列。
void appendCommandValues(
  std::vector<std::string> & row,
  const std::vector<JointCommandValues> & joints);

/// 生成详细状态 CSV 的关节数值列头。
/// joint_names 应传入已经完成展示名转换的关节名。
std::vector<std::string> stateValueHeaders(const std::vector<std::string> & joint_names);

/// 按 stateValueHeaders() 的字段顺序追加详细状态 CSV 数值列。
void appendStateValues(
  std::vector<std::string> & row,
  const std::vector<JointStateValues> & joints);

}  // namespace robot_recorder

#endif  // ROBOT_RECORDER__JOINT_RECORD_SAMPLE_HPP_
