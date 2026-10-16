#include "robot_recorder/joint_record_sample.hpp"

#include <array>
#include <utility>

namespace robot_recorder
{
namespace
{

using CommandFloatMember = float JointCommandValues::*;
using StateFloatMember = float JointStateValues::*;
using StateErrorMember = uint8_t JointStateValues::*;

// 命令 CSV 的关节数值字段顺序。
// 该顺序必须与 kCommandValueSuffixes 和 appendCommandValues() 保持一致。
constexpr std::array<CommandFloatMember, 5> kCommandFloatMembers{
  &JointCommandValues::position,
  &JointCommandValues::velocity,
  &JointCommandValues::effort,
  &JointCommandValues::kp,
  &JointCommandValues::kd,
};

// 命令 CSV 中每组关节列的后缀：先输出所有关节 pos，再输出所有关节 vel，以此类推。
constexpr std::array<const char *, 5> kCommandValueSuffixes{
  "_pos",
  "_vel",
  "_tor",
  "_kp",
  "_kd",
};

// 状态 CSV 的运动量字段顺序：位置、速度、力矩。
constexpr std::array<StateFloatMember, 3> kStateMotionMembers{
  &JointStateValues::position,
  &JointStateValues::velocity,
  &JointStateValues::effort,
};

// 状态 CSV 中运动量列后缀。
constexpr std::array<const char *, 3> kStateMotionSuffixes{
  "_pos",
  "_vel",
  "_tor",
};

// 状态 CSV 的错误码字段顺序：关节错误码、电机原始错误码。
constexpr std::array<StateErrorMember, 2> kStateErrorMembers{
  &JointStateValues::joint_error,
  &JointStateValues::motor_error,
};

// 状态 CSV 中错误码列后缀。
constexpr std::array<const char *, 2> kStateErrorSuffixes{
  "_err",
  "_merr",
};

// 状态 CSV 的温度字段顺序：电机温度、MOS 温度。
constexpr std::array<StateFloatMember, 2> kStateTempMembers{
  &JointStateValues::motor_temp,
  &JointStateValues::mos_temp,
};

// 状态 CSV 中温度列后缀。
constexpr std::array<const char *, 2> kStateTempSuffixes{
  "_temp",
  "_mos",
};

// 按“字段组”追加表头：每个 suffix 先展开所有关节，再追加 --- 作为组分隔。
// 例如 joint_a_pos, joint_b_pos, ---, joint_a_vel, joint_b_vel, ---。
template<size_t N>
void appendGroupedHeaders(
  std::vector<std::string> & headers,
  const std::vector<std::string> & joint_names,
  const std::array<const char *, N> & suffixes)
{
  headers.reserve(headers.size() + (joint_names.size() + 1) * suffixes.size());
  for (const auto * suffix : suffixes) {
    for (const auto & name : joint_names) {
      headers.push_back(name + suffix);
    }
    headers.push_back("---");
  }
}

// 按成员指针数组追加 float 字段组，字段顺序与对应表头 suffix 顺序一致。
template<typename Member, size_t N>
void appendFloatGroups(
  std::vector<std::string> & row,
  const std::vector<JointStateValues> & joints,
  const std::array<Member, N> & members)
{
  for (const auto member : members) {
    for (const auto & joint : joints) {
      row.push_back(std::to_string(joint.*member));
    }
    row.push_back("");
  }
}

}  // namespace

// ROS 时间戳按秒 + 纳秒表示；CSV 中统一落成毫秒，便于和 recorder_time_ms 对齐
int64_t toMillis(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000LL +
         static_cast<int64_t>(stamp.nanosec) / 1000000;
}

// 从 JointCommands 消息生成详细记录
// 按固定 joint_names 顺序展开各关节命令值，保证 CSV 每行列数一致
JointCommandsRecord makeJointCommandsRecord(
  const robot_interfaces::msg::JointCommands & msg,
  const std::vector<std::string> & joint_names,
  int64_t recorder_time_ms,
  int64_t msg_stamp_ms)
{
  JointCommandsFrame frame;
  frame.msg_stamp_ms = msg_stamp_ms;
  frame.trace_id = msg.trace_id;
  frame.sequence = msg.sequence;
  frame.parent_sequence = msg.parent_sequence;
  frame.source = msg.source;
  frame.parent_source = msg.parent_source;
  frame.stage = msg.stage;
  frame.type = msg.type;
  frame.joints.reserve(joint_names.size());

  for (const auto & name : joint_names) {
    JointCommandValues values;
    for (const auto & cmd : msg.commands) {
      if (cmd.joint_name == name) {
        values.position = cmd.position;
        values.velocity = cmd.velocity;
        values.effort = cmd.effort;
        values.kp = cmd.kp;
        values.kd = cmd.kd;
        values.enable = cmd.enable;
        break;
      }
    }
    frame.joints.push_back(std::move(values));
  }

  return JointCommandsRecord{recorder_time_ms, std::move(frame)};
}

// 从 JointStates 消息生成详细记录
// 按固定 joint_names 顺序展开各关节状态，附加当前缓存的 IMU 数据
JointStatesRecord makeJointStatesRecord(
  const robot_interfaces::msg::JointStates & msg,
  const std::vector<std::string> & joint_names,
  int64_t recorder_time_ms,
  int64_t msg_stamp_ms,
  const ImuSample & imu)
{
  JointStatesFrame frame;
  frame.msg_stamp_ms = msg_stamp_ms;
  frame.sequence = msg.sequence;
  frame.last_command_trace_id = msg.last_command_trace_id;
  frame.last_command_sequence = msg.last_command_sequence;
  frame.joints.reserve(joint_names.size());
  frame.imu = imu;

  for (const auto & name : joint_names) {
    JointStateValues values;
    for (const auto & state : msg.states) {
      if (state.joint_name == name) {
        values.position = state.position;
        values.velocity = state.velocity;
        values.effort = state.effort;
        values.joint_error = state.joint_error;
        values.motor_error = state.motor_error;
        values.motor_temp = state.motor_temp;
        values.mos_temp = state.mos_temp;
        break;
      }
    }
    frame.joints.push_back(std::move(values));
  }

  return JointStatesRecord{recorder_time_ms, std::move(frame)};
}

// 生成命令 CSV 的关节数值表头。
// joint_names 应是展示用别名；输出顺序为 pos/vel/tor/kp/kd，每组后追加 --- 分隔。
std::vector<std::string> commandValueHeaders(const std::vector<std::string> & joint_names)
{
  std::vector<std::string> headers;
  appendGroupedHeaders(headers, joint_names, kCommandValueSuffixes);
  return headers;
}

// 按 commandValueHeaders() 的字段顺序追加一行命令数值。
// 只写入 CSV 实际使用的 pos/vel/tor/kp/kd 字段。
void appendCommandValues(
  std::vector<std::string> & row,
  const std::vector<JointCommandValues> & joints)
{
  row.reserve(row.size() + (joints.size() + 1) * kCommandValueSuffixes.size());
  for (const auto member : kCommandFloatMembers) {
    for (const auto & joint : joints) {
      row.push_back(std::to_string(joint.*member));
    }
    row.push_back("");
  }
}

// 生成状态 CSV 的关节数值表头。
// 输出顺序为运动量 pos/vel/tor、错误码 err/merr、温度 temp/mos。
std::vector<std::string> stateValueHeaders(const std::vector<std::string> & joint_names)
{
  std::vector<std::string> headers;
  appendGroupedHeaders(headers, joint_names, kStateMotionSuffixes);
  appendGroupedHeaders(headers, joint_names, kStateErrorSuffixes);
  appendGroupedHeaders(headers, joint_names, kStateTempSuffixes);
  return headers;
}

// 按 stateValueHeaders() 的字段顺序追加一行状态数值。
// 只写入 CSV 实际使用的运动量、错误码和温度字段。
void appendStateValues(
  std::vector<std::string> & row,
  const std::vector<JointStateValues> & joints)
{
  row.reserve(
    row.size() +
    (joints.size() + 1) *
    (kStateMotionMembers.size() + kStateErrorMembers.size() + kStateTempMembers.size()));
  appendFloatGroups(row, joints, kStateMotionMembers);
  for (const auto member : kStateErrorMembers) {
    for (const auto & joint : joints) {
      row.push_back(std::to_string(static_cast<int>(joint.*member)));
    }
    row.push_back("");
  }
  appendFloatGroups(row, joints, kStateTempMembers);
}

}  // namespace robot_recorder
