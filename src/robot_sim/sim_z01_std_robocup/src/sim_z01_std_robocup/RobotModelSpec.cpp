#include "sim_z01_std_robocup/RobotModelSpec.h"

#include <cstdio>
#include <stdexcept>

#include "robot_header/Z01_std_parallel_ankle.h"
#include "robot_header/Z01_std_robocup.h"

namespace sim_z01_std_robocup {
namespace {

// 单机模板命名：全部来自 Z01_std_parallel_ankle 的生成头（该模型与
// Z01_std_football_player 的机器人本体逐字相同）。
constexpr const char* kPrefixFormat = "robot_%02d_";
constexpr const char* kRosPrefixFormat = "/robot_%02d";

// 对外统一关节名：本模型没有 waist_pitch/waist_roll 执行器，踝关节对外统一叫
// left/right_ankle_pitch|roll（MuJoCo 内部为 left/right_ankle_1|2）。
const std::vector<std::string>& ControlJointNamesImpl() {
  static const std::vector<std::string> kNames = {
      "head_yaw",            "head_pitch",           "left_shoulder_pitch", "left_shoulder_roll",
      "left_shoulder_yaw",   "left_elbow",           "left_wrist_yaw",      "left_wrist_pitch",
      "left_wrist_roll",     "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
      "right_elbow",         "right_wrist_yaw",      "right_wrist_pitch",   "right_wrist_roll",
      "waist_yaw",           "left_hip_pitch",       "left_hip_roll",       "left_hip_yaw",
      "left_knee",           "left_ankle_pitch",     "left_ankle_roll",     "right_hip_pitch",
      "right_hip_roll",      "right_hip_yaw",        "right_knee",          "right_ankle_pitch",
      "right_ankle_roll",
  };
  return kNames;
}

}  // namespace

std::string RobotModelSpec::RosNodeName() const {
  return "sim_z01_std_robocup_" + mj_prefix.substr(0, mj_prefix.size() - 1);
}

RobotModelSpec MakeRobotModelSpec(int index) {
  if (index < 0 || index >= RobotModelSpec::kRobotCount) {
    throw std::out_of_range("RobotModelSpec index out of range: " + std::to_string(index));
  }
  char prefix[16] = {0};
  char ros_prefix[16] = {0};
  std::snprintf(prefix, sizeof(prefix), kPrefixFormat, index + 1);
  std::snprintf(ros_prefix, sizeof(ros_prefix), kRosPrefixFormat, index + 1);

  RobotModelSpec spec;
  spec.index = index;
  spec.mj_prefix = prefix;
  spec.ros_prefix = ros_prefix;
  for (const auto& name : Z01_std_parallel_ankle::kJointsName) {
    spec.mj_joint_names.push_back(spec.mj_prefix + name);
  }
  for (const auto& name : Z01_std_parallel_ankle::kActuatorsName) {
    spec.mj_actuator_names.push_back(spec.mj_prefix + name);
  }
  spec.mj_base_name = spec.mj_prefix + Z01_std_parallel_ankle::kBaseName;
  spec.mj_orientation_sensor = spec.mj_prefix + Z01_std_parallel_ankle::kOrientationSensorName;
  spec.mj_gyro_sensor = spec.mj_prefix + Z01_std_parallel_ankle::kGyroSensorName;
  spec.mj_acc_sensor = spec.mj_prefix + Z01_std_parallel_ankle::kAccSensorName;
  spec.mj_camera_name = spec.mj_prefix + Z01_std_parallel_ankle::kCameraNames[0];
  return spec;
}

const std::vector<std::string>& ControlJointNames() { return ControlJointNamesImpl(); }

const sim_common::JointStatesLayout& PublishedJointStatesLayout() {
  // 两个 waist 关节未建模，作为零状态占位插在 waist_yaw 之前（与 football_player 一致）。
  static const sim_common::JointStatesLayout kLayout = sim_common::BuildJointStatesLayout(
      ControlJointNamesImpl(),
      {sim_common::PlaceholdersBefore("waist_yaw", {"waist_pitch", "waist_roll"})});
  return kLayout;
}

const std::array<motor_spec::MotorSpec, RobotModelSpec::kJointCount>& MotorSpecs() {
  // 与 sim_z01_std_football_player 的电机表一致：Z01_std 量产版 TN 数据暂缺，
  // 用占位规格（不限幅、不削减），拿到实测电机数据后再替换。
  static const std::array<motor_spec::MotorSpec, RobotModelSpec::kJointCount> kSpecs = {
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
      motor_spec::kMotorTienKungHRA58P,  // 16: waist_yaw
      motor_spec::kMotorTienKungHRA88P,  // 17: left_hip_pitch
      motor_spec::kMotorTienKungHRA88P,  // 18: left_hip_roll
      motor_spec::kMotorTienKungHRA88P,  // 19: left_hip_yaw
      motor_spec::kMotorTienKungHRA88P,  // 20: left_knee
      motor_spec::kMotorTienKungHRA58P,  // 21: left_ankle_pitch (MuJoCo: left_ankle_1)
      motor_spec::kMotorTienKungHRA58P,  // 22: left_ankle_roll  (MuJoCo: left_ankle_2)
      motor_spec::kMotorTienKungHRA88P,  // 23: right_hip_pitch
      motor_spec::kMotorTienKungHRA88P,  // 24: right_hip_roll
      motor_spec::kMotorTienKungHRA88P,  // 25: right_hip_yaw
      motor_spec::kMotorTienKungHRA88P,  // 26: right_knee
      motor_spec::kMotorTienKungHRA58P,  // 27: right_ankle_pitch (MuJoCo: right_ankle_1)
      motor_spec::kMotorTienKungHRA58P,  // 28: right_ankle_roll  (MuJoCo: right_ankle_2)
  };
  return kSpecs;
}

void ValidateAgainstGeneratedHeader() {
  // Z01_std_robocup.h 里只有 robot_01 那一份名字：header_generator 只取第一个
  // <actuator>/<sensor> 块，而 6 台机器人各自是一个子模型文件（xml/robots/robot_0X.xml）。
  // 因此这里逐名比对 robot_01，校验"前缀 + 模板名"的拼接规则与模型完全一致
  // （模板改名、前缀写错都会在这里立刻暴露），另外用 kCameraNum 校验台数
  // （camera 是 findall，6 台的都能看到）。
  const int header_joints = static_cast<int>(Z01_std_robocup::kJointsNum);
  if (header_joints != RobotModelSpec::kJointCount) {
    throw std::runtime_error("Z01_std_robocup 单机关节数 " + std::to_string(header_joints) +
                             " 与 RobotModelSpec::kJointCount " +
                             std::to_string(RobotModelSpec::kJointCount) + " 不一致");
  }
  if (static_cast<int>(Z01_std_robocup::kCameraNum) != RobotModelSpec::kRobotCount) {
    throw std::runtime_error("Z01_std_robocup 的相机数不等于机器人台数");
  }

  const RobotModelSpec first = MakeRobotModelSpec(0);
  if (Z01_std_robocup::kBaseName != first.mj_base_name) {
    throw std::runtime_error("基座 body 名不匹配：模型里是 " + Z01_std_robocup::kBaseName +
                             "，按前缀拼出的是 " + first.mj_base_name);
  }
  for (int i = 0; i < header_joints; ++i) {
    if (Z01_std_robocup::kJointsName[i] != first.mj_joint_names[i]) {
      throw std::runtime_error("关节名不匹配（第 " + std::to_string(i) + " 个）：模型里是 " +
                               Z01_std_robocup::kJointsName[i] + "，按前缀拼出的是 " +
                               first.mj_joint_names[i]);
    }
    if (Z01_std_robocup::kActuatorsName[i] != first.mj_actuator_names[i]) {
      throw std::runtime_error("执行器名不匹配（第 " + std::to_string(i) + " 个）：模型里是 " +
                               Z01_std_robocup::kActuatorsName[i] + "，按前缀拼出的是 " +
                               first.mj_actuator_names[i]);
    }
  }
}

}  // namespace sim_z01_std_robocup
