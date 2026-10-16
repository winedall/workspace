#ifndef ROBOT_RECORDER__JOINT_NAME_ALIAS_HPP_
#define ROBOT_RECORDER__JOINT_NAME_ALIAS_HPP_

#include <string>
#include <unordered_map>

namespace robot_recorder
{

inline std::string jointNameAlias(const std::string & name)
{
  static const std::unordered_map<std::string, std::string> kAliasMap = {
    {"head_yaw",              "head_1"},
    {"head_pitch",            "head_2"},
    {"left_shoulder_pitch",   "arm_l_1"},
    {"left_shoulder_roll",    "arm_l_2"},
    {"left_shoulder_yaw",     "arm_l_3"},
    {"left_elbow",            "arm_l_4"},
    {"left_wrist_yaw",        "arm_l_5"},
    {"left_wrist_pitch",      "arm_l_6"},
    {"left_wrist_roll",       "arm_l_7"},
    {"right_shoulder_pitch",  "arm_r_1"},
    {"right_shoulder_roll",   "arm_r_2"},
    {"right_shoulder_yaw",    "arm_r_3"},
    {"right_elbow",           "arm_r_4"},
    {"right_wrist_yaw",       "arm_r_5"},
    {"right_wrist_pitch",     "arm_r_6"},
    {"right_wrist_roll",      "arm_r_7"},
    {"waist_pitch",           "waist_1"},
    {"waist_roll",            "waist_2"},
    {"waist_yaw",             "waist_3"},
    {"left_hip_pitch",        "leg_l_1"},
    {"left_hip_roll",         "leg_l_2"},
    {"left_hip_yaw",          "leg_l_3"},
    {"left_knee",             "leg_l_4"},
    {"left_ankle_pitch",      "leg_l_5"},
    {"left_ankle_roll",       "leg_l_6"},
    {"right_hip_pitch",       "leg_r_1"},
    {"right_hip_roll",        "leg_r_2"},
    {"right_hip_yaw",         "leg_r_3"},
    {"right_knee",            "leg_r_4"},
    {"right_ankle_pitch",     "leg_r_5"},
    {"right_ankle_roll",      "leg_r_6"},
  };

  auto it = kAliasMap.find(name);
  return (it != kAliasMap.end()) ? it->second : name;
}

}  // namespace robot_recorder

#endif  // ROBOT_RECORDER__JOINT_NAME_ALIAS_HPP_
