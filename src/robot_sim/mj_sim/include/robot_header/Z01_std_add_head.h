#pragma once

#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <array>

#include "Eigen/Core"

namespace Z01_std_add_head {

/**
 * @brief Auto-generated configuration for Z01_std_add_head
 * @note Generated from MuJoCo XML file
 */

// Actuator and Joint configuration
// Actuator names in the order they appear in XML
const std::array<std::string, 29> kActuatorsName = {
    "head_yaw_motor",
    "head_pitch_motor",
    "left_shoulder_pitch_motor",
    "left_shoulder_roll_motor",
    "left_shoulder_yaw_motor",
    "left_elbow_motor",
    "left_wrist_yaw_motor",
    "left_wrist_pitch_motor",
    "left_wrist_roll_motor",
    "right_shoulder_pitch_motor",
    "right_shoulder_roll_motor",
    "right_shoulder_yaw_motor",
    "right_elbow_motor",
    "right_wrist_yaw_motor",
    "right_wrist_pitch_motor",
    "right_wrist_roll_motor",
    "waist_yaw_motor",
    "left_hip_pitch_motor",
    "left_hip_roll_motor",
    "left_hip_yaw_motor",
    "left_knee_motor",
    "left_ankle_pitch_motor",
    "left_ankle_roll_motor",
    "right_hip_pitch_motor",
    "right_hip_roll_motor",
    "right_hip_yaw_motor",
    "right_knee_motor",
    "right_ankle_pitch_motor",
    "right_ankle_roll_motor"
};

// Corresponding joint names in the same order as actuators
const std::array<std::string, 29> kJointsName = {
    "head_yaw",
    "head_pitch",
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_yaw",
    "left_wrist_pitch",
    "left_wrist_roll",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
    "right_wrist_yaw",
    "right_wrist_pitch",
    "right_wrist_roll",
    "waist_yaw",
    "left_hip_pitch",
    "left_hip_roll",
    "left_hip_yaw",
    "left_knee",
    "left_ankle_pitch",
    "left_ankle_roll",
    "right_hip_pitch",
    "right_hip_roll",
    "right_hip_yaw",
    "right_knee",
    "right_ankle_pitch",
    "right_ankle_roll"
};

// Joint torque limits (Nm)
const std::array<double, 29> kJointsMaxTorque = {
    12,
    12,
    25,
    25,
    25,
    25,
    25,
    12,
    12,
    25,
    25,
    25,
    25,
    25,
    12,
    12,
    140,
    140,
    140,
    140,
    140,
    50,
    25,
    140,
    140,
    140,
    140,
    50,
    25
};
const std::array<double, 29> kJointsMinTorque = {
    -12,
    -12,
    -25,
    -25,
    -25,
    -25,
    -25,
    -12,
    -12,
    -25,
    -25,
    -25,
    -25,
    -25,
    -12,
    -12,
    -140,
    -140,
    -140,
    -140,
    -140,
    -50,
    -25,
    -140,
    -140,
    -140,
    -140,
    -50,
    -25
};
const uint8_t kJointsNum = 29;
const std::string kBaseName = "torso_link";

// Sensor configuration
const std::string kOrientationSensorName = "base_rotation";
const std::string kVelSensorName = "";
const std::string kGyroSensorName = "base_ang_vel";
const std::string kAccSensorName = "base_acc";

// Camera configuration
const uint8_t kCameraNum = 1;
const std::array<std::string, 1> kCameraNames = {
    "depth_cam"
};
const std::array<int, 1> kCameraWidths = {
    64
};
const std::array<int, 1> kCameraHeights = {
    36
};

using SystemTau = Eigen::Vector<double, 35>;
using SystemDdq = Eigen::Vector<double, 35>;
using SystemDq = Eigen::Vector<double, 35>;
using SystemQ = Eigen::Vector<double, 36>;
using JointTau = Eigen::Vector<double, 29>;
using JointDdq = Eigen::Vector<double, 29>;
using JointDq = Eigen::Vector<double, 29>;
using JointTau = Eigen::Vector<double, 29>;
} // namespace Z01_std_add_head