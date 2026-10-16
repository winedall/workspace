#pragma once

#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <array>

#include "Eigen/Core"

namespace Z01_edm {

/**
 * @brief Auto-generated configuration for Z01_edm
 * @note Generated from MuJoCo XML file
 */

// Actuator and Joint configuration
// Actuator names in the order they appear in XML
const std::array<std::string, 29> kActuatorsName = {
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
    "waist_pitch_motor",
    "waist_roll_motor",
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
    "waist_pitch",
    "waist_roll",
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
    36,
    36,
    36,
    36,
    36,
    12,
    12,
    36,
    36,
    36,
    36,
    36,
    12,
    12,
    7200,
    7200,
    120,
    130,
    120,
    120,
    130,
    72,
    36,
    130,
    120,
    120,
    130,
    72,
    36
};
const std::array<double, 29> kJointsMinTorque = {
    -36,
    -36,
    -36,
    -36,
    -36,
    -12,
    -12,
    -36,
    -36,
    -36,
    -36,
    -36,
    -12,
    -12,
    -7200,
    -7200,
    -120,
    -130,
    -120,
    -120,
    -130,
    -72,
    -36,
    -130,
    -120,
    -120,
    -130,
    -72,
    -36
};
const uint8_t kJointsNum = 29;
const std::string kBaseName = "torso_link";

// Sensor configuration
const std::string kOrientationSensorName = "base_rotation";
const std::string kVelSensorName = "";
const std::string kGyroSensorName = "base_ang_vel";
const std::string kAccSensorName = "base_acc";

// Camera configuration
const uint8_t kCameraNum = 0;
const std::array<std::string, 0> kCameraNames = {};
const std::array<int, 0> kCameraWidths = {};
const std::array<int, 0> kCameraHeights = {};

using SystemTau = Eigen::Vector<double, 35>;
using SystemDdq = Eigen::Vector<double, 35>;
using SystemDq = Eigen::Vector<double, 35>;
using SystemQ = Eigen::Vector<double, 36>;
using JointTau = Eigen::Vector<double, 29>;
using JointDdq = Eigen::Vector<double, 29>;
using JointDq = Eigen::Vector<double, 29>;
using JointTau = Eigen::Vector<double, 29>;
} // namespace Z01_edm