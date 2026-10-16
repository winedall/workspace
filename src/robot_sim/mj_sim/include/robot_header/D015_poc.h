#pragma once

#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <array>

#include "Eigen/Core"

namespace D015_poc {

/**
 * @brief Auto-generated configuration for D015_poc
 * @note Generated from MuJoCo XML file
 */

// Actuator and Joint configuration
// Actuator names in the order they appear in XML
const std::array<std::string, 12> kActuatorsName = {
    "left_front_hip_roll_motor",
    "left_front_hip_pitch_motor",
    "left_front_knee_motor",
    "right_front_hip_roll_motor",
    "right_front_hip_pitch_motor",
    "right_front_knee_motor",
    "left_rear_hip_roll_motor",
    "left_rear_hip_pitch_motor",
    "left_rear_knee_motor",
    "right_rear_hip_roll_motor",
    "right_rear_hip_pitch_motor",
    "right_rear_knee_motor"
};

// Corresponding joint names in the same order as actuators
const std::array<std::string, 12> kJointsName = {
    "left_front_hip_roll",
    "left_front_hip_pitch",
    "left_front_knee",
    "right_front_hip_roll",
    "right_front_hip_pitch",
    "right_front_knee",
    "left_rear_hip_roll",
    "left_rear_hip_pitch",
    "left_rear_knee",
    "right_rear_hip_roll",
    "right_rear_hip_pitch",
    "right_rear_knee"
};

// Joint torque limits (Nm)
const std::array<double, 12> kJointsMaxTorque = {
    23.7,
    23.7,
    35.55,
    23.7,
    23.7,
    35.55,
    23.7,
    23.7,
    35.55,
    23.7,
    23.7,
    35.55
};
const std::array<double, 12> kJointsMinTorque = {
    -23.7,
    -23.7,
    -35.55,
    -23.7,
    -23.7,
    -35.55,
    -23.7,
    -23.7,
    -35.55,
    -23.7,
    -23.7,
    -35.55
};
const uint8_t kJointsNum = 12;
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

using SystemTau = Eigen::Vector<double, 18>;
using SystemDdq = Eigen::Vector<double, 18>;
using SystemDq = Eigen::Vector<double, 18>;
using SystemQ = Eigen::Vector<double, 19>;
using JointTau = Eigen::Vector<double, 12>;
using JointDdq = Eigen::Vector<double, 12>;
using JointDq = Eigen::Vector<double, 12>;
using JointTau = Eigen::Vector<double, 12>;
} // namespace D015_poc