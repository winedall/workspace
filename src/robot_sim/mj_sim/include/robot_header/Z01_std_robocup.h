#pragma once

#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <array>

#include "Eigen/Core"

namespace Z01_std_robocup {

/**
 * @brief Auto-generated configuration for Z01_std_robocup
 * @note Generated from MuJoCo XML file
 */

// Actuator and Joint configuration
// Actuator names in the order they appear in XML
const std::array<std::string, 29> kActuatorsName = {
    "robot_01_head_yaw_motor",
    "robot_01_head_pitch_motor",
    "robot_01_left_shoulder_pitch_motor",
    "robot_01_left_shoulder_roll_motor",
    "robot_01_left_shoulder_yaw_motor",
    "robot_01_left_elbow_motor",
    "robot_01_left_wrist_yaw_motor",
    "robot_01_left_wrist_pitch_motor",
    "robot_01_left_wrist_roll_motor",
    "robot_01_right_shoulder_pitch_motor",
    "robot_01_right_shoulder_roll_motor",
    "robot_01_right_shoulder_yaw_motor",
    "robot_01_right_elbow_motor",
    "robot_01_right_wrist_yaw_motor",
    "robot_01_right_wrist_pitch_motor",
    "robot_01_right_wrist_roll_motor",
    "robot_01_waist_yaw_motor",
    "robot_01_left_hip_pitch_motor",
    "robot_01_left_hip_roll_motor",
    "robot_01_left_hip_yaw_motor",
    "robot_01_left_knee_motor",
    "robot_01_left_ankle_pitch_motor",
    "robot_01_left_ankle_roll_motor",
    "robot_01_right_hip_pitch_motor",
    "robot_01_right_hip_roll_motor",
    "robot_01_right_hip_yaw_motor",
    "robot_01_right_knee_motor",
    "robot_01_right_ankle_pitch_motor",
    "robot_01_right_ankle_roll_motor"
};

// Corresponding joint names in the same order as actuators
const std::array<std::string, 29> kJointsName = {
    "robot_01_head_yaw",
    "robot_01_head_pitch",
    "robot_01_left_shoulder_pitch",
    "robot_01_left_shoulder_roll",
    "robot_01_left_shoulder_yaw",
    "robot_01_left_elbow",
    "robot_01_left_wrist_yaw",
    "robot_01_left_wrist_pitch",
    "robot_01_left_wrist_roll",
    "robot_01_right_shoulder_pitch",
    "robot_01_right_shoulder_roll",
    "robot_01_right_shoulder_yaw",
    "robot_01_right_elbow",
    "robot_01_right_wrist_yaw",
    "robot_01_right_wrist_pitch",
    "robot_01_right_wrist_roll",
    "robot_01_waist_yaw",
    "robot_01_left_hip_pitch",
    "robot_01_left_hip_roll",
    "robot_01_left_hip_yaw",
    "robot_01_left_knee",
    "robot_01_left_ankle_1",
    "robot_01_left_ankle_2",
    "robot_01_right_hip_pitch",
    "robot_01_right_hip_roll",
    "robot_01_right_hip_yaw",
    "robot_01_right_knee",
    "robot_01_right_ankle_1",
    "robot_01_right_ankle_2"
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
    36,
    36,
    140,
    140,
    140,
    140,
    36,
    36
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
    -36,
    -36,
    -140,
    -140,
    -140,
    -140,
    -36,
    -36
};
const uint8_t kJointsNum = 29;
const std::string kBaseName = "robot_01_torso_link";

// Sensor configuration
const std::string kOrientationSensorName = "robot_01_base_rotation";
const std::string kVelSensorName = "";
const std::string kGyroSensorName = "robot_01_base_ang_vel";
const std::string kAccSensorName = "robot_01_base_acc";

// Camera configuration
const uint8_t kCameraNum = 6;
const std::array<std::string, 6> kCameraNames = {
    "robot_01_depth_cam",
    "robot_02_depth_cam",
    "robot_03_depth_cam",
    "robot_04_depth_cam",
    "robot_05_depth_cam",
    "robot_06_depth_cam"
};
const std::array<int, 6> kCameraWidths = {
    64,
    64,
    64,
    64,
    64,
    64
};
const std::array<int, 6> kCameraHeights = {
    36,
    36,
    36,
    36,
    36,
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
} // namespace Z01_std_robocup