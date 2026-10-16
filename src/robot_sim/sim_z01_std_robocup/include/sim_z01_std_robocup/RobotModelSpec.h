#ifndef SIMZ01STDROBOCUP_ROBOTMODELSPEC_H
#define SIMZ01STDROBOCUP_ROBOTMODELSPEC_H

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "mj_sim/MotorSpec.h"
#include "sim_common/SimRosNodeBase.hpp"

// RobotModelSpec 是"多机器人模型里某一台机器人"的命名与关节/电机表的单一来源。
//
// 为什么要在 C++ 里拼名字，而不是用生成的 robot_header/Z01_std_robocup.h：
// header_generator.py 只取第一个 <actuator>/<sensor> 块和第一个带 free joint 的
// body，174 个名字里只有 robot_01 的那 29 个是"第一块"，其余机器人拿不到；
// 而 6 台机器人在 XML 里的差别只是统一前缀，所以用
// "robot_0X_" + Z01_std_parallel_ankle 模板名 拼接最稳。
//
// 对外（ROS 侧）的关节名不带前缀，与 sim_z01_std_football_player 完全一致；
// robot_control/robot_rl_manager 只需按话题前缀区分机器人，不需要知道前缀命名规则。
namespace sim_z01_std_robocup {

struct RobotModelSpec {
  // 与 MuJoCo 模型中的实际数量一致（Z01_std_parallel_ankle 的 29 个执行器）。
  static constexpr int kRobotCount = 6;
  static constexpr int kJointCount = 29;

  int index{0};            // 0..5 -> robot_01..robot_06
  std::string mj_prefix;   // MuJoCo 对象前缀，例如 "robot_01_"
  std::string ros_prefix;  // ROS 话题前缀，例如 "/robot_01"

  std::vector<std::string> mj_joint_names;       // robot_01_left_hip_pitch ...
  std::vector<std::string> mj_actuator_names;    // robot_01_left_hip_pitch_motor ...
  std::string mj_base_name;                      // robot_01_torso_link
  std::string mj_orientation_sensor;             // robot_01_base_rotation
  std::string mj_gyro_sensor;                    // robot_01_base_ang_vel
  std::string mj_acc_sensor;                     // robot_01_base_acc
  std::string mj_camera_name;                    // robot_01_depth_cam

  // ROS 节点名：sim_z01_std_robocup_robot_01 ...
  std::string RosNodeName() const;
  // 队伍：1 = robot_01~03（y=+4.5 边线外，守 x=+7.02 球门）；2 = robot_04~06。
  int team() const { return index < kRobotCount / 2 ? 1 : 2; }
};

// index 越界时抛 std::out_of_range。
RobotModelSpec MakeRobotModelSpec(int index);

// 控制系统统一关节名（不带前缀，顺序与 MuJoCo 侧的 kJointsName 严格对齐：
// left_ankle_pitch/left_ankle_roll 对应模型里的 left_ankle_1/left_ankle_2）。
const std::vector<std::string>& ControlJointNames();

// 对外发布的 JointStates 布局：29 个仿真关节 + waist_pitch/waist_roll 两个占位
// （本模型腰部只有 waist_yaw 执行器），与 football_player 的 31 项视图一致。
const sim_common::JointStatesLayout& PublishedJointStatesLayout();

// 电机表，按控制关节顺序排列（与 sim_z01_std_football_player 相同）。
const std::array<motor_spec::MotorSpec, RobotModelSpec::kJointCount>& MotorSpecs();

// 启动自检：RobotModelSpec 的关节数与生成的 robot_header 不匹配时抛异常，
// 避免把"名字拼错"延迟成运行期的静默失效。
void ValidateAgainstGeneratedHeader();

}  // namespace sim_z01_std_robocup

#endif  // SIMZ01STDROBOCUP_ROBOTMODELSPEC_H
