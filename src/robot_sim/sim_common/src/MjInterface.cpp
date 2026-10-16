#include "sim_common/MjInterface.hpp"

#include <iostream>

namespace sim_common {

MjInterface::MjInterface(const MjInterfaceConfig& cfg) : cfg_(cfg) {}

bool MjInterface::InitDataHandle() {
  if (model_ == nullptr) {
    std::cerr << "Cannot initialize MuJoCo interface: model is null" << std::endl;
    return false;
  }
  // 任一必要对象找不到时返回 false，让上层尽早暴露模型/头文件不一致问题。
  return InitJointIds() && InitBaseIds() && InitSensorIds() && InitCameraIds();
}

bool MjInterface::InitJointIds() {
  // 这些数组保存 MuJoCo 内部索引。MuJoCo 的 qpos/qvel/actuator 顺序由 XML 决定，
  // 不能直接假设等于 joint_names/actuator_names 的数组下标。
  const std::size_t n = cfg_.joint_names.size();
  joints_pos_id_.assign(n, -1);
  joint_vel_id_.assign(n, -1);
  motor_id_.assign(n, -1);

  for (std::size_t i = 0; i < n; ++i) {
    // 通过关节名称查 qpos/qvel 地址，用于后续快速读取关节位置和速度。
    const int joint_id = GetObjId(mjOBJ_JOINT, cfg_.joint_names[i]);
    if (joint_id == -1) {
      return false;
    }
    joints_pos_id_[i] = model_->jnt_qposadr[joint_id];
    joint_vel_id_[i] = model_->jnt_dofadr[joint_id];

    // 通过执行器名称查 actuator id，用于后续写入 d->ctrl。
    // 力矩指令限幅不在此读取：由 motor_spec::MotorSpec 的 max_torque 承载，
    // 狗形/人形仿真共用同一套力矩计算路径。
    const int actuator_id = GetObjId(mjOBJ_ACTUATOR, cfg_.actuator_names[i]);
    if (actuator_id == -1) {
      return false;
    }
    motor_id_[i] = actuator_id;
  }
  return true;
}

bool MjInterface::InitBaseIds() {
  base_id_ = GetObjId(mjOBJ_BODY, cfg_.base_name);
  if (base_id_ == -1) {
    return false;
  }
  if (!cfg_.fill_base_pose) {
    return true;
  }
  if (model_->body_jntnum[base_id_] > 0) {
    base_joint_id_ = model_->body_jntadr[base_id_];
    base_qpos_adr_ = model_->jnt_qposadr[base_joint_id_];
    base_dof_adr_ = model_->jnt_dofadr[base_joint_id_];

    // （可选）确认是自由关节
    if (model_->jnt_type[base_joint_id_] != mjJNT_FREE) {
      std::cerr << "Warning: base joint is not FREE type!" << std::endl;
    }
  } else {
    std::cerr << "No joint attached to base body!" << std::endl;
    return false;
  }
  return true;
}

bool MjInterface::InitSensorIds() {
  // 传感器名称与 XML 中的 sensor 名称保持一致。
  if (!cfg_.orientation_sensor_name.empty()) {
    orientation_sensor_id_ = GetObjId(mjOBJ_SENSOR, cfg_.orientation_sensor_name);
    if (orientation_sensor_id_ == -1) {
      return false;
    }
  }
  if (!cfg_.vel_sensor_name.empty()) {
    base_vel_sensor_id_ = GetObjId(mjOBJ_SENSOR, cfg_.vel_sensor_name);
    if (base_vel_sensor_id_ == -1) {
      return false;
    }
  }
  if (!cfg_.gyro_sensor_name.empty()) {
    gyro_sensor_id_ = GetObjId(mjOBJ_SENSOR, cfg_.gyro_sensor_name);
    if (gyro_sensor_id_ == -1) {
      return false;
    }
  }
  if (!cfg_.acc_sensor_name.empty()) {
    acc_sensor_id_ = GetObjId(mjOBJ_SENSOR, cfg_.acc_sensor_name);
    if (acc_sensor_id_ == -1) {
      return false;
    }
  }
  return true;
}

bool MjInterface::InitCameraIds() {
  // 深度相机（仅 add_head/parallel 模型包含相机）。
  for (const auto& camera_name : cfg_.camera_names) {
    const int cam_id = GetObjId(mjOBJ_CAMERA, camera_name);
    if (cam_id == -1) {
      return false;
    }
    depth_cam_id_.push_back(cam_id);
  }
  return true;
}

int MjInterface::GetObjId(mjtObj obj_type, const std::string& obj_name) const {
  if (obj_name.empty()) {
    std::cerr << "Required MuJoCo object name is empty, type=" << obj_type << std::endl;
    return -1;
  }
  // MuJoCo 名称查找失败通常意味着 robot_header 与 XML 模型不匹配。
  const int tmp_id = mj_name2id(model_, obj_type, obj_name.c_str());
  if (tmp_id == -1) {
    std::cerr << "Type: " << obj_type << ", name: " << obj_name
              << " not found in the XML file!" << std::endl;
    return -1;
  }
  return tmp_id;
}

}  // namespace sim_common
