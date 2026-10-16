#pragma once

#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include <Eigen/Core>

#include "sim_common/SimErrorModel.hpp"

namespace sim_common {

// MjInterface 的构造配置。所有模型相关数据经参数传入，
// 使本类与 robot_header 生成头解耦（库保持模型无关）。
struct MjInterfaceConfig {
  // MuJoCo XML 中的关节名/执行器名（来自 robot_header kJointsName/kActuatorsName）。
  std::vector<std::string> joint_names;
  std::vector<std::string> actuator_names;
  // 基座 body 名（如 "torso_link"）。
  std::string base_name;
  // 传感器名；空字符串表示跳过该传感器查询。
  std::string orientation_sensor_name;
  std::string vel_sensor_name;
  std::string gyro_sensor_name;
  std::string acc_sensor_name;
  // 深度相机名；为空则不查询（狗形/无头人形包）。
  std::vector<std::string> camera_names;
  // 是否需要基座位置/速度真值（仿真可直接读取，真机没有对应输出）：
  // true（人形包）：读取基座 qpos/qvel 到 q[0..2]/dq[0..2]（NavAll pos/vel
  //   输出真值），并要求基座带 FREE 关节——真值的来源，否则初始化失败；
  // false（狗形包）：不读基座位姿（NavAll pos/vel 恒为 0），仅要求基座
  //   body 存在。
  bool fill_base_pose{true};
};

// MjInterface 封装 MuJoCo 原始 mjModel/mjData 与项目内部数组类型之间的转换。
//
// MuJoCo 中 qpos/qvel/actuator 的顺序由 XML 决定，不能假设与控制系统
// joint_name 顺序天然一致。因此初始化时先通过名称查出索引，后续控制和
// 状态读取都使用这些缓存索引，避免每个仿真步重复查表。
//
// 内部数组约定（与 robot_header 的 SystemQ/SystemDq/SystemTau 一致）：
// q = [base_pos(3), base_quat(x,y,z,w)(4), joints(n)]，共 n+7；
// dq/tau = [base_vel(3), gyro(3), joints(n)]，共 n+6。
class MjInterface {
 public:
  explicit MjInterface(const MjInterfaceConfig& cfg);
  MjInterface() = default;

  void LoadMjModel(mjModel* model) { model_ = model; }

  // 挂上误差模型（不持有，nullptr = 不注入）。读取函数是 q_rel_/dq_rel_/acc_ 的
  // 唯一来源，发布路径与各包 ComputeControl 的 PD 路径都从这里取值，所以在读取
  // 函数里注入一次，就同时覆盖了"对外发布值"与"控制回路看到的编码器值"。
  void SetErrorModel(SimErrorModel* error_model) { error_model_ = error_model; }

  // 从 MuJoCo 模型解析关节、执行器、基座和传感器名称，建立快速访问索引。
  bool InitDataHandle();

  // 轻量读取：只填关节位置/速度（ComputeControl 路径）。
  // Q 需支持 q[i+7] 赋值，Dq 需支持 dq[i+6] 赋值（Eigen 向量均可）。
  template <typename Q, typename Dq>
  void ReadJointState(const mjData* data, Q& q, Dq& dq) const;

  // 全量读取：关节 + 基座位姿 + 四元数 + 陀螺 + 加速度计（ExchangeMessages 路径）。
  template <typename Q, typename Dq, typename Tau>
  void ReadState(const mjData* data, Q& q, Dq& dq, Tau& tau, Eigen::Vector3d& acc) const;

  std::size_t GetJointCount() const { return cfg_.joint_names.size(); }
  const std::vector<int>& GetMotorIds() const { return motor_id_; }
  const std::vector<int>& GetCamIds() const { return depth_cam_id_; }

  // 以下访问器用于复位/诊断（例如把某一台机器人瞬移回出生点）。
  // 注意 GetJointQposAdr()/GetJointDofAdr() 返回的是 qpos/qvel 数组下标
  // （成员名沿用历史命名 *_id_，实际存的是地址），顺序与 joint_names 一致。
  int GetBaseBodyId() const { return base_id_; }
  int GetBaseQposAdr() const { return base_qpos_adr_; }
  int GetBaseDofAdr() const { return base_dof_adr_; }
  const std::vector<int>& GetJointQposAdr() const { return joints_pos_id_; }
  const std::vector<int>& GetJointDofAdr() const { return joint_vel_id_; }

 private:
  int GetObjId(mjtObj obj_type, const std::string& obj_name) const;
  bool InitJointIds();
  bool InitBaseIds();
  bool InitSensorIds();
  bool InitCameraIds();

  MjInterfaceConfig cfg_;
  mjModel* model_{nullptr};
  std::vector<int> joints_pos_id_;
  std::vector<int> joint_vel_id_;
  std::vector<int> motor_id_;
  int orientation_sensor_id_{-1};
  int base_vel_sensor_id_{-1};
  int gyro_sensor_id_{-1};
  int acc_sensor_id_{-1};
  int base_id_{-1};
  int base_joint_id_{-1};
  int base_qpos_adr_{-1};
  int base_dof_adr_{-1};
  std::vector<int> depth_cam_id_;
  // 误差模型（不持有所有权）。读取函数是 const，但这里是指向非 const 对象的指针，
  // 所以仍可在读取路径里推进误差模型内部状态（噪声缓存、延迟环）。
  SimErrorModel* error_model_{nullptr};
};

template <typename Q, typename Dq>
void MjInterface::ReadJointState(const mjData* data, Q& q, Dq& dq) const {
  for (std::size_t i = 0; i < cfg_.joint_names.size(); ++i) {
    // qpos/qvel 的真实地址来自 joints_pos_id_/joint_vel_id_，不是直接使用 i。
    q[i + 7] = data->qpos[joints_pos_id_[i]];
    dq[i + 6] = data->qvel[joint_vel_id_[i]];
    // 误差注入：模拟"编码器读数 = 真值 + 零位偏置 + 噪声"。注意这里同时是 PD 的
    // 反馈来源与对外发布值的来源，所以零位偏置会真的让关节稳态偏位（与真机一致）。
    if (error_model_ != nullptr) {
      error_model_->PerturbJoint(i, data->time, q[i + 7], dq[i + 6]);
    }
  }
}

template <typename Q, typename Dq, typename Tau>
void MjInterface::ReadState(const mjData* data, Q& q, Dq& dq, Tau& tau,
                            Eigen::Vector3d& acc) const {
  ReadJointState(data, q, dq);
  for (std::size_t i = 0; i < cfg_.joint_names.size(); ++i) {
    // tau 当前取 MuJoCo 控制输入 d->ctrl，表示仿真执行器收到的目标力矩。
    tau[i + 6] = data->ctrl[motor_id_[i]];
  }

  if (cfg_.fill_base_pose) {
    for (int i = 0; i < 3; ++i) {
      q[i] = data->qpos[base_qpos_adr_ + i];
      dq[i] = data->qvel[base_dof_adr_ + i];
    }
  }

  // MuJoCo orientation sensor 输出四元数。这里按项目 SystemQ 的浮动基座
  // 字段顺序重排（MuJoCo 为 w,x,y,z，SystemQ 存 x,y,z,w 于 3..6）。
  for (int i = 0; i < 4; ++i) {
    q[(i + 3) % 4 + 3] = data->sensordata[model_->sensor_adr[orientation_sensor_id_] + i];
  }
  for (int i = 0; i < 3; ++i) {
    dq[i + 3] = data->sensordata[model_->sensor_adr[gyro_sensor_id_] + i];
    acc[i] = data->sensordata[model_->sensor_adr[acc_sensor_id_] + i];
  }

  // 误差注入：IMU 的零偏/白噪声/安装角/姿态误差。要求 q[3..6] 与 dq[3..5] 连续
  // 可取址（Eigen::VectorXd 与 std::vector<double> 都满足）。
  if (error_model_ != nullptr) {
    error_model_->PerturbImu(data->time, &q[3], &dq[3], acc);
  }
}

}  // namespace sim_common
