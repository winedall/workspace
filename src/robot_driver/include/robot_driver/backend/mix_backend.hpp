#ifndef ROBOT_DRIVER__BACKEND__MIX_BACKEND_HPP_
#define ROBOT_DRIVER__BACKEND__MIX_BACKEND_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "robot_driver/backend/hardware_backend.hpp"
#include "robot_driver/canfd/canfd_transport.hpp"
#include "robot_driver/protocol/encos_protocol.hpp"
#include "robot_driver/protocol/tienkung_protocol.hpp"

namespace robot_driver
{

// Mix CANFD 管理事务结果。
//
// 该结构只服务 MixBackend 的低频配置/查询接口，用于聚合 Encos/Tienkung
// 两类电机的返回字段，再由 robot_driver_node 转成对应 ROS service 响应。
struct MixAdminResult
{
  bool success{false};
  std::string message;
  std::string vendor;
  std::uint16_t motor_id{0};
  std::uint16_t new_motor_id{0};
  std::uint8_t feedback_code{0};
  double zero_position_rad{0.0};
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  std::uint16_t motor_error{0};
  float motor_temp{0.0f};
  float mos_temp{0.0f};
};

// 混装 CANFD 后端。
//
// 该 backend 服务 vendor_type=mix、bus_type=canfd 的新硬件形态：
// 同一台机器人中可以同时安装 Encos 和 Tienkung 电机，通过每个关节配置的 vendor
// 选择对应厂家 protocol，再由 CanfdTransport 统一完成 SocketCAN 或 EtherCAT gateway 收发。
//
// 注意：现有 EncosBackend/TienkungBackend 的 EtherCAT 路径保持独立，不通过本类。
class MixBackend : public HardwareBackend
{
public:
  // 正常运行构造函数：加载配置并根据 bus_type 创建真实 CanfdTransport。
  explicit MixBackend(const BackendConfig & config);
  // 测试构造函数：允许注入 fake transport，避免单测访问真实 SocketCAN/EtherCAT。
  MixBackend(const BackendConfig & config, std::unique_ptr<CanfdTransport> transport);
  // 析构时停止 worker 并关闭 CANFD master。
  ~MixBackend() override;

  // 按关节名查询当前状态（仅 config_mode 下用于低频配置维护）。
  MixAdminResult queryStatus(const std::string & joint_name, double timeout_sec);
  // 按关节名设置电机零点。
  MixAdminResult setZero(const std::string & joint_name, double timeout_sec);
  // 按关节名查询电机 ID。
  MixAdminResult queryID(const std::string & joint_name, double timeout_sec);
  // 按关节名修改电机 ID。
  MixAdminResult modifyID(const std::string & joint_name, std::uint16_t new_motor_id, double timeout_sec);

private:
  // 校验 mix/canfd 配置。
  bool validateConfig();
  // 打开 CANFD 总线并建立每个关节的信箱映射。
  void initCanfd();
  // 记录带 CANFD transport 细节的故障，并退出已配置状态。
  void recordCanfdFault(const std::string & message);
  // 按关节 vendor 选择 Encos/Tienkung protocol 编码单个命令。
  bool encodeJointCommand(const robot_interfaces::msg::JointCommand & command) override;
  // 对 mix 中的天工关节执行启动序列；Encos 关节不需要该流程。
  bool runStartup() override;
  // 按单关节事务依次执行 mix 中天工关节的启停状态切换。
  bool runTienkungStartStop(std::uint8_t state, const std::string & action);
  // 对 mix 中单个天工关节执行一次启停事务。
  bool syncTienkungStartStop(JointRuntime & joint, std::uint8_t state, const std::string & action);
  // 等待 mix 中单个天工关节返回指定启停应答。
  bool waitTienkungStartStop(JointRuntime & joint, const HardwareJointConfig & hardware_joint, std::uint8_t expected_state, int max_cycles, const std::string & action);
  // 从 CANFD 输入信箱读取反馈，并刷新统一关节状态。
  void readJointFeedback() override;
  // Mix CANFD 是否可执行周期收发。
  bool transportReady() const override;
  // 执行一次 CANFD process data 交换。
  bool exchangeProcessData() override;
  // CANFD 是否已经完成配置。
  bool configured() const override;
  // 返回关节在 transport 信箱数组中的下标；不存在时返回 hardware_config_.joints.size()。
  std::size_t transportJointIndex(const std::string & joint_name) const;
  
  // 查询指定关节的 transport 输出信箱并写入管理帧。
  bool setAdminOutputFrame(const HardwareJointConfig & hardware_joint, const CanfdFrame & frame, MixAdminResult & result);
  // 查询指定关节的 transport 输入信箱。
  bool getAdminInputFrame(const HardwareJointConfig & hardware_joint, CanfdFrame & frame) const;
  // Encos 标零使用 0x7FF 管理帧，回包不会进入普通 motor_id 信箱，必须走单帧总线事务。
  MixAdminResult setEncosJointZero(const HardwareJointConfig & hardware_joint, double timeout_sec);
  // Tienkung 标零使用 motor_id 定向帧，回包会进入普通信箱，按周期 process data 等待。
  MixAdminResult setTienkungJointZero(const HardwareJointConfig & hardware_joint, double timeout_sec);

  // CANFD 帧传输封装，负责 SocketCAN 直连或 EtherCAT gateway 转发。
  std::unique_ptr<CanfdTransport> transport_;
  // Encos 厂家协议编解码器，供 mix 中的 Encos 关节复用。
  EncosProtocol encos_protocol_;
  // Tienkung 厂家协议编解码器，供 mix 中的 Tienkung 关节复用。
  TienkungProtocol tienkung_protocol_;
  // CANFD master 是否已经完成打开和配置。
  bool canfd_configured_{false};
  // 天工运动帧计数器；Encos 关节不会使用该字段。
  std::uint8_t command_counter_{0};
};

}  // namespace robot_driver

#endif  // ROBOT_DRIVER__BACKEND__MIX_BACKEND_HPP_
