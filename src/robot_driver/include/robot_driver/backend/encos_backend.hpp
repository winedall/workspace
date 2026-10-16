#ifndef ROBOT_DRIVER__BACKEND__ENCOS_BACKEND_HPP_
#define ROBOT_DRIVER__BACKEND__ENCOS_BACKEND_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "robot_driver/backend/hardware_backend.hpp"
#include "robot_driver/ethercat/ethercat_master.hpp"
#include "robot_driver/ethercat/gateway/encos_gateway.hpp"
#include "robot_driver/protocol/encos_protocol.hpp"

namespace robot_driver
{

// Encos EtherCAT 后端。
class EncosBackend : public HardwareBackend
{
public:
  // 构造阶段加载配置并打开 EtherCAT 做只读链路初始化。
  explicit EncosBackend(const BackendConfig & config);
  // 测试专用构造函数。
  EncosBackend(const BackendConfig & config, std::unique_ptr<EthercatMaster> ethercat_master);
  ~EncosBackend() override;

  // 按关节名读取电机 ID。
  EncosAdminResult getMotorId(const std::string & joint_name, double timeout_sec);
  // 按关节名设置电机 ID。
  EncosAdminResult setMotorId(const std::string & joint_name, std::uint16_t new_motor_id, double timeout_sec);
  // 按关节名设置电机零点。
  EncosAdminResult setMotorZero(const std::string & joint_name, double timeout_sec);
  // 按关节名读取厂家参数。
  EncosAdminResult getMotorParam(const std::string & joint_name, std::uint8_t param_code, double timeout_sec);

 private:
  // 校验 Encos 硬件配置。
  bool validateConfig();
  // 初始化 EtherCAT 链路。
  void initEthercat();

  // 通过 Encos 协议对象把统一 JointCommand 写入对应从站 PDO 输出缓冲区。
  bool encodeJointCommand(const robot_interfaces::msg::JointCommand & command) override;
  // 从 EtherCAT 输入缓冲区读取反馈，并刷新统一关节状态。
  void readJointFeedback() override;
  // Encos EtherCAT 是否可执行周期收发。
  bool transportReady() const override;
  // 执行一次 SOEM process data 交换。
  bool exchangeProcessData() override;
  // EtherCAT 是否已经完成配置。
  bool configured() const override;

 private:
  // EtherCAT master。负责总线 IO。
  std::unique_ptr<EthercatMaster> ethercat_master_;
  // 厂商协议编码器。当前为 Encos；后续 Tienkung 等 EtherCAT 厂商应新增自己的 Protocol，
  std::unique_ptr<EncosProtocol> protocol_;
  // EtherCAT PDO 与 Encos CAN 帧之间的槽位适配器。
  EncosEthercatGateway gateway_;
  // EtherCAT 初始化是否完整成功。
  bool ethercat_configured_{false};
};

}  // namespace robot_driver

#endif  // ROBOT_DRIVER__BACKEND__ENCOS_BACKEND_HPP_
