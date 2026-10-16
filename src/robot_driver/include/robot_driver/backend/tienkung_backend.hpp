#ifndef ROBOT_DRIVER__BACKEND__TIENKUNG_BACKEND_HPP_
#define ROBOT_DRIVER__BACKEND__TIENKUNG_BACKEND_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "robot_driver/backend/hardware_backend.hpp"
#include "robot_driver/ethercat/ethercat_master.hpp"
#include "robot_driver/ethercat/gateway/tienkung_gateway.hpp"
#include "robot_driver/protocol/tienkung_protocol.hpp"

namespace robot_driver
{

// 天工 EtherCAT 后端。
//
// 这个 backend 负责把统一关节命令映射到天工网关的 CAN-FD 槽位，并通过 IGH
// 主站完成周期收发。它只处理天工硬件和统一 ROS 接口之间的转换，不承担上层
// 控制策略或轨迹规划职责。
class TienkungBackend : public HardwareBackend
{
public:
  // 从 BackendConfig 和硬件配置文件构造天工后端。
  explicit TienkungBackend(const BackendConfig & config);
  // 便于测试注入自定义 EtherCAT 主站实现。
  TienkungBackend(const BackendConfig & config, std::unique_ptr<EthercatMaster> ethercat_master);
  ~TienkungBackend() override;

  // 查询指定关节的电机状态。
  TienkungAdminResult queryStatus(const std::string & joint_name, double timeout_sec);
  // 把指定关节的当前位置设置为电机内部零点。
  TienkungAdminResult setZero(const std::string & joint_name, double timeout_sec);
  // 修改指定关节的电机 ID。
  TienkungAdminResult modifyID(const std::string & joint_name, std::uint8_t new_motor_id, double timeout_sec);
  // 广播查询所有电机 ID。
  TienkungAdminResult queryID(double timeout_sec);
  // 广播重置所有电机 ID 为出厂默认值。
  TienkungAdminResult resetID(double timeout_sec);

 private:
  // 校验天工硬件配置。
  bool validateConfig();
  // 打开并配置 EtherCAT 主站。
  void initEthercat();
  // 记录带 EtherCAT 运行时细节的故障。
  void recordEthercatFault(const std::string & message);
  // 启动阶段一次性完成断使能/使能握手。
  bool runStartup() override;
  // 按单关节事务依次执行启停状态切换。
  bool runStartStop(std::uint8_t state, const std::string & action);
  // 对单个关节执行一次完整启停事务：写指令、交换数据、等待应答。
  bool syncStartStop(JointRuntime & joint, std::uint8_t state, const std::string & action);
  // 等待单个关节返回指定启停应答。
  bool waitStartStop(JointRuntime & joint, const HardwareJointConfig & hardware_joint, std::uint8_t expected_state, int max_cycles, const std::string & action);

  // 把单个关节命令编码到对应的天工槽位。
  bool encodeJointCommand(const robot_interfaces::msg::JointCommand & command) override;
  // 从 EtherCAT 输入缓冲区读取反馈，并刷新统一关节状态。
  void readJointFeedback() override;
  // 天工 EtherCAT 是否可执行周期收发。
  bool transportReady() const override;
  // 执行一次 IGH process data 交换。
  bool exchangeProcessData() override;
  // EtherCAT 是否已经完成配置。
  bool configured() const override;

  // EtherCAT master，当前天工链路使用 IGH 实现。
  std::unique_ptr<EthercatMaster> ethercat_master_;
  // 天工厂家协议编解码器。
  std::unique_ptr<TienkungProtocol> protocol_;
  // 天工 EtherCAT 网关 PDO 槽位适配器。
  TienkungEthercatGateway gateway_;
  // EtherCAT 是否已经完成打开和配置。
  bool ethercat_configured_{false};
  // 天工运动帧计数器，用于协议帧中的滚动计数字段。
  std::uint8_t command_counter_{0};
};

}  // namespace robot_driver

#endif  // ROBOT_DRIVER__BACKEND__TIENKUNG_BACKEND_HPP_
