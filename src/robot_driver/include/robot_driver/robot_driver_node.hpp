#ifndef ROBOT_DRIVER__ROBOT_DRIVER_NODE_HPP_
#define ROBOT_DRIVER__ROBOT_DRIVER_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robot_driver/backend/hardware_backend.hpp"
#include "robot_interfaces/msg/driver_status.hpp"
#include "robot_interfaces/msg/joint_commands.hpp"
#include "robot_interfaces/msg/joint_states.hpp"
#include "robot_interfaces/srv/encos_get_motor_id.hpp"
#include "robot_interfaces/srv/encos_get_motor_param.hpp"
#include "robot_interfaces/srv/encos_set_motor_id.hpp"
#include "robot_interfaces/srv/encos_set_motor_zero.hpp"
#include "robot_interfaces/srv/mix_query_joint.hpp"
#include "robot_interfaces/srv/mix_query_motor_id.hpp"
#include "robot_interfaces/srv/mix_set_motor_id.hpp"
#include "robot_interfaces/srv/mix_set_zero.hpp"
#include "robot_interfaces/srv/tienkung_modify_id.hpp"
#include "robot_interfaces/srv/tienkung_query_id.hpp"
#include "robot_interfaces/srv/tienkung_query_mcu_id.hpp"
#include "robot_interfaces/srv/tienkung_query_status.hpp"
#include "robot_interfaces/srv/tienkung_reset_id.hpp"
#include "robot_interfaces/srv/tienkung_set_zero.hpp"

namespace robot_driver
{

// ROS 2 驱动节点。
//
// 节点负责参数声明、topic 订阅/发布和定时器调度；具体硬件协议访问封装在
// HardwareBackend 中，避免 robot_control 与 EtherCAT/串口等细节耦合。
class RobotDriverNode : public rclcpp::Node
{
public:
  // 构造 ROS 节点，声明参数、创建 backend、订阅控制命令并注册状态发布定时器。
  explicit RobotDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 接收 /control/hardware_commands。
  // 这里只把命令交给 backend；backend 自己决定是缓存还是真实写入。
  void handleHardwareCommand(const robot_interfaces::msg::JointCommands::SharedPtr msg);

  // 高频发布 /joint_states，并推进 backend 的周期状态。
  void publishJointStates();

  // 低频发布 /driver/status，用于诊断和状态机安全判断。
  void publishDriverStatus();

  // Encos 厂家维护 service：只在当前 backend 确认为 EncosBackend 时生效。
  // 具体 EtherCAT 事务和安全门控都在 EncosBackend 内完成，节点只做 ROS 消息转换。
  void handleEncosGetMotorId(
    const std::shared_ptr<robot_interfaces::srv::EncosGetMotorId::Request> request,
    std::shared_ptr<robot_interfaces::srv::EncosGetMotorId::Response> response);
  void handleEncosSetMotorId(
    const std::shared_ptr<robot_interfaces::srv::EncosSetMotorId::Request> request,
    std::shared_ptr<robot_interfaces::srv::EncosSetMotorId::Response> response);
  void handleEncosSetMotorZero(
    const std::shared_ptr<robot_interfaces::srv::EncosSetMotorZero::Request> request,
    std::shared_ptr<robot_interfaces::srv::EncosSetMotorZero::Response> response);
  void handleEncosGetMotorParam(
    const std::shared_ptr<robot_interfaces::srv::EncosGetMotorParam::Request> request,
    std::shared_ptr<robot_interfaces::srv::EncosGetMotorParam::Response> response);

  // Mix CANFD 维护命令 service，全部按 joint_name 定位关节。
  void handleMixQueryJoint(
    const std::shared_ptr<robot_interfaces::srv::MixQueryJoint::Request> request,
    std::shared_ptr<robot_interfaces::srv::MixQueryJoint::Response> response);
  void handleMixSetZero(
    const std::shared_ptr<robot_interfaces::srv::MixSetZero::Request> request,
    std::shared_ptr<robot_interfaces::srv::MixSetZero::Response> response);
  void handleMixQueryMotorId(
    const std::shared_ptr<robot_interfaces::srv::MixQueryMotorId::Request> request,
    std::shared_ptr<robot_interfaces::srv::MixQueryMotorId::Response> response);
  void handleMixSetMotorId(
    const std::shared_ptr<robot_interfaces::srv::MixSetMotorId::Request> request,
    std::shared_ptr<robot_interfaces::srv::MixSetMotorId::Response> response);

  // 天工维护命令 service。
  void handleTienkungQueryStatus(
    const std::shared_ptr<robot_interfaces::srv::TienkungQueryStatus::Request> request,
    std::shared_ptr<robot_interfaces::srv::TienkungQueryStatus::Response> response);
  void handleTienkungSetZero(
    const std::shared_ptr<robot_interfaces::srv::TienkungSetZero::Request> request,
    std::shared_ptr<robot_interfaces::srv::TienkungSetZero::Response> response);
  void handleTienkungModifyID(
    const std::shared_ptr<robot_interfaces::srv::TienkungModifyID::Request> request,
    std::shared_ptr<robot_interfaces::srv::TienkungModifyID::Response> response);
  void handleTienkungQueryMcuId(
    const std::shared_ptr<robot_interfaces::srv::TienkungQueryMcuId::Request> request,
    std::shared_ptr<robot_interfaces::srv::TienkungQueryMcuId::Response> response);
  void handleTienkungQueryID(
    const std::shared_ptr<robot_interfaces::srv::TienkungQueryID::Request> request,
    std::shared_ptr<robot_interfaces::srv::TienkungQueryID::Response> response);
  void handleTienkungResetID(
    const std::shared_ptr<robot_interfaces::srv::TienkungResetID::Request> request,
    std::shared_ptr<robot_interfaces::srv::TienkungResetID::Response> response);

  // 启动参数。
  std::string robot_joints_config_;
  std::string hardware_config_;

  // 配置模式只开放 admin service，不进入正常运控周期。
  bool config_mode_{false};
  double joint_state_pub_hz_{200.0};
  double driver_status_pub_hz_{10.0};

  // backend_ 是唯一硬件状态源；publisher 每个周期从 backend 快照生成 ROS 消息。
  std::unique_ptr<HardwareBackend> backend_;
  // 上游控制命令订阅；回调中只做消息转交，不直接访问硬件。
  rclcpp::Subscription<robot_interfaces::msg::JointCommands>::SharedPtr command_sub_;
  // 统一关节状态发布器。
  rclcpp::Publisher<robot_interfaces::msg::JointStates>::SharedPtr joint_states_pub_;
  // 驱动状态发布器。
  rclcpp::Publisher<robot_interfaces::msg::DriverStatus>::SharedPtr driver_status_pub_;
  // 控制命令回调组，与状态发布和 admin service 分离，避免低频维护命令阻塞控制入口。
  rclcpp::CallbackGroup::SharedPtr control_callback_group_;
  // JointStates 发布回调组。
  rclcpp::CallbackGroup::SharedPtr joint_states_callback_group_;
  // DriverStatus 发布回调组。
  rclcpp::CallbackGroup::SharedPtr driver_status_callback_group_;
  // 厂家维护 service 回调组；允许配置命令与周期状态发布分开调度。
  rclcpp::CallbackGroup::SharedPtr admin_callback_group_;
  // Encos 维护命令 service。
  rclcpp::Service<robot_interfaces::srv::EncosGetMotorId>::SharedPtr encos_get_motor_id_srv_;
  rclcpp::Service<robot_interfaces::srv::EncosSetMotorId>::SharedPtr encos_set_motor_id_srv_;
  rclcpp::Service<robot_interfaces::srv::EncosSetMotorZero>::SharedPtr encos_set_motor_zero_srv_;
  rclcpp::Service<robot_interfaces::srv::EncosGetMotorParam>::SharedPtr encos_get_motor_param_srv_;
  // Mix CANFD 维护命令 service。
  rclcpp::Service<robot_interfaces::srv::MixQueryJoint>::SharedPtr mix_query_joint_srv_;
  rclcpp::Service<robot_interfaces::srv::MixSetZero>::SharedPtr mix_set_zero_srv_;
  rclcpp::Service<robot_interfaces::srv::MixQueryMotorId>::SharedPtr mix_query_motor_id_srv_;
  rclcpp::Service<robot_interfaces::srv::MixSetMotorId>::SharedPtr mix_set_motor_id_srv_;
  // 天工维护命令 service。
  rclcpp::Service<robot_interfaces::srv::TienkungQueryStatus>::SharedPtr tienkung_query_status_srv_;
  rclcpp::Service<robot_interfaces::srv::TienkungSetZero>::SharedPtr tienkung_set_zero_srv_;
  rclcpp::Service<robot_interfaces::srv::TienkungModifyID>::SharedPtr tienkung_modify_id_srv_;
  rclcpp::Service<robot_interfaces::srv::TienkungQueryMcuId>::SharedPtr tienkung_query_mcu_id_srv_;
  rclcpp::Service<robot_interfaces::srv::TienkungQueryID>::SharedPtr tienkung_query_id_srv_;
  rclcpp::Service<robot_interfaces::srv::TienkungResetID>::SharedPtr tienkung_reset_id_srv_;
  // 周期发布定时器。
  rclcpp::TimerBase::SharedPtr joint_states_timer_;
  rclcpp::TimerBase::SharedPtr driver_status_timer_;
};

}  // namespace robot_driver

#endif  // ROBOT_DRIVER__ROBOT_DRIVER_NODE_HPP_
