#include "robot_driver/robot_driver_node.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/qos.hpp"
#include "robot_driver/backend/backend_factory.hpp"
#include "robot_driver/backend/encos_backend.hpp"
#include "robot_driver/backend/mix_backend.hpp"
#include "robot_driver/backend/tienkung_backend.hpp"

namespace robot_driver
{

namespace
{

// 管理事务超时时间。
double adminTimeout(float timeout_sec)
{
  return timeout_sec > 0.0F ? static_cast<double>(timeout_sec) : 0.2;
}

// 格式化管理事务结果。
std::string formatAdminMessage(const EncosAdminResult & result)
{
  if (result.success) {
    return "encos admin transaction completed";
  }
  if (!result.message.empty()) {
    return "encos admin transaction failed: " + result.message;
  }
  return "encos admin transaction failed";
}

}  // namespace

RobotDriverNode::RobotDriverNode(const rclcpp::NodeOptions & options)
: Node("robot_driver_node", options)
{
  // 所有运行参数都通过 ROS parameter 声明，便于 launch/YAML 覆盖。
  robot_joints_config_ = declare_parameter<std::string>("robot_joints_config", "");
  hardware_config_ = declare_parameter<std::string>("hardware_config", "");
  config_mode_ = declare_parameter<bool>("config_mode", false);
  joint_state_pub_hz_ = declare_parameter<double>("joint_state_pub_hz", 200.0);
  driver_status_pub_hz_ = declare_parameter<double>("driver_status_pub_hz", 10.0);
  const auto motor_fault_confirm_frames = declare_parameter<int>("motor_fault_confirm_frames", 10);

  // 配置硬件后端。
  BackendConfig backend_config;
  backend_config.robot_joints_config = robot_joints_config_;
  backend_config.hardware_config = loadHardwareConfig(hardware_config_);
  backend_config.config_mode = config_mode_;
  backend_config.motor_fault_confirm_frames = motor_fault_confirm_frames;
  backend_ = createHardwareBackend(backend_config);
  RCLCPP_INFO(get_logger(), "robot_driver backend selected: %s", backend_->backendName().c_str());
  RCLCPP_INFO(get_logger(), "robot_driver config_mode: %s", config_mode_ ? "true" : "false");
  {
    const auto initial_status = backend_->driverStatus();
    if (initial_status.fault_active) {
      RCLCPP_ERROR(get_logger(), "robot_driver initial status: %s", initial_status.status_message.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "robot_driver initial status: %s", initial_status.status_message.c_str());
    }
  }

  // 创建回调组，用于隔离不同类型的回调，避免互斥执行。
  control_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  joint_states_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  driver_status_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  admin_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // 关节命令 topic 订阅，用于接收控制命令。
  rclcpp::SubscriptionOptions command_options;
  command_options.callback_group = control_callback_group_;
  command_sub_ = create_subscription<robot_interfaces::msg::JointCommands>(
    "control/hardware_commands",
    rclcpp::SensorDataQoS(),
    std::bind(&RobotDriverNode::handleHardwareCommand, this, std::placeholders::_1),
    command_options);

  // 关节状态 topic 发布，用于发布控制反馈。
  joint_states_pub_ = create_publisher<robot_interfaces::msg::JointStates>(
    "driver/joint_states", rclcpp::SensorDataQoS());
    
  // 驱动状态 topic 发布，用于发布驱动状态。
  driver_status_pub_ = create_publisher<robot_interfaces::msg::DriverStatus>(
    "driver/status", rclcpp::SystemDefaultsQoS());

  if (!config_mode_) {
    // 关节状态按控制反馈节拍发布，默认 200 Hz（5 ms）。
    joint_states_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / std::max(joint_state_pub_hz_, 1.0))),
      std::bind(&RobotDriverNode::publishJointStates, this),
      joint_states_callback_group_);
      
    // 驱动状态低频发布，默认 10 Hz，避免把状态 topic 做成高频控制链路。
    driver_status_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / std::max(driver_status_pub_hz_, 1.0))),
      std::bind(&RobotDriverNode::publishDriverStatus, this),
      driver_status_callback_group_);
  } else {
    RCLCPP_INFO(get_logger(), "robot_driver config_mode: periodic joint/status timers disabled");
  }

  // 获取Encos电机 ID 服务。
  encos_get_motor_id_srv_ = create_service<robot_interfaces::srv::EncosGetMotorId>(
    "driver/encos/get_motor_id",
    std::bind(&RobotDriverNode::handleEncosGetMotorId, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    admin_callback_group_);
  // 设置Encos电机 ID 服务。
  encos_set_motor_id_srv_ = create_service<robot_interfaces::srv::EncosSetMotorId>(
    "driver/encos/set_motor_id",
    std::bind(&RobotDriverNode::handleEncosSetMotorId, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    admin_callback_group_);
  // 设置Encos电机零点服务。
  encos_set_motor_zero_srv_ = create_service<robot_interfaces::srv::EncosSetMotorZero>(
    "driver/encos/set_motor_zero",
    std::bind(&RobotDriverNode::handleEncosSetMotorZero, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    admin_callback_group_);
  // 获取Encos电机参数服务。
  encos_get_motor_param_srv_ = create_service<robot_interfaces::srv::EncosGetMotorParam>(
    "driver/encos/get_motor_param",
    std::bind(&RobotDriverNode::handleEncosGetMotorParam, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    admin_callback_group_);
  // 获取Mix关节位置服务。
  mix_query_joint_srv_ = create_service<robot_interfaces::srv::MixQueryJoint>(
    "driver/mix/query_joint",
    std::bind(&RobotDriverNode::handleMixQueryJoint, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    admin_callback_group_);
  // 设置Mix关节零点服务。
  mix_set_zero_srv_ = create_service<robot_interfaces::srv::MixSetZero>(
    "driver/mix/set_zero",
    std::bind(&RobotDriverNode::handleMixSetZero, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    admin_callback_group_);
  // 获取Mix电机 ID 服务。
  mix_query_motor_id_srv_ = create_service<robot_interfaces::srv::MixQueryMotorId>(
    "driver/mix/query_motor_id",
    std::bind(&RobotDriverNode::handleMixQueryMotorId, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    admin_callback_group_);
  // 设置Mix电机 ID 服务。
  mix_set_motor_id_srv_ = create_service<robot_interfaces::srv::MixSetMotorId>(
    "driver/mix/set_motor_id",
    std::bind(&RobotDriverNode::handleMixSetMotorId, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    admin_callback_group_);
  // 获取Tienkung电机状态服务。
  tienkung_query_status_srv_ = create_service<robot_interfaces::srv::TienkungQueryStatus>(
    "driver/tienkung/query_status",
    std::bind(&RobotDriverNode::handleTienkungQueryStatus, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    admin_callback_group_);
  // 设置Tienkung电机零点服务。
  tienkung_set_zero_srv_ = create_service<robot_interfaces::srv::TienkungSetZero>(
    "driver/tienkung/set_zero",
    std::bind(&RobotDriverNode::handleTienkungSetZero, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    admin_callback_group_);
  // 修改Tienkung电机 ID 服务。
  tienkung_modify_id_srv_ = create_service<robot_interfaces::srv::TienkungModifyID>(
    "driver/tienkung/modify_id",
    std::bind(&RobotDriverNode::handleTienkungModifyID, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    admin_callback_group_);
  // 查询Tienkung电机 MCU ID 服务。
  tienkung_query_mcu_id_srv_ = create_service<robot_interfaces::srv::TienkungQueryMcuId>(
    "driver/tienkung/query_mcu_id",
    std::bind(&RobotDriverNode::handleTienkungQueryMcuId, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    admin_callback_group_);
  // 获取Tienkung电机 ID 服务。
  tienkung_query_id_srv_ = create_service<robot_interfaces::srv::TienkungQueryID>(
    "driver/tienkung/query_id",
    std::bind(&RobotDriverNode::handleTienkungQueryID, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    admin_callback_group_);
  // 重置Tienkung电机 ID 服务。
  tienkung_reset_id_srv_ = create_service<robot_interfaces::srv::TienkungResetID>(
    "driver/tienkung/reset_id",
    std::bind(&RobotDriverNode::handleTienkungResetID, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    admin_callback_group_);
}

// 处理硬件命令。
void RobotDriverNode::handleHardwareCommand(const robot_interfaces::msg::JointCommands::SharedPtr msg)
{
  try {
    backend_->handleCommand(*msg, now().seconds());
  } catch (const std::exception & error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected hardware command in robot_driver: %s", error.what());
  }
}

// 发布关节状态。
void RobotDriverNode::publishJointStates()
{
  const auto current_time = now();
  
  // 关节反馈周期是 backend 的主要外部调度入口。
  backend_->update(current_time.seconds());

  // 更新关节状态。
  auto joint_states = backend_->jointStates();
  joint_states.header.stamp = current_time;
  joint_states.header.frame_id = "base_link";
  joint_states_pub_->publish(joint_states);
}

// 发布驱动状态。
void RobotDriverNode::publishDriverStatus()
{
  const auto current_time = now();
  
  // 更新驱动状态。
  auto status = backend_->driverStatus();
  status.header.stamp = current_time;
  status.header.frame_id = "base_link";
  driver_status_pub_->publish(status);
}

// 处理获取电机 ID 服务。
void RobotDriverNode::handleEncosGetMotorId(
  const std::shared_ptr<robot_interfaces::srv::EncosGetMotorId::Request> request,
  std::shared_ptr<robot_interfaces::srv::EncosGetMotorId::Response> response)
{
  // 确认当前 backend 支持 Encos 维护命令。
  auto * encos_backend = dynamic_cast<EncosBackend *>(backend_.get());
  if (encos_backend == nullptr) {
    response->success = false;
    response->message = "active backend does not support Encos admin services";
    return;
  }

  // 调用 EncosBackend 获取电机 ID。
  const auto result = encos_backend->getMotorId(request->joint_name, adminTimeout(request->timeout_sec));
  response->success = result.success;
  response->message = formatAdminMessage(result);
  response->motor_id = result.motor_id;
  response->feedback_code = result.feedback_code;
}

// 处理设置电机 ID 服务。
void RobotDriverNode::handleEncosSetMotorId(
  const std::shared_ptr<robot_interfaces::srv::EncosSetMotorId::Request> request,
  std::shared_ptr<robot_interfaces::srv::EncosSetMotorId::Response> response)
{
  //请求 确认当前 backend 支持 Encos 维护命令。
  auto * encos_backend = dynamic_cast<EncosBackend *>(backend_.get());
  if (encos_backend == nullptr) {
    response->success = false;
    response->message = "active backend does not support Encos admin services";
    return;
  }

  // 调用 EncosBackend 设置电机 ID。
  const auto result = encos_backend->setMotorId(request->joint_name, request->new_motor_id, adminTimeout(request->timeout_sec));
  response->success = result.success;
  response->message = formatAdminMessage(result);
  response->motor_id = result.motor_id;
  response->new_motor_id = result.new_motor_id;
  response->feedback_code = result.feedback_code;
}

// 处理设置零点服务。
void RobotDriverNode::handleEncosSetMotorZero(
  const std::shared_ptr<robot_interfaces::srv::EncosSetMotorZero::Request> request,
  std::shared_ptr<robot_interfaces::srv::EncosSetMotorZero::Response> response)
{
  // 确认当前 backend 支持 Encos 维护命令。
  auto * encos_backend = dynamic_cast<EncosBackend *>(backend_.get());
  if (encos_backend == nullptr) {
    response->success = false;
    response->message = "active backend does not support Encos admin services";
    return;
  }
  
  // 调用 EncosBackend 设置零点。
  const auto result = encos_backend->setMotorZero(request->joint_name, adminTimeout(request->timeout_sec));
  response->success = result.success;
  response->message = formatAdminMessage(result);
  response->motor_id = result.motor_id;
  response->feedback_code = result.feedback_code;
}

// 处理获取电机参数服务。
void RobotDriverNode::handleEncosGetMotorParam(
  const std::shared_ptr<robot_interfaces::srv::EncosGetMotorParam::Request> request,
  std::shared_ptr<robot_interfaces::srv::EncosGetMotorParam::Response> response)
{
  // 确认当前 backend 支持 Encos 维护命令。
  auto * encos_backend = dynamic_cast<EncosBackend *>(backend_.get());
  if (encos_backend == nullptr) {
    response->success = false;
    response->message = "active backend does not support Encos admin services";
    return;
  }
  
  // 调用 EncosBackend 获取电机参数。
  const auto result = encos_backend->getMotorParam(request->joint_name, request->param_code, adminTimeout(request->timeout_sec));
  response->success = result.success;
  response->message = formatAdminMessage(result);
  response->motor_id = result.motor_id;
  response->param_code = result.param_code;
  response->feedback_code = result.feedback_code;
  response->value = result.value;
  response->value_min = result.value_min;
  response->value_max = result.value_max;
}

// 处理 mix 查询关节状态服务。
void RobotDriverNode::handleMixQueryJoint(
  const std::shared_ptr<robot_interfaces::srv::MixQueryJoint::Request> request,
  std::shared_ptr<robot_interfaces::srv::MixQueryJoint::Response> response)
{
  auto * mix_backend = dynamic_cast<MixBackend *>(backend_.get());
  if (mix_backend == nullptr) {
    response->success = false;
    response->message = "active backend does not support Mix admin services";
    return;
  }
  const auto result = mix_backend->queryStatus(request->joint_name, adminTimeout(request->timeout_sec));
  response->success = result.success;
  response->message = result.message;
  response->vendor = result.vendor;
  response->motor_id = result.motor_id;
  response->feedback_code = result.feedback_code;
  response->position = result.position;
  response->velocity = result.velocity;
  response->effort = result.effort;
  response->motor_error = result.motor_error;
  response->motor_temp = result.motor_temp;
  response->mos_temp = result.mos_temp;
}

// 处理 mix 设置零点服务。
void RobotDriverNode::handleMixSetZero(
  const std::shared_ptr<robot_interfaces::srv::MixSetZero::Request> request,
  std::shared_ptr<robot_interfaces::srv::MixSetZero::Response> response)
{
  auto * mix_backend = dynamic_cast<MixBackend *>(backend_.get());
  if (mix_backend == nullptr) {
    response->success = false;
    response->message = "active backend does not support Mix admin services";
    return;
  }
  const auto result = mix_backend->setZero(request->joint_name, adminTimeout(request->timeout_sec));
  response->success = result.success;
  response->message = result.message;
  response->vendor = result.vendor;
  response->motor_id = result.motor_id;
  response->feedback_code = result.feedback_code;
  response->zero_position_rad = result.zero_position_rad;
}

// 处理 mix 查询电机 ID 服务。
void RobotDriverNode::handleMixQueryMotorId(
  const std::shared_ptr<robot_interfaces::srv::MixQueryMotorId::Request> request,
  std::shared_ptr<robot_interfaces::srv::MixQueryMotorId::Response> response)
{
  auto * mix_backend = dynamic_cast<MixBackend *>(backend_.get());
  if (mix_backend == nullptr) {
    response->success = false;
    response->message = "active backend does not support Mix admin services";
    return;
  }
  const auto result = mix_backend->queryID(request->joint_name, adminTimeout(request->timeout_sec));
  response->success = result.success;
  response->message = result.message;
  response->vendor = result.vendor;
  response->motor_id = result.motor_id;
  response->feedback_code = result.feedback_code;
}

// 处理 mix 设置电机 ID 服务。
void RobotDriverNode::handleMixSetMotorId(
  const std::shared_ptr<robot_interfaces::srv::MixSetMotorId::Request> request,
  std::shared_ptr<robot_interfaces::srv::MixSetMotorId::Response> response)
{
  auto * mix_backend = dynamic_cast<MixBackend *>(backend_.get());
  if (mix_backend == nullptr) {
    response->success = false;
    response->message = "active backend does not support Mix admin services";
    return;
  }
  const auto result = mix_backend->modifyID(request->joint_name, request->new_motor_id, adminTimeout(request->timeout_sec));
  response->success = result.success;
  response->message = result.message;
  response->vendor = result.vendor;
  response->motor_id = result.motor_id;
  response->new_motor_id = result.new_motor_id;
  response->feedback_code = result.feedback_code;
}

// 处理天工查询状态服务。
void RobotDriverNode::handleTienkungQueryStatus(
  const std::shared_ptr<robot_interfaces::srv::TienkungQueryStatus::Request> request,
  std::shared_ptr<robot_interfaces::srv::TienkungQueryStatus::Response> response)
{
  auto * tienkung_backend = dynamic_cast<TienkungBackend *>(backend_.get());
  if (tienkung_backend == nullptr) {
    response->success = false;
    response->message = "active backend does not support Tienkung admin services";
    return;
  }
  const auto result = tienkung_backend->queryStatus(request->joint_name, adminTimeout(request->timeout_sec));
  response->success = result.success;
  response->message = result.success ? "tienkung query status completed" : result.message;
  response->motor_id = result.motor_id;
  response->feedback_code = result.feedback_code;
  response->position = result.position;
  response->velocity = result.velocity;
  response->effort = result.effort;
  response->motor_error = result.motor_error;
  response->motor_temp = result.motor_temp;
  response->mos_temp = result.mos_temp;
}

// 处理天工设置零点服务。
void RobotDriverNode::handleTienkungSetZero(
  const std::shared_ptr<robot_interfaces::srv::TienkungSetZero::Request> request,
  std::shared_ptr<robot_interfaces::srv::TienkungSetZero::Response> response)
{
  auto * tienkung_backend = dynamic_cast<TienkungBackend *>(backend_.get());
  if (tienkung_backend == nullptr) {
    response->success = false;
    response->message = "active backend does not support Tienkung admin services";
    return;
  }
  const auto result = tienkung_backend->setZero(request->joint_name, adminTimeout(request->timeout_sec));
  response->success = result.success;
  response->message = result.success ? "tienkung set zero completed" : result.message;
  response->motor_id = result.motor_id;
  response->feedback_code = result.feedback_code;
  response->zero_position_rad = result.zero_position_rad;
}

// 处理天工修改 ID 服务。
void RobotDriverNode::handleTienkungModifyID(
  const std::shared_ptr<robot_interfaces::srv::TienkungModifyID::Request> request,
  std::shared_ptr<robot_interfaces::srv::TienkungModifyID::Response> response)
{
  auto * tienkung_backend = dynamic_cast<TienkungBackend *>(backend_.get());
  if (tienkung_backend == nullptr) {
    response->success = false;
    response->message = "active backend does not support Tienkung admin services";
    return;
  }
  const auto result = tienkung_backend->modifyID(request->joint_name, request->new_motor_id, adminTimeout(request->timeout_sec));
  response->success = result.success;
  response->message = result.success ? "tienkung modify id completed" : result.message;
  response->motor_id = result.motor_id;
  response->new_motor_id = result.new_motor_id;
  response->feedback_code = result.feedback_code;
}

// 处理天工查询 MCU ID 服务。
void RobotDriverNode::handleTienkungQueryMcuId(
  const std::shared_ptr<robot_interfaces::srv::TienkungQueryMcuId::Request> request,
  std::shared_ptr<robot_interfaces::srv::TienkungQueryMcuId::Response> response)
{
  auto * tienkung_backend = dynamic_cast<TienkungBackend *>(backend_.get());
  if (tienkung_backend == nullptr) {
    response->success = false;
    response->message = "active backend does not support Tienkung admin services";
    return;
  }
  const auto result = tienkung_backend->queryMcuId(request->joint_name, adminTimeout(request->timeout_sec));
  response->success = result.success;
  response->message = result.success ? "tienkung query mcu id completed" : result.message;
  response->motor_id = result.motor_id;
  response->resp_can_id = result.resp_can_id;
  response->resp_dlc = result.resp_dlc;
  response->resp_data = result.resp_data;
}

// 处理天工查询 ID 服务。
void RobotDriverNode::handleTienkungQueryID(
  const std::shared_ptr<robot_interfaces::srv::TienkungQueryID::Request> request,
  std::shared_ptr<robot_interfaces::srv::TienkungQueryID::Response> response)
{
  auto * tienkung_backend = dynamic_cast<TienkungBackend *>(backend_.get());
  if (tienkung_backend == nullptr) {
    response->success = false;
    response->message = "active backend does not support Tienkung admin services";
    return;
  }
  const auto result = tienkung_backend->queryID(adminTimeout(request->timeout_sec));
  response->success = result.success;
  response->message = result.success ? "tienkung query id completed" : result.message;
  response->motor_id = result.motor_id;
  response->feedback_code = result.feedback_code;
}

// 处理天工重置 ID 服务。
void RobotDriverNode::handleTienkungResetID(
  const std::shared_ptr<robot_interfaces::srv::TienkungResetID::Request> request,
  std::shared_ptr<robot_interfaces::srv::TienkungResetID::Response> response)
{
  auto * tienkung_backend = dynamic_cast<TienkungBackend *>(backend_.get());
  if (tienkung_backend == nullptr) {
    response->success = false;
    response->message = "active backend does not support Tienkung admin services";
    return;
  }
  const auto result = tienkung_backend->resetID(adminTimeout(request->timeout_sec));
  response->success = result.success;
  response->message = result.success ? "tienkung reset id completed" : result.message;
  response->feedback_code = result.feedback_code;
}

}  // namespace robot_driver
