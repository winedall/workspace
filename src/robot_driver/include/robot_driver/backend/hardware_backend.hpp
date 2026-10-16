#ifndef ROBOT_DRIVER__BACKEND__HARDWARE_BACKEND_HPP_
#define ROBOT_DRIVER__BACKEND__HARDWARE_BACKEND_HPP_

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "robot_common/joint_config.hpp"
#include "robot_driver/hardware_config.hpp"
#include "robot_interfaces/msg/driver_status.hpp"
#include "robot_interfaces/msg/joint_commands.hpp"
#include "robot_interfaces/msg/joint_states.hpp"

namespace robot_driver
{

// 创建硬件 backend 所需的统一配置。
//
// 该结构由 RobotDriverNode 根据 ROS 参数填充，再交给 backend_factory。这里不直接
// 保存 rclcpp::Node，目的是让 backend 层可以在 gtest 中独立测试，也避免协议适配层
// 反向依赖 ROS 节点生命周期。
struct BackendConfig
{
  // robot_joints.yaml 路径，用于生成统一 JointStates 顺序和校验关节名称。
  std::string robot_joints_config;
  // 已解析的厂商硬件映射配置；vendor/bus 类型以该配置为唯一来源。
  HardwareConfig hardware_config;
  // Encos 非零 motor_error 需要连续确认的帧数，避免单帧状态字毛刺触发告警。
  int motor_fault_confirm_frames{10};
  // 配置模式。true=仅 admin 服务，不启动 worker 线程；false=正常运控模式。
  bool config_mode{false};
};

// 所有硬件适配后端的统一接口。
//
// RobotDriverNode 只面对该接口，不关心底层是 EtherCAT 还是串口。后续接入
// Encos/Tienkung/Unitree 时，只新增具体 backend，不应把协议细节写回节点或
// robot_control。
class HardwareBackend
{
public:
  // 构造函数。
  HardwareBackend(const BackendConfig & config, std::string backend_name);
  // 析构函数。
  virtual ~HardwareBackend();

  // 返回当前后端名称，主要用于日志和测试诊断。
  virtual std::string backendName() const;
  // 接收 robot_control 输出的统一关节命令。
  virtual void handleCommand(const robot_interfaces::msg::JointCommands & command_msg, double receive_time_sec);
  // 推进后端状态。
  virtual void update(double now_sec);
  // 返回统一关节状态，屏蔽厂商反馈格式、方向、零位、单位换算等差异。
  virtual robot_interfaces::msg::JointStates jointStates() const;
  // 返回统一驱动状态，供 state_manager/command_router 做安全准入判断。
  virtual robot_interfaces::msg::DriverStatus driverStatus() const;

protected:
  // 统一故障诊断上下文。各 backend 只填自己知道的字段，基类负责格式化、
  // 打印并写入 DriverStatus.status_message。
  struct FaultDetail
  {
    std::string source;
    std::string message;
    std::string transport;
    std::string joint_name;
    std::string transport_error;
    std::string protocol_error;
    std::string gateway_error;
    std::string slave_summary;
    int expected_work_counter{-1};
    int actual_work_counter{-1};
    int expected_slave_count{-1};
    int detected_slave_count{-1};
    int slave_id{-1};
    int passage{-1};
    int bus{-1};
    int motor_id{-1};
  };

  // backend 通用运行时关节状态。厂家特有解析最终都落到这些统一字段。
  struct JointRuntime
  {
    // 逻辑关节名，与 robot_joints.yaml 保持一致。
    std::string name;
    // 当前关节对应的硬件厂商；mix 模式下用于区分 Encos/Tienkung。
    std::string vendor;
    // 统一关节空间下的位置、速度、力矩反馈。
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
    // 厂家协议上报的原始电机错误码，最终会映射到 joint_error。
    std::uint16_t motor_error{0};
    // 电机和 MOS 温度反馈。
    float motor_temp{0.0f};
    float mos_temp{0.0f};
    // 当前 backend 认为该关节是否处于使能/有效反馈状态。
    bool enabled{false};
    // 统一关节错误码，供上层状态机和安全策略使用。
    std::uint8_t joint_error{robot_interfaces::msg::JointState::JOINT_ERR_NONE};
    // 对外可读的关节错误说明。
    std::string error_message;
    // 反馈位置越界、命令越界、零点缺失、电机故障等去抖计数。
    int invalid_state_count{0};
    int invalid_cmd_count{0};
    int zero_absence_count{0};
    int motor_fault_count{0};
  };

  // 初始化统一关节运行时数组和索引。
  void initializeJoints();
  // 设置控制周期，单位 ms；非法值回退到 2ms。
  void setCycleTimeMs(double cycle_ms);
  // 根据 joint_name 查询 runtime 下标。
  std::size_t requireJointIndex(const std::string & joint_name) const;
  // 判断关节是否默认参与硬件使能。
  bool jointDefaultEnabled(const std::string & joint_name) const;
  // 设置后端故障短状态；具体 backend 可以封装更细的恢复动作。
  virtual void setFault(const std::string & message);
  // 记录结构化故障上下文，并锁存 backend fault。
  void recordFault(const FaultDetail & detail);
  // 将结构化故障上下文格式化为适合日志和 DriverStatus 的单行文本。
  std::string formatFaultDetail(const FaultDetail & detail) const;
  // 设置对外状态文本。
  void setStatusMessage(const std::string & message);
  // 启动统一 worker。
  void startWorker();
  // 停止统一 worker。
  void stopWorker();
  // worker 主循环。
  void workerLoop();
  // 执行一个统一控制周期。
  bool runCycle(double now_sec);
  // 从缓存命令写入当前通讯输出区。
  bool writeJointCommands();

  // backend 差异点：通讯是否已经可用。
  virtual bool transportReady() const = 0;
  // backend 差异点：执行一次底层数据交换。
  virtual bool exchangeProcessData() = 0;
  // backend 差异点：把单条统一关节命令编码到输出区。
  virtual bool encodeJointCommand(const robot_interfaces::msg::JointCommand & command) = 0;
  // backend 差异点：从输入区刷新关节反馈。
  virtual void readJointFeedback() = 0;
  // backend 差异点：首帧命令到达后的启动流程；默认无动作。
  virtual bool runStartup();
  // backend 差异点：是否已经完成通讯初始化，用于 update() 启动 worker。
  virtual bool configured() const = 0;

  // 节点传入的运行配置，包含配置模式和静态硬件映射。
  BackendConfig backend_config_;
  // 统一关节配置，决定对外 JointStates 顺序和命令限位。
  robot_common::RobotJointsConfig joints_config_;
  // 硬件映射配置，包含厂商、总线、从站、电机 ID 等底层映射。
  HardwareConfig hardware_config_;
  // 运行时关节状态缓存，按 robot_joints.yaml 顺序排列。
  std::vector<JointRuntime> joints_;
  // joint_name 到 joints_ 下标的索引，避免周期中线性查找。
  std::unordered_map<std::string, std::size_t> joint_index_;
  // 最近一批通过关节名和限位校验的运动命令。
  std::vector<robot_interfaces::msg::JointCommand> cached_commands_;
  // JointStates 发布序号。
  mutable std::uint32_t sequence_{0};
  // 最近一次收到的控制命令 trace 信息，用于状态链路追踪。
  std::uint64_t last_command_trace_id_{0};
  std::uint32_t last_command_sequence_{0};
  // 后端故障锁存标志；置位后周期线程停止继续写硬件。
  std::atomic<bool> fault_active_{false};
  // 是否已经收到过有效运动命令；用于触发需要首帧后执行的启动流程。
  std::atomic<bool> has_received_command_{false};
  // 启动流程是否已经完成，避免重复执行电机使能握手。
  std::atomic<bool> startup_done_{false};
  // DriverStatus 中对外展示的短状态文本。
  std::string status_message_;
  // 统一工作线程周期。
  std::chrono::nanoseconds cycle_ns_{std::chrono::milliseconds(2)};
  // 统一后台工作线程，负责周期性执行 runCycle()。
  std::thread worker_;
  // worker 线程运行标志。
  std::atomic<bool> worker_running_{false};
  // 保护 joints_、status_message_ 和 trace 状态。
  mutable std::mutex feedback_mutex_;
  // 保护 cached_commands_。
  mutable std::mutex command_mutex_;
  // 保护低频 admin/config 事务，避免 service 并发交错读写同一底层信箱。
  mutable std::mutex admin_mutex_;
  // 配置模式标志：true=仅提供低频维护服务，不启动运控 worker。
  bool config_mode_{false};
#ifdef WORKER_LOOP_DIAG
  // 当前 worker 周期内 runCycle 分段耗时，仅用于轻量诊断。
  std::uint64_t diag_last_write_us_{0};
  std::uint64_t diag_last_exchange_us_{0};
  std::uint64_t diag_last_read_us_{0};
#endif

private:
  // backend 名称，用于日志、状态消息和工厂识别。
  std::string backend_name_;
};

}  // namespace robot_driver

#endif  // ROBOT_DRIVER__BACKEND__HARDWARE_BACKEND_HPP_
