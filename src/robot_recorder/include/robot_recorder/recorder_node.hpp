#ifndef ROBOT_RECORDER__RECORDER_NODE_HPP_
#define ROBOT_RECORDER__RECORDER_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/driver_status.hpp"
#include "robot_interfaces/msg/joint_commands.hpp"
#include "robot_interfaces/msg/joint_states.hpp"
#include "robot_interfaces/msg/robot_state.hpp"
#include "yesense_interface/msg/nav_all.hpp"

#include "robot_recorder/csv_writer.hpp"
#include "robot_recorder/event_manager.hpp"
#include "robot_recorder/joint_record_sample.hpp"
#include "robot_recorder/ring_buffer.hpp"

namespace robot_recorder
{

/// recorder 节点：常态记录关节详细数据 + IMU，事件时记录故障元数据
///
/// 数据流：
///   onJointStates → states_ring_ (JointStatesRecord) → writerLoop → states_csv_
///   onJointCommands → commands/desired/accepted_ring_ (JointCommandsRecord) → writerLoop → 对应 CSV
///   onNavAll → latest_imu_ (缓存)
///   onRobotState → EventManager → events_csv_（故障元数据，含 sequence 定位）
class RecorderNode : public rclcpp::Node
{
public:
  explicit RecorderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RecorderNode();

private:
  // ---------- ROS 回调 ----------
  void onJointCommands(const robot_interfaces::msg::JointCommands::SharedPtr msg, const std::string & topic);
  void onJointStates(const robot_interfaces::msg::JointStates::SharedPtr msg);
  void onNavAll(const yesense_interface::msg::NavAll::SharedPtr msg);
  void onRobotState(const robot_interfaces::msg::RobotState::SharedPtr msg);
  void onDriverStatus(const robot_interfaces::msg::DriverStatus::SharedPtr msg);

  // ---------- 工具函数 ----------
  /// 确保目录存在；用于启动时创建输出根目录，也用于事件触发时创建 event_xxxx 子目录。
  bool ensureDir(const std::string & path) const;

  // ---------- Writer 线程 ----------
  /// 后台写线程主循环。
  /// continuous 模式周期性落盘；event_window 模式只在事件窗口打开时写出窗口缓存。
  void writerLoop();

  /// 通知写线程退出并等待线程结束，析构时调用。
  void stopWriter();

  // ---------- CSV 写入 ----------
  /// 将命令宽表记录批量写入指定 CSV。
  void writeCommands(
    const std::vector<JointCommandsRecord> & batch,
    std::unique_ptr<CsvWriter> & csv);

  /// 将状态宽表记录批量写入指定 CSV，并返回实际写入的状态记录数。
  size_t writeStates(
    const std::vector<JointStatesRecord> & batch,
    std::unique_ptr<CsvWriter> & csv);

  /// 从 EventManager 取出已完成的事件会话，并按当前 recording_mode 写入元数据或启动窗口录制。
  void handleEvents();

  /// 将单条事件元数据写入当前 events.csv。
  void writeEvent(const EventSession & session);

  /// flush 当前事件/概要 CSV；CsvWriter 生命周期仍由对应 unique_ptr 管理。
  void flushEvents();

  /// event_window 模式下创建 event_xxxx 目录，初始化 CSV，并先写出故障前窗口。
  void startEventRecording(const EventSession & session);

  /// event_window 模式下结束当前事件窗口，写出尾部缓存并关闭事件 CSV。
  void stopEventRecording();

  /// 将 TimeWindowBuffer 中的数据 drain 到当前事件 CSV。
  /// 只在事件窗口打开时调用，因此平时高频数据不会持续落盘。
  void drainWindowBuffersToCsv();

  // ---------- CSV 初始化 ----------
  /// continuous 模式下启动时创建启用的高频 CSV。
  void initContinuousCsvFiles();

  /// 在指定目录创建状态 CSV；continuous 使用 output_dir，event_window 使用 event_xxxx 目录。
  void initStatesCsvForDir(
    const std::string & dir,
    std::unique_ptr<CsvWriter> & csv);

  /// 在指定目录创建命令 CSV；suffix 决定 desired/accepted/hardware 三类文件名。
  void initCommandsCsvForDir(
    const std::string & dir,
    const std::string & suffix,
    std::unique_ptr<CsvWriter> & csv);

  /// 在指定目录创建 events.csv。
  void initEventsCsvForDir(const std::string & dir);

  // ========== 成员变量 ==========

  // ---------- 参数 ----------
  std::string output_dir_;
  size_t buffer_capacity_;
  double flush_interval_ms_;
  std::string recording_mode_;
  double pre_event_window_sec_;
  double post_event_window_sec_;
  double event_cooldown_sec_{2.0};
  std::string joints_config_path_;

  // ---------- 常态详细缓冲区（按 topic 独立） ----------
  RingBuffer<JointStatesRecord> states_ring_;
  RingBuffer<JointCommandsRecord> commands_ring_;
  RingBuffer<JointCommandsRecord> desired_ring_;
  RingBuffer<JointCommandsRecord> accepted_ring_;

  // ---------- 事件窗口缓冲区（event_window 模式） ----------
  TimeWindowBuffer<JointStatesRecord> states_window_;
  TimeWindowBuffer<JointCommandsRecord> commands_window_;
  TimeWindowBuffer<JointCommandsRecord> desired_window_;
  TimeWindowBuffer<JointCommandsRecord> accepted_window_;

  // ---------- CSV 写入器 ----------
  std::unique_ptr<CsvWriter> desired_csv_;
  std::unique_ptr<CsvWriter> accepted_csv_;
  std::unique_ptr<CsvWriter> commands_csv_;
  std::unique_ptr<CsvWriter> states_csv_;

  // ---------- 线程 ----------
  std::thread writer_thread_;
  std::atomic<bool> running_{true};

  // ---------- 统计 ----------
  std::atomic<size_t> written_records_{0};
  std::atomic<size_t> write_errors_{0};

  // ---------- 订阅 ----------
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;

  // ---------- 记录开关 ----------
  bool record_states_ = true;
  bool record_commands_ = true;
  bool record_desired_ = true;
  bool record_accepted_ = true;
  bool record_imu_ = true;

  // ---------- Topic 名称 ----------
  std::string states_topic_;
  std::string commands_topic_;
  std::string desired_topic_;
  std::string accepted_topic_;
  std::string imu_topic_;

  // ---------- 事件记录 ----------
  EventManager event_mgr_;
  std::unique_ptr<CsvWriter> events_csv_;
  bool event_active_{false};
  int64_t event_end_ms_{0};

  // ---------- 关节名称（由 robot_joints_config 固定） ----------
  std::vector<std::string> joint_names_;

  // ---------- IMU 缓存 ----------
  ImuSample latest_imu_;

  // ---------- sequence 缓存（用于事件定位） ----------
  uint32_t latest_joint_states_seq_{0};
  uint32_t latest_joint_commands_seq_{0};
  std::string latest_driver_msg_;

};

}  // namespace robot_recorder

#endif  // ROBOT_RECORDER__RECORDER_NODE_HPP_
