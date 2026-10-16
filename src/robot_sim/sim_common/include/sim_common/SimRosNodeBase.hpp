#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <boost/lockfree/spsc_queue.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/driver_status.hpp>
#include <robot_interfaces/msg/joint_commands.hpp>
#include <robot_interfaces/msg/joint_states.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yesense_interface/msg/nav_all.hpp>

#include "sim_common/SimErrorModel.hpp"
#include "sim_common/SimJointCommand.hpp"

namespace sim_common {

// JointStates 消息中固定名称的零状态占位关节（模型内没有对应执行器）。
struct PlaceholderJointState {
  std::size_t msg_index{0};  // joint_states_msg_.states 中的下标
  std::string joint_name;    // 例如 "waist_pitch"
};

// 把 n 个仿真关节（控制名顺序）映射到对外发布的 JointStates 消息下标。
struct JointStatesLayout {
  // 长度 = 仿真关节数 n；-1 表示该关节不对外发布。
  std::vector<int> sim_to_msg_index;
  // 固定名称的占位条目，发布零数据 + JOINT_ERR_NONE。
  std::vector<PlaceholderJointState> placeholders;
};

// 一组"未建模关节"（真机存在、MuJoCo 模型没有对应执行器）：
// 发布时作为零状态占位，插入到锚点关节之前。
struct PlaceholderGroup {
  std::string before;                    // 锚点：插在该仿真关节之前；空 = 插在最前
  std::vector<std::string> joint_names;  // 未建模关节名，按真机顺序
};

// 插在某个仿真关节之前的一组占位关节。
inline PlaceholderGroup PlaceholdersBefore(std::string anchor,
                                           std::vector<std::string> joint_names) {
  return PlaceholderGroup{std::move(anchor), std::move(joint_names)};
}

// 插在发布顺序最前的一组占位关节（真机以头部关节开头时使用）。
inline PlaceholderGroup PlaceholdersFirst(std::vector<std::string> joint_names) {
  return PlaceholderGroup{"", std::move(joint_names)};
}

// 由"仿真关节列表 + 占位组"推导发布布局，避免手写下标偏移：
// 发布顺序 = 仿真关节按控制名顺序排列，并在各锚点前插入对应占位关节。
// 例如：
//   BuildJointStatesLayout(names, {PlaceholdersFirst({"head_yaw", "head_pitch"})})
//   → [head_yaw, head_pitch, names[0], names[1], ...]
//
// 不传占位组时得到恒等映射（狗形包：发布的关节与仿真关节一一对应）。
// 锚点不在仿真关节中时抛 std::invalid_argument（属配置错误，尽早暴露）。
JointStatesLayout BuildJointStatesLayout(
    const std::vector<std::string>& simulated_joint_names,
    const std::vector<PlaceholderGroup>& placeholder_groups = {});

struct SimRosNodeOptions {
  std::string node_name;                 // rclcpp 节点名
  std::string topic_prefix{"/robot"};    // 话题前缀（末尾不带 '/'），多机仿真用 "/robot_0X"
  std::vector<std::string> control_joint_names;  // 控制系统统一 joint_name，仿真关节顺序
  JointStatesLayout joint_states_layout;
  std::string driver_status_message;     // 例如 "mujoco simulation online (parallel leg)"
  // 相机消息的 frame_id（多机仿真按机器人区分，避免 TF 上同名冲突）
  std::string camera_frame_id{"depth_cam_optical_frame"};
  // true: state.enabled = enable_control_ && cmd.enable（狗形包语义）
  // false: state.enabled = cmd.enable（人形包语义）
  bool joint_enabled_requires_enable_control{false};
};

// SimRosNodeBase 只负责 ROS 2 通信层：
// 1. 接收 /robot/control/hardware_commands；
// 2. 发布 /robot/driver/joint_states；
// 3. 发布 /robot/driver/status（5 Hz 定时器）；
// 4. 将统一 JointCommands 按 joint_name 转换为仿真内部缓存。
//
// 注意：这个类不直接访问真实硬件，也不做上层控制决策。最终位置/速度/
// 力矩限幅仍属于 robot_control，仿真侧只做 MuJoCo 执行器输入转换。
class SimRosNodeBase : public rclcpp::Node {
 public:
  SimRosNodeBase(const SimRosNodeOptions& opts,
                 const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
  ~SimRosNodeBase() override = default;

  // 误差注入模型：由物理层在模型加载后注入（与 MjInterface::SetErrorModel 同样式）。
  // 通信层需要它来施加 errors.command_delay_ms 的指令延迟——UpdateCommandCache 是
  // 所有仿真包唯一共用的指令写入点，放那里才能一处覆盖全部包。
  // 公开是因为多机路径要从外部对 ros_ 调用（RobotChannel 不是本类的派生类）。
  void SetErrorModel(SimErrorModel* model) { error_model_ = model; }

 protected:
  void PublishJointStates();
  void PublishDriverStatus();
  // 从命令队列取出"最新一帧"：排空整个队列，返回最后入队的一帧。
  // 命令发布频率可能高于仿真消费频率，FIFO 逐帧消费会让仿真执行陈旧目标；
  // 每步只取最新帧并丢弃中间帧，保证缓存始终是最新控制意图。
  bool TryTakeLatestCommand(robot_interfaces::msg::JointCommands& out);
  // 单帧 FIFO 出队，供需要逐帧消费的控制器使用（狗形仿真路径）。
  bool PopCommand(robot_interfaces::msg::JointCommands& out);
  // 将一帧 JointCommands 合并进"最新指令"缓存，并把 errors.command_delay_ms 之前的
  // 那一帧写进 sim_joint_commands_（供 ComputeControl 读取）。未知关节会被忽略。
  // time 取仿真时间（d->time）：延迟线按它索引，与物理步长解耦。
  void UpdateCommandCache(const robot_interfaces::msg::JointCommands& msg, double time);
  int GetSimJointIndex(const std::string& joint_name) const;
  const std::vector<std::string>& GetSimJointNames() const { return sim_joint_names_; }

  // enable_control_ 表示仿真执行器是否接收控制输入。即使为 true，也只写入
  // MuJoCo 的 d->ctrl，不会触发真实硬件通信。
  bool enable_control_{true};
  // 可调参数（来自各仿真包自己的 config YAML）：
  // driver_status_period_ms_ — DriverStatus 定时发布周期；
  // camera_period_us_ — 深度相机发布周期，0 表示不创建相机话题也不发布。
  std::int64_t driver_status_period_ms_{200};
  std::int64_t camera_period_us_{0};
  // sim_joint_commands_ 的顺序严格对应仿真关节（与 control_joint_names 一致）。
  // 它是**延迟后**的参考（ComputeControl 读这个）；sim_joint_commands_fresh_ 保存未经
  // 延迟的最新参考，供"一帧只含部分关节时保持其他关节上次值"的合并逻辑使用——
  // 若直接合并进延迟后的缓存，中间几帧的新值会被丢掉。
  std::vector<SimJointCommand> sim_joint_commands_;
  std::vector<SimJointCommand> sim_joint_commands_fresh_;
  SimErrorModel* error_model_{nullptr};
  robot_interfaces::msg::JointStates joint_states_msg_;
  robot_interfaces::msg::DriverStatus driver_status_msg_;
  JointStatesLayout joint_states_layout_;
  bool joint_enabled_requires_enable_control_{false};
  // 话题前缀（构造时校验：以 '/' 开头、不以 '/' 结尾）与相机 frame_id。
  std::string topic_prefix_;
  std::string camera_frame_id_;

  // ROS subscription callback 和 MuJoCo 控制回调运行在不同线程中。
  // 使用单生产者单消费者队列把 ROS 命令安全传给仿真控制循环。
  boost::lockfree::spsc_queue<robot_interfaces::msg::JointCommands,
                              boost::lockfree::capacity<10>> cmd_queue_;

  // ---- 物理量话题：发布动作与消息对象归通信层，数值由物理层（SimNodeBase）注入。
  // imu_data_ 的字段由物理层填充后调用 PublishImu 发出；
  // 相机话题由物理层传入渲染结果后由 PublishImages 序列化并发布。
  yesense_interface::msg::NavAll imu_data_;
  void PublishImu();
  void PublishImages(const std::vector<unsigned char>& rgb, const std::vector<float>& depth,
                     int width, int height);
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  bool camera_info_ready_{false};

 private:
  void LoadParam();
  void InitCommandCache();
  void InitRosInterfaces();
  void HandleHardwareCommands(const robot_interfaces::msg::JointCommands& msg);

  // sim_joint_names_ 使用控制系统统一 joint_name，而不是 MuJoCo XML 内部名字。
  // 这样 robot_control/robot_rl_manager 仍然只需要面对 robot_interfaces 的统一接口。
  std::vector<std::string> sim_joint_names_;
  std::string driver_status_message_;
  rclcpp::Subscription<robot_interfaces::msg::JointCommands>::SharedPtr sub_hardware_commands_;
  rclcpp::Publisher<robot_interfaces::msg::JointStates>::SharedPtr pub_joint_states_;
  rclcpp::Publisher<robot_interfaces::msg::DriverStatus>::SharedPtr pub_driver_status_;
  rclcpp::Publisher<yesense_interface::msg::NavAll>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
  rclcpp::TimerBase::SharedPtr driver_status_timer_;
  uint32_t joint_state_sequence_{0};
  uint64_t last_command_trace_id_{0};
  uint32_t last_command_sequence_{0};
};

}  // namespace sim_common
