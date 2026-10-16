#include "robot_battery/robot_battery_node.hpp"

#include <algorithm>
#include <chrono>

namespace robot_battery
{

namespace
{
constexpr uint32_t kBatteryQueryId = 0x0400FF80U;
constexpr int kReadTimeoutMs = 0;
constexpr int kMaxReadFramesPerTick = 20;
}  // namespace

// 初始化参数、发布器和定时器，并尝试打开电池 CAN 设备。
RobotBatteryNode::RobotBatteryNode(const rclcpp::NodeOptions & options)
: Node("robot_battery_node", options)
{
  device_ = declare_parameter<std::string>("device", "can0");
  poll_rate_hz_ = declare_parameter<double>("poll_rate_hz", 10.0);
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);

  battery_pub_ = create_publisher<robot_interfaces::msg::BatteryInfo>(
    "battery/info", rclcpp::SystemDefaultsQoS());

  const auto poll_period = std::chrono::duration<double>(1.0 / std::max(poll_rate_hz_, 0.1));
  poll_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(poll_period),
    std::bind(&RobotBatteryNode::onPoll, this));

  read_timer_ = create_wall_timer(
    std::chrono::milliseconds(5),
    std::bind(&RobotBatteryNode::onRead, this));

  const auto publish_period = std::chrono::duration<double>(1.0 / std::max(publish_rate_hz_, 0.1));
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
    std::bind(&RobotBatteryNode::onPublish, this));

  openCan();
}

// 周期查询 BMS；如果 CAN 设备暂时不可用，则在该周期尝试重新打开。
void RobotBatteryNode::onPoll()
{
  if (!can_.isOpen() && !openCan()) {
    return;
  }
  sendQuery();
}

// 从 CAN socket 中尽量读完当前积压帧，并更新解析器缓存。
void RobotBatteryNode::onRead()
{
  if (!can_.isOpen()) {
    return;
  }

  for (int i = 0; i < kMaxReadFramesPerTick; ++i) {
    CanFrame frame;
    std::string error;
    if (!can_.readFrame(frame, kReadTimeoutMs, error)) {
      if (!error.empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "读取电池 CAN 数据失败: %s", error.c_str());
        can_.close();
      }
      return;
    }

    parser_.parse(frame.id, frame.data.data(), frame.size);
  }
}

// 有效收到总信息帧后，周期发布最近一次电池状态。
void RobotBatteryNode::onPublish()
{
  if (parser_.hasSummary()) {
    battery_pub_->publish(parser_.message());
  }
}

// 打开 SocketCAN 设备，失败时节流打印错误日志。
bool RobotBatteryNode::openCan()
{
  std::string error;
  if (!can_.open(device_, error)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "打开电池 CAN 设备 %s 失败: %s", device_.c_str(), error.c_str());
    return false;
  }

  RCLCPP_INFO(get_logger(), "电池 CAN 设备已打开: %s", device_.c_str());
  return true;
}

// 发送 BMS 查询扩展帧，触发电池模块回传状态帧。
void RobotBatteryNode::sendQuery()
{
  CanFrame frame;
  frame.id = kBatteryQueryId;
  frame.extended = true;
  frame.size = 8;

  std::string error;
  if (!can_.writeFrame(frame, error)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "发送电池查询帧失败: %s", error.c_str());
    can_.close();
  }
}

}  // namespace robot_battery
