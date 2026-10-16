#ifndef ROBOT_BATTERY__ROBOT_BATTERY_NODE_HPP_
#define ROBOT_BATTERY__ROBOT_BATTERY_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robot_battery/battery_parser.hpp"
#include "robot_battery/socket_can.hpp"
#include "robot_interfaces/msg/battery_info.hpp"

namespace robot_battery
{

// BMS 电池读取节点：通过 SocketCAN 查询电池并发布 battery/info。
class RobotBatteryNode : public rclcpp::Node
{
public:
  explicit RobotBatteryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 周期发送 BMS 查询帧，并在设备未打开时尝试重新打开。
  void onPoll();

  // 周期读取 CAN 回包，并交给解析器更新电池状态。
  void onRead();

  // 周期发布最近一次有效电池状态。
  void onPublish();

  // 尝试打开 SocketCAN 设备。
  bool openCan();

  // 发送 BMS 查询帧，触发电池模块回传状态。
  void sendQuery();

  std::string device_{"can0"};
  double poll_rate_hz_{10.0};
  double publish_rate_hz_{10.0};

  SocketCan can_;
  BatteryParser parser_;

  rclcpp::Publisher<robot_interfaces::msg::BatteryInfo>::SharedPtr battery_pub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace robot_battery

#endif  // ROBOT_BATTERY__ROBOT_BATTERY_NODE_HPP_
