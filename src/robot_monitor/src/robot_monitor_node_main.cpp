#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_monitor/robot_monitor_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_monitor::RobotMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
