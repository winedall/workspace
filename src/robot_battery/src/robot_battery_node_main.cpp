#include "rclcpp/rclcpp.hpp"
#include "robot_battery/robot_battery_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_battery::RobotBatteryNode>());
  rclcpp::shutdown();
  return 0;
}
