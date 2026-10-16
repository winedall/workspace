#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_control/robot_control_node.hpp"

int main(int argc, char ** argv)
{
  // 节点入口保持最小化，业务逻辑放在 RobotControlNode 和纯逻辑类中。
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_control::RobotControlNode>());
  rclcpp::shutdown();
  return 0;
}
