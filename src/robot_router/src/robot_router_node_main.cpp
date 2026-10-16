#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_router/robot_router_node.hpp"

int main(int argc, char ** argv)
{
  // 节点入口保持最小化，便于 CommandRouter 和节点封装分别测试。
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_router::RobotRouterNode>());
  rclcpp::shutdown();
  return 0;
}
