#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "robot_driver/robot_driver_node.hpp"

int main(int argc, char ** argv)
{
  // 可执行入口只负责 ROS 初始化和 spin，业务逻辑保持在 RobotDriverNode /
  // HardwareBackend 中，便于测试和后续替换真实硬件 backend。
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_driver::RobotDriverNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
