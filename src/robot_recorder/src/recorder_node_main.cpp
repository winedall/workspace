#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_recorder/recorder_node.hpp"

/// 机器人数据录制节点入口。
/// 初始化 ROS2，创建 RecorderNode 并进入事件循环。
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_recorder::RecorderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
