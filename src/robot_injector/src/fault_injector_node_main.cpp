// Copyright 2024 Zoomlion. All rights reserved.
//
// 故障注入节点主入口
// 启动故障注入节点并进入ROS 2事件循环

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "robot_injector/fault_injector_node.hpp"

int main(int argc, char * argv[])
{
  // 初始化ROS 2
  rclcpp::init(argc, argv);

  // 创建故障注入节点
  auto node = std::make_shared<robot_injector::FaultInjectorNode>();

  // 进入事件循环，处理回调
  rclcpp::spin(node);

  // 关闭ROS 2
  rclcpp::shutdown();
  return 0;
}
