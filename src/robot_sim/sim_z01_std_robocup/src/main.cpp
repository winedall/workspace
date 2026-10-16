#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "sim_z01_std_robocup/SimZ01StdRobocup.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // Run() 会启动 MuJoCo 渲染循环和物理线程；节点内部另起 ROS executor 线程
  // （1 个主节点 + 6 个机器人节点共用一个 executor）。
  auto node = std::make_shared<sim_z01_std_robocup::SimZ01StdRobocup>();
  // Start() 必须在 shared_ptr 就绪后调用，避免构造期内 shared_from_this 失效。
  node->Start();
  node->Run();
  return 0;
}
