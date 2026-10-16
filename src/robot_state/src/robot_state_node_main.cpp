#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_state/robot_state_node.hpp"

int main(int argc, char ** argv)
{
  // 可执行入口保持最小化，便于把业务逻辑集中在 Node 和 StateMachine 中测试。
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_state::RobotStateNode>());
  rclcpp::shutdown();
  return 0;
}
