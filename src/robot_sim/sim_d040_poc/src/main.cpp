#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "sim_d040_poc/SimD040Poc.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sim_d040_poc::SimD040Poc>();
  node->Start();
  node->Run();
  rclcpp::shutdown();
  return 0;
}
