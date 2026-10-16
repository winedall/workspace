#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "sim_d015_poc/SimD015Poc.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sim_d015_poc::SimD015Poc>();
  node->Start();
  node->Run();
  rclcpp::shutdown();
  return 0;
}
