#include "sim_z01_std_robocup/SimRosExecutor.h"

#include <chrono>
#include <utility>

#include <pthread.h>

namespace sim_z01_std_robocup {

SimRosExecutor::SimRosExecutor(std::function<void()> on_shutdown)
    : on_shutdown_(std::move(on_shutdown)) {}

SimRosExecutor::~SimRosExecutor() { Stop(); }

void SimRosExecutor::AddNode(const rclcpp::Node::SharedPtr& node) {
  if (running_) {
    return;  // Start 之后不再接收新节点
  }
  nodes_.push_back(node);
}

void SimRosExecutor::Start() {
  if (running_) {
    return;
  }
  running_ = true;
  spin_thread_ = std::thread(&SimRosExecutor::SpinLoop, this);
  pthread_setname_np(spin_thread_.native_handle(), "sim_ros_exec");
}

void SimRosExecutor::Stop() {
  running_ = false;
  executor_.cancel();
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

void SimRosExecutor::SpinLoop() {
  for (const auto& node : nodes_) {
    executor_.add_node(node);
  }
  while (running_ && rclcpp::ok()) {
    // spin_some 处理所有就绪回调后返回；没有工作时最多等待 500µs。
    executor_.spin_some(std::chrono::microseconds(500));
  }
  // ROS 2 收到 SIGINT/SIGTERM 后同步通知 MuJoCo GUI/物理循环退出，
  // 否则 RenderLoop 会继续阻塞，launch 最终只能 SIGKILL 仿真进程。
  if (on_shutdown_) {
    on_shutdown_();
  }
  for (const auto& node : nodes_) {
    executor_.remove_node(node);
  }
}

}  // namespace sim_z01_std_robocup
