#ifndef SIMZ01STDROBOCUP_SIMROSEXECUTOR_H
#define SIMZ01STDROBOCUP_SIMROSEXECUTOR_H

#include <atomic>
#include <functional>
#include <thread>
#include <vector>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

namespace sim_z01_std_robocup {

// 多机器人仿真有 7 个 rclcpp::Node（1 个主节点 + 6 台机器人），共用一个
// SingleThreadedExecutor 和一条 spin 线程。
//
// 与单机器人仿真（SimNodeBase::RunRosExecutor 用 spin_once(1ms)）的差异：
// 6 台 × 200Hz 命令 ≈ 1200 msg/s，spin_once 每毫秒只处理一个回调会积压，
// 这里用 spin_some() 把"当前就绪的回调"一次处理完（没有工作时最多等 500µs）。
class SimRosExecutor {
 public:
  explicit SimRosExecutor(std::function<void()> on_shutdown = {});
  ~SimRosExecutor();

  // 必须在 Start() 之前调用。
  void AddNode(const rclcpp::Node::SharedPtr& node);
  // 起 spin 线程。
  void Start();
  // 停止 spin 线程并在退出前执行 on_shutdown（用于置 sim_->exitrequest）。
  void Stop();

 private:
  void SpinLoop();

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::vector<rclcpp::Node::SharedPtr> nodes_;
  std::function<void()> on_shutdown_;
  std::thread spin_thread_;
  std::atomic<bool> running_{false};
};

}  // namespace sim_z01_std_robocup

#endif  // SIMZ01STDROBOCUP_SIMROSEXECUTOR_H
