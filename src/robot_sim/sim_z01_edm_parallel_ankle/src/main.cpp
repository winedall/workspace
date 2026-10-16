#include "sim_z01_edm_parallel_ankle/SimZ01EdmParallelAnkle.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // SimZ01EdmParallelAnkle::Run() 会启动 MuJoCo 渲染循环和物理线程；
  // 节点内部另起 ROS executor 线程处理 topic。
  auto node = std::make_shared<SimZ01EdmParallelAnkle>();
  // Start() 在 shared_ptr 就绪后启动 ROS executor 线程，避免构造期内
  // shared_from_this() 触发 bad_weak_ptr。
  node->Start();
  node->Run();
  return 0;
}
