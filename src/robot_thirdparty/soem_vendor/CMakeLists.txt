cmake_minimum_required(VERSION 3.10)
project(soem_vendor)

find_package(ament_cmake REQUIRED)

# 仅把 SOEM 源码作为第三方库构建并导出。
# 当前阶段 robot_driver 还没有链接或调用 SOEM，真实 EtherCAT backend 后续再显式接入。
add_subdirectory(soem)

# 导出 SOEM 的头文件目录和库目标，供后续 encos/tienkung EtherCAT backend 使用。
ament_export_include_directories(include/soem)
ament_export_libraries(soem)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
