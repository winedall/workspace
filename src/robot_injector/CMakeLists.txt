# Copyright 2024 Zoomlion. All rights reserved.
#
# recorder 事件注入工具构建配置

cmake_minimum_required(VERSION 3.10)
project(robot_injector)

# 编译器警告选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)

# =========================================================================
# 构建库（包含所有业务逻辑）
# =========================================================================
add_library(${PROJECT_NAME}
  src/fault_injector_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} rclcpp robot_interfaces)

# =========================================================================
# 构建可执行文件
# =========================================================================
add_executable(fault_injector_node src/fault_injector_node_main.cpp)
target_link_libraries(fault_injector_node ${PROJECT_NAME})
ament_target_dependencies(fault_injector_node rclcpp)

# =========================================================================
# 安装目标文件
# =========================================================================
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} fault_injector_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# =========================================================================
# 安装配置、launch和脚本文件
# =========================================================================
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
# 脚本安装到lib目录，使ros2 run可以找到
install(PROGRAMS scripts/inject_fault.sh
  DESTINATION lib/${PROJECT_NAME})

# =========================================================================
# 测试配置
# =========================================================================
# =========================================================================
# 导出包信息
# =========================================================================
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp robot_interfaces)
ament_package()
