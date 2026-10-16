cmake_minimum_required(VERSION 3.10)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(robot_common REQUIRED)
find_package(robot_interfaces REQUIRED)

add_library(${PROJECT_NAME}
  src/ankle_command_converter.cpp
  src/joint_command_filter.cpp
  src/robot_control_node.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  robot_common
  robot_interfaces
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

add_executable(robot_control_node src/robot_control_node_main.cpp)
target_link_libraries(robot_control_node ${PROJECT_NAME})
ament_target_dependencies(robot_control_node rclcpp)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME} robot_control_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp robot_common robot_interfaces)
ament_package()
