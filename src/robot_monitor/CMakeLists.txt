cmake_minimum_required(VERSION 3.10)
project(robot_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_common REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(usbcan_bridge_interface REQUIRED)
find_package(std_srvs REQUIRED)

add_library(${PROJECT_NAME}
  src/system_monitor.cpp
  src/audio_manager.cpp
  src/robot_monitor_node.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp robot_common robot_interfaces usbcan_bridge_interface std_srvs
)

add_executable(robot_monitor_node src/robot_monitor_node_main.cpp)
target_link_libraries(robot_monitor_node ${PROJECT_NAME})
ament_target_dependencies(robot_monitor_node rclcpp)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS robot_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp robot_common robot_interfaces usbcan_bridge_interface std_srvs)
ament_package()
