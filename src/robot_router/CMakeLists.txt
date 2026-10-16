cmake_minimum_required(VERSION 3.10)
project(robot_router)

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
  src/command_router.cpp
  src/robot_router_node.cpp
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

add_executable(robot_router_node src/robot_router_node_main.cpp)
target_link_libraries(robot_router_node ${PROJECT_NAME})
ament_target_dependencies(robot_router_node rclcpp)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME} robot_router_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp robot_common robot_interfaces)
ament_package()
