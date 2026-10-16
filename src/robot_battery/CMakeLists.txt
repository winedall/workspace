cmake_minimum_required(VERSION 3.10)
project(robot_battery)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)

add_library(${PROJECT_NAME}
  src/battery_parser.cpp
  src/socket_can.cpp
  src/robot_battery_node.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  robot_interfaces
)

add_executable(robot_battery_node src/robot_battery_node_main.cpp)
target_link_libraries(robot_battery_node ${PROJECT_NAME})
ament_target_dependencies(robot_battery_node rclcpp)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} robot_battery_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_battery_parser
    test/test_battery_parser.cpp
  )
  target_link_libraries(test_battery_parser ${PROJECT_NAME})
  ament_target_dependencies(test_battery_parser robot_interfaces)
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp robot_interfaces)
ament_package()
