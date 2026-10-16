cmake_minimum_required(VERSION 3.16)
project(sim_d015_poc VERSION 0.1.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(mj_sim REQUIRED)
find_package(sim_common REQUIRED)

add_executable(sim_d015_poc
  src/main.cpp
  src/sim_d015_poc/SimD015Poc.cpp
)

target_include_directories(sim_d015_poc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(sim_d015_poc
  rclcpp
  robot_interfaces
  mj_sim
  sim_common
)

target_link_libraries(sim_d015_poc pthread dl)

install(TARGETS sim_d015_poc
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_dependencies(rclcpp robot_interfaces mj_sim)
ament_package()
