cmake_minimum_required(VERSION 3.10)
project(sim_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(yesense_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mj_sim REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME}
  src/SimRosNodeBase.cpp
  src/MjInterface.cpp
  src/SimNodeBase.cpp
  src/SimRuntimeOptions.cpp
  src/SimErrorModel.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  robot_interfaces
  yesense_interface
  sensor_msgs
  mj_sim
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp robot_interfaces yesense_interface sensor_msgs mj_sim Eigen3)
ament_package()
