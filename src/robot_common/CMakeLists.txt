cmake_minimum_required(VERSION 3.10)
project(robot_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME}
  src/audio_client.cpp
  src/joint_config.cpp
  src/joint_limits.cpp
  src/yaml_loader.cpp
  src/ankle_transform.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} rclcpp robot_interfaces)
target_link_libraries(${PROJECT_NAME} yaml-cpp Eigen3::Eigen)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 踝关节串并联转换手动验证程序（独立可执行，不依赖 BUILD_TESTING）
add_executable(test_ankle_convert test/test_ankle_convert.cpp)
target_link_libraries(test_ankle_convert ${PROJECT_NAME} Eigen3::Eigen)
install(TARGETS test_ankle_convert DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp robot_interfaces yaml-cpp Eigen3)
ament_package()
