cmake_minimum_required(VERSION 3.10)
project(robot_recorder)

# 开启编译器警告
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 使用 C++17 标准（支持 std::variant, std::filesystem 等）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(robot_common REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(yesense_interface REQUIRED)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

# 构建核心库（csv_writer + recorder_node）
add_library(${PROJECT_NAME}
  src/csv_writer.cpp
  src/joint_record_sample.cpp
  src/recorder_node.cpp
  src/event_manager.cpp
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
  yesense_interface
)

# 构建节点可执行文件
add_executable(robot_recorder_node src/recorder_node_main.cpp)
target_link_libraries(robot_recorder_node ${PROJECT_NAME})
ament_target_dependencies(robot_recorder_node rclcpp)

# 安装头文件和库
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME} robot_recorder_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ament_add_gtest(test_command_csv_fields
    test/test_command_csv_fields.cpp
  )
  target_link_libraries(test_command_csv_fields ${PROJECT_NAME})
  ament_target_dependencies(test_command_csv_fields robot_interfaces)

  ament_add_gtest(test_time_window_buffer
    test/test_time_window_buffer.cpp
  )
  target_link_libraries(test_time_window_buffer ${PROJECT_NAME})

  ament_add_gtest(test_recording_paths
    test/test_recording_paths.cpp
  )
  target_link_libraries(test_recording_paths ${PROJECT_NAME})

  ament_add_gtest(test_event_manager
    test/test_event_manager.cpp
  )
  target_link_libraries(test_event_manager ${PROJECT_NAME})
  ament_target_dependencies(test_event_manager robot_interfaces)
endif()

# 导出依赖信息给下游包
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp robot_common robot_interfaces yesense_interface)
ament_package()
