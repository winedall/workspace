cmake_minimum_required(VERSION 3.10)
project(robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(WORKER_LOOP_DIAG "Enable backend worker loop timing diagnostics" OFF)
option(WORKER_RT "Enable SCHED_FIFO scheduling for backend worker thread" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(robot_common REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(soem_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

find_path(IGH_INCLUDE_DIR ecrt.h PATHS /usr/local/include /usr/include)
find_library(IGH_LIBRARY ethercat PATHS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu)

add_library(${PROJECT_NAME}
  src/backend/backend_factory.cpp
  src/backend/hardware_backend.cpp
  src/backend/encos_backend.cpp
  src/backend/mix_backend.cpp
  src/backend/tienkung_backend.cpp
  src/canfd/canfd_master.cpp
  src/canfd/canfd_transport.cpp
  src/ethercat/gateway/canfd_gateway.cpp
  src/ethercat/gateway/encos_gateway.cpp
  src/ethercat/gateway/tienkung_gateway.cpp
  src/ethercat/igh_master.cpp
  src/ethercat/soem_master.cpp
  src/hardware_config.cpp
  src/protocol/encos_protocol.cpp
  src/protocol/tienkung_protocol.cpp
  src/robot_driver_node.cpp
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
  soem_vendor
)
if(WORKER_LOOP_DIAG)
  target_compile_definitions(${PROJECT_NAME} PRIVATE WORKER_LOOP_DIAG)
endif()
if(WORKER_RT)
  target_compile_definitions(${PROJECT_NAME} PRIVATE WORKER_RT)
endif()
target_link_libraries(${PROJECT_NAME} yaml-cpp)
target_link_libraries(${PROJECT_NAME} Threads::Threads)
if(IGH_INCLUDE_DIR AND IGH_LIBRARY)
  target_include_directories(${PROJECT_NAME} PRIVATE ${IGH_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME} ${IGH_LIBRARY})
  target_compile_definitions(${PROJECT_NAME} PRIVATE ROBOT_DRIVER_HAS_IGH)
endif()

add_executable(robot_driver_node src/robot_driver_node_main.cpp)
target_link_libraries(robot_driver_node ${PROJECT_NAME})
ament_target_dependencies(robot_driver_node rclcpp)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME} robot_driver_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp robot_common robot_interfaces soem_vendor)

if(BUILD_TESTING)
  ament_add_gtest(test_fault_detail test/test_fault_detail.cpp)
  target_compile_definitions(test_fault_detail
    PRIVATE ROBOT_DRIVER_TEST_WORKSPACE="${CMAKE_CURRENT_SOURCE_DIR}/../..")
  target_link_libraries(test_fault_detail ${PROJECT_NAME})
  ament_target_dependencies(test_fault_detail rclcpp robot_common robot_interfaces)

  ament_add_gtest(test_hardware_config test/test_hardware_config.cpp)
  target_compile_definitions(test_hardware_config
    PRIVATE ROBOT_DRIVER_TEST_WORKSPACE="${CMAKE_CURRENT_SOURCE_DIR}/../..")
  target_link_libraries(test_hardware_config ${PROJECT_NAME})
  ament_target_dependencies(test_hardware_config rclcpp robot_common robot_interfaces)
endif()

ament_package()
