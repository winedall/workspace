cmake_minimum_required(VERSION 3.16)
project(sim_z01_std_robocup VERSION 0.1.0)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yesense_interface REQUIRED)
find_package(mj_sim REQUIRED)
find_package(sim_common REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
    set(CMAKE_CXX_FLAGS_RELEASE -Ofast)
    set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall")
endif()

add_executable(sim_z01_std_robocup
    src/main.cpp
    src/sim_z01_std_robocup/SimZ01StdRobocup.cpp
    src/sim_z01_std_robocup/RobotChannel.cpp
    src/sim_z01_std_robocup/RobotModelSpec.cpp
    src/sim_z01_std_robocup/SimRosExecutor.cpp
)

target_include_directories(sim_z01_std_robocup PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(sim_z01_std_robocup
    rclcpp
    robot_interfaces
    geometry_msgs
    std_msgs
    std_srvs
    sensor_msgs
    yesense_interface
    mj_sim
    sim_common
)

target_link_libraries(sim_z01_std_robocup
    pthread
    dl
)

install(TARGETS sim_z01_std_robocup
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
    install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
    )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
    install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
    )
endif()

install(DIRECTORY include/
    DESTINATION include/
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

ament_export_dependencies(
    rclcpp
    robot_interfaces
    mj_sim
    sim_common
)

ament_package()
