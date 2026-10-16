cmake_minimum_required(VERSION 3.16)
project(sim_z01_std_parallel_ankle VERSION 0.1.0)

# 如果是ROS2，需要至少CMake 3.16
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# 查找 ament_cmake
find_package(ament_cmake REQUIRED)

# ROS2 依赖 - 根据实际需要调整
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(mj_sim REQUIRED)
find_package(sim_common REQUIRED)

# 设置默认编译选项
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
    set(CMAKE_CXX_FLAGS_RELEASE -Ofast)
    set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall")
endif()

# 创建可执行文件
add_executable(sim_z01_std_parallel_ankle
    src/main.cpp
    src/sim_z01_std_parallel_ankle/SimZ01StdParallelAnkle.cpp
)

# 添加包含目录
target_include_directories(sim_z01_std_parallel_ankle PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# 添加ament依赖（会自动处理包含目录）
ament_target_dependencies(sim_z01_std_parallel_ankle
    rclcpp
    robot_interfaces
    mj_sim
    sim_common
)

# 链接其他库
target_link_libraries(sim_z01_std_parallel_ankle
    pthread
    dl
)

# 安装目标
install(TARGETS sim_z01_std_parallel_ankle
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件（如果有）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
    install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
    )
endif()

# 安装仿真配置文件（发布频率与可调常量）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
    install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
    )
endif()

# 安装头文件（如果有库需要暴露给其他包）
install(DIRECTORY include/
    DESTINATION include/
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# 导出依赖
ament_export_dependencies(
    rclcpp
    robot_interfaces
    mj_sim
)

ament_package()
