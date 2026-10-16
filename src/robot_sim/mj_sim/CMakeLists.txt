cmake_minimum_required(VERSION 3.16)
project(mj_sim VERSION 0.1.0)

# 如果是ROS2，通常需要至少CMake 3.16
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)

find_package(Python3 REQUIRED)

# 设置默认编译选项
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  set(CMAKE_CXX_FLAGS_RELEASE -Ofast)
  set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall")
endif()

# 模型资产位于独立仓库 robot_assets（与 robot_sim 同级，见 src/robot_assets）。
# 默认按相对路径查找；资产仓库单独 clone 到别处时可用
#   colcon build ... --cmake-args -DROBOT_ASSETS_DIR=/path/to/robot_assets
# 覆盖。
set(ROBOT_ASSETS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../robot_assets"
    CACHE PATH "zl_robot_assets 模型资产仓库路径")
if(NOT EXISTS "${ROBOT_ASSETS_DIR}")
  message(FATAL_ERROR
    "robot_assets not found at ${ROBOT_ASSETS_DIR}. "
    "请单独 clone zl_robot_assets 仓库到 src/robot_assets，"
    "或通过 -DROBOT_ASSETS_DIR=<path> 指定其位置。")
endif()

# 设置路径
set(SCRIPT_DIR ${ROBOT_ASSETS_DIR}/utils)
set(XML_DESCRIPTION_DIR ${ROBOT_ASSETS_DIR})
set(HEADER_OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_header)

# 确保输出目录存在
file(MAKE_DIRECTORY ${HEADER_OUTPUT_DIR})

# 查找所有XML文件
file(GLOB XML_FILES 
     "${XML_DESCRIPTION_DIR}/*/xml/*.xml"
)

# 为每个XML文件创建生成规则
foreach(XML_FILE ${XML_FILES})
  # 获取文件名（不含扩展名）
  get_filename_component(BASE_NAME ${XML_FILE} NAME_WE)

  # 设置输出头文件路径
  set(HEADER_FILE ${HEADER_OUTPUT_DIR}/${BASE_NAME}.h)

  # 添加自定义命令生成头文件
  add_custom_command(
    OUTPUT ${HEADER_FILE}
    COMMAND ${Python3_EXECUTABLE}
    ${SCRIPT_DIR}/header_generator.py
    ${XML_FILE}
    -o ${HEADER_FILE}
    --namespace ${BASE_NAME}
    DEPENDS ${XML_FILE} ${SCRIPT_DIR}/header_generator.py
    COMMENT "Generating header: ${BASE_NAME}.h"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    VERBATIM
  )

  # 收集所有生成的头文件
  list(APPEND GENERATED_HEADERS ${HEADER_FILE})
endforeach()

add_custom_target(generate_robot_headers ALL
  DEPENDS ${GENERATED_HEADERS}
  COMMENT "All robot header files generated"
)

# 第三方路径统一管理
set(THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty")
set(MUJOCO_INCLUDE_DIR "${THIRD_PARTY_DIR}/mujoco")
set(GLFW_INCLUDE_DIR "${THIRD_PARTY_DIR}/glfw")
set(EIGEN_DIR "${THIRD_PARTY_DIR}/eigen3")

# 使用target_include_directories代替include_directories
# 添加包含目录到库目标中

# 自动匹配架构并设置库路径
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64" OR ${CMAKE_CXX_COMPILER} MATCHES "aarch64")
  message(STATUS "Target architecture: ARM64")
  set(ARCH_SUFFIX "_lin_arm64")
  set(MUJOCO_LIB_PATH "${THIRD_PARTY_DIR}/mujoco/lin_arm64")
  list(APPEND EXTRA_LIBS dl) # ARM 特定链接库
else()
  message(STATUS "Target architecture: x86_64")
  set(ARCH_SUFFIX "_lin_x64")
  set(MUJOCO_LIB_PATH "${THIRD_PARTY_DIR}/mujoco/lin_x64")
endif()

# 动态查找库
find_package(Threads REQUIRED)

find_library(GLFW3_LIB NAMES glfw3${ARCH_SUFFIX} glfw3 PATHS ${GLFW_INCLUDE_DIR})
if(NOT GLFW3_LIB)
  message(FATAL_ERROR "Could not find Glfw library")
endif()

# 查找 Mujoco 库
find_library(MUJOCO_LIB
  NAMES mujoco libmujoco.so libmujoco.so.3.1.1
  PATHS ${MUJOCO_LIB_PATH}
  NO_DEFAULT_PATH
)

if(NOT MUJOCO_LIB)
  message(FATAL_ERROR "Could not find Mujoco library")
endif()

# 创建导入目标以便更好地传递依赖
add_library(mujoco_imported SHARED IMPORTED)
set_target_properties(mujoco_imported PROPERTIES
  IMPORTED_LOCATION ${MUJOCO_LIB}
  INTERFACE_INCLUDE_DIRECTORIES ${MUJOCO_INCLUDE_DIR}
)

add_library(glfw_imported SHARED IMPORTED)
set_target_properties(glfw_imported PROPERTIES
  IMPORTED_LOCATION ${GLFW3_LIB}
  INTERFACE_INCLUDE_DIRECTORIES ${GLFW_INCLUDE_DIR}
)

# 添加库
aux_source_directory(./src/simulate SIMULATE_SRCS)

add_library(mj_sim
  src/mj_sim/MujocoSimBase.cpp
  ${SIMULATE_SRCS})

# 添加依赖
add_dependencies(mj_sim generate_robot_headers)

# 设置包含目录 - 使用target_include_directories
target_include_directories(mj_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/simulate>
  $<BUILD_INTERFACE:${HEADER_OUTPUT_DIR}>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(mj_sim PUBLIC
  $<BUILD_INTERFACE:${MUJOCO_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${GLFW_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${EIGEN_DIR}>
)

# 设置链接库
target_link_libraries(mj_sim
  mujoco_imported
  glfw_imported
  Threads::Threads
  dl
  ${EXTRA_LIBS}
)

# 添加ament依赖
ament_target_dependencies(mj_sim
)

# 安装目标
install(TARGETS
  mj_sim
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# 安装生成的机器人头文件
install(DIRECTORY ${HEADER_OUTPUT_DIR}/
  DESTINATION include/${PROJECT_NAME}/robot_header
  FILES_MATCHING PATTERN "*.h"
)

# 安装 XML 场景文件（来自独立资产仓库，排除其 .git 目录）
install(
  DIRECTORY ${ROBOT_ASSETS_DIR}/
  DESTINATION share/${PROJECT_NAME}/description
  FILES_MATCHING PATTERN "*"
  PATTERN ".git" EXCLUDE
)

# 安装 mujoco 库文件（如果需要）
install(FILES ${MUJOCO_LIB_PATH}/libmujoco.so.3.1.1
  DESTINATION lib
)

# 安装Eigen
install(
  DIRECTORY ${EIGEN_DIR}/Eigen
  DESTINATION include/${PROJECT_NAME}/thirdparty/eigen3
)
install(
  DIRECTORY ${EIGEN_DIR}/unsupported
  DESTINATION include/${PROJECT_NAME}/thirdparty/eigen3
)

# 安装mujoco头文件
install(
  DIRECTORY ${MUJOCO_INCLUDE_DIR}/mujoco/
  DESTINATION include/${PROJECT_NAME}/thirdparty/mujoco
)

# 安装glfw头文件
install(
  DIRECTORY ${GLFW_INCLUDE_DIR}/GLFW/
  DESTINATION include/${PROJECT_NAME}/thirdparty/glfw
)

# 导出库信息
ament_export_include_directories(
  include
  include/${PROJECT_NAME}/thirdparty/eigen3
  include/${PROJECT_NAME}/thirdparty
  include/${PROJECT_NAME}/
)
ament_export_libraries(mj_sim
  mujoco_imported
  glfw_imported
)


# 必须放在最后
ament_package()
