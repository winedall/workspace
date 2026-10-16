#!/bin/bash
#
# zl_robot 编译脚本
# 编译 ROS2 工作空间中的所有包。
# 注意：编译完成后需执行 install.sh 完成系统配置。
#
# 用法：
#   ./build.sh                    # 编译所有包

# 当前用户
USER_NAME=$(whoami)

# 获取执行脚本所在路径
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"   

# 工程路径
PROJECT_PATH=${SCRIPT_DIR}/..

# 切换路径
cd ${PROJECT_PATH}

# 编译程序
colcon build  --parallel-workers 2

# 单独编译robot_driver节点，开启CANFD延迟计算
colcon build --packages-select robot_driver --cmake-args -DWORKER_LOOP_DIAG=OFF -DWORKER_RT=ON

