#!/bin/bash
#
# zl_robot 清理脚本
# 删除编译产物（build、install、log 目录）。
# 执行后需要重新运行 build.sh 编译。
#
# 用法：
#   ./clear.sh                    # 清理编译产物

# 当前用户
USER_NAME=$(whoami)

# 获取执行脚本所在路径
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"   

# 工程路径
PROJECT_PATH=${SCRIPT_DIR}/..

# 切换路径
cd ${PROJECT_PATH}

# 编译程序
rm -rf build install log
