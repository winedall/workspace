#!/bin/bash
#
# zl_robot 故障注入脚本（前台运行）
# 设置 ROS/DDS 环境后启动交互式故障注入工具。
#
# 用法：
#   ./injector.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_PATH="${SCRIPT_DIR}/.."

# ── ROS / DDS 环境 ────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
source "${PROJECT_PATH}/install/setup.bash"

# 运行故障注入脚本
exec ros2 run robot_injector inject_fault.sh
