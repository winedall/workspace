#!/bin/bash
#
# OTA 服务启动脚本（前台运行）。
# 前提：工作区已完成构建。
#
# 用法：
#   ./scripts/ota.sh
#   ./scripts/ota.sh -p 8001

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_PATH="${SCRIPT_DIR}/.."
PORT=""

usage() {
  echo "用法: $(basename "$0") [-p port]"
}

while getopts ":p:h" option; do
  case "$option" in
    p)
      PORT="$OPTARG"
      ;;
    h)
      usage
      exit 0
      ;;
    :)
      echo "选项 -$OPTARG 需要指定端口号" >&2
      usage >&2
      exit 2
      ;;
    \?)
      echo "未知选项: -$OPTARG" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -n "$PORT" ]] && { [[ ! "$PORT" =~ ^[0-9]+$ ]] || ((PORT < 1 || PORT > 65535)); }; then
  echo "端口号必须是 1 到 65535 之间的整数: $PORT" >&2
  exit 2
fi

# ── ROS / DDS 环境 ────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
source "${PROJECT_PATH}/install/setup.bash"

# 运行ota服务
if [[ -n "$PORT" ]]; then
  exec ros2 launch robot_bringup zl_robot_ota.launch.py "port:=${PORT}"
fi

exec ros2 launch robot_bringup zl_robot_ota.launch.py


