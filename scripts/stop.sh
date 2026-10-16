#!/bin/bash
#
# zl_robot 停止/清理脚本。
# 日常停止请使用：sudo systemctl stop zl_robot
#
# 用法：
#   sudo systemctl stop zl_robot        # systemd 按 service cgroup 停止（推荐）
#   ./stop.sh                           # 只清理当前项目路径相关的残留进程
#   ./stop.sh --force                   # 对当前项目路径相关进程直接 SIGKILL
#   ./stop.sh --cleanup                 # 危险：清理当前 DDS domain 可发现的本机 ROS 节点

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_PATH="$(cd "${SCRIPT_DIR}/.." && pwd)"

FORCE=false
CLEANUP_ROS_DOMAIN=false

usage() {
    echo "用法: $(basename "$0") [--force] [--cleanup]"
}

for arg in "$@"; do
    case "$arg" in
        --force)
            FORCE=true
            ;;
        --cleanup)
            CLEANUP_ROS_DOMAIN=true
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "未知参数: $arg" >&2
            usage >&2
            exit 2
            ;;
    esac
done

signal_project_processes() {
    local signal="$1"

    pkill "-$signal" -f "${PROJECT_PATH}/install/" 2>/dev/null || true
    pkill "-$signal" -f "${PROJECT_PATH}/scripts/run.sh" 2>/dev/null || true
    pkill "-$signal" -f "ros2 launch robot_bringup zl_robot.launch.py" 2>/dev/null || true
    pkill "-$signal" -f "/opt/ros/humble/lib/joy/joy_node" 2>/dev/null || true
}

cleanup_project_processes() {
    if $FORCE; then
        signal_project_processes "KILL"
        return
    fi

    signal_project_processes "TERM"
    sleep 2
    signal_project_processes "KILL"
}

cleanup_ros_domain() {
    echo "[WARN] --cleanup 会清理当前 DDS domain 下本机可发现的 ROS 节点。"
    echo "[WARN] 如果本机还有其他 ROS 程序，它们也可能被停止。"

    source /opt/ros/humble/setup.bash 2>/dev/null || true
    source "${PROJECT_PATH}/install/setup.bash" 2>/dev/null || true

    local first_signal="TERM"
    $FORCE && first_signal="KILL"

    if command -v ros2 >/dev/null 2>&1; then
        while IFS= read -r node; do
            [ -n "$node" ] && pkill "-$first_signal" -f "$node" 2>/dev/null || true
        done < <(ros2 node list 2>/dev/null)
    fi

    if ! $FORCE; then
        sleep 1
        if command -v ros2 >/dev/null 2>&1; then
            while IFS= read -r node; do
                [ -n "$node" ] && pkill -KILL -f "$node" 2>/dev/null || true
            done < <(ros2 node list 2>/dev/null)
        fi
    fi
}

if $CLEANUP_ROS_DOMAIN; then
    cleanup_ros_domain
else
    cleanup_project_processes
fi

echo "zl_robot 停止清理完成。"
