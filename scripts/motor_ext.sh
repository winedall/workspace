#!/usr/bin/env bash
# motor_ext.sh - 借用【正在运行的完整系统】做单电机位置覆盖调试（不启动 driver）。
#
# 前置：先在另一终端 ./scripts/run.sh 起整机，把机器人吊起/离地，
#       切到 RL 持续吐命令的站立/待命态，并把 router 切到 MIX（遥控器 mode→mix，
#       或本工具里敲 mix）。然后运行本脚本选关节下发覆盖命令。
#
# 依赖: bash, python3(+PyYAML), 已 source 的 ros2 与 install/setup.bash
# 用法: ./motor_ext.sh [-t humanoid] [-c CONFIG_ROOT] [--ns /robot] [--rate HZ]

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}/.."
CONFIG_ROOT="${PROJECT_ROOT}/config"

ROBOT_TYPE="humanoid"
HARDWARE_PROFILE="tienkung"
NS="/robot"
RATE="50"

while [[ $# -gt 0 ]]; do
    case "$1" in
        -t|--type)     ROBOT_TYPE="${2:-}"; shift 2 ;;
        -p|--profile)  HARDWARE_PROFILE="${2:-}"; shift 2 ;;
        -c|--config)   CONFIG_ROOT="${2:-}"; shift 2 ;;
        --ns)          NS="${2:-}"; shift 2 ;;
        --rate)        RATE="${2:-}"; shift 2 ;;
        -h|--help)     sed -n '2,12p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "未知参数: $1"; exit 1 ;;
    esac
done

JOINTS_CONFIG="${CONFIG_ROOT}/${ROBOT_TYPE}/robot_joints_${HARDWARE_PROFILE}.yaml"
[[ -f "${JOINTS_CONFIG}" ]] || { echo "[ERROR] 找不到关节配置: ${JOINTS_CONFIG}"; exit 1; }

# 对齐 DDS 环境（否则看不到正在跑的整机节点——老坑）。
if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
fi
if [[ -f "${PROJECT_ROOT}/install/setup.bash" ]]; then
    source "${PROJECT_ROOT}/install/setup.bash"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${CONFIG_ROOT}/cyclonedds_localhost.xml"

echo "[INFO] 关节配置 = ${JOINTS_CONFIG}  ns=${NS}  rate=${RATE}Hz"
echo "[INFO] 前提：另一终端已 ./scripts/run.sh，机器人离地、RL 站立/待命、router 已切 MIX。"
exec python3 "${SCRIPT_DIR}/motor_ext.py" \
    --joints-config "${JOINTS_CONFIG}" --ns "${NS}" --rate "${RATE}" "$@"
