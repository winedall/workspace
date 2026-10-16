#!/bin/bash
set -e

MODE="${1:-both}"
JOINT_NAME="${2:-waist_yaw}"
ORDER="${3:-warning_first}"
COUNT="${4:-3}"

case "${MODE}" in
  system_fault|joint_warning|both) ;;
  *)
    echo "usage: ros2 run robot_injector inject_fault.sh [system_fault|joint_warning|both] [joint_name] [warning_first|fault_first] [count]"
    exit 1
    ;;
esac

exec ros2 launch robot_injector fault_inject.launch.py \
  mode:="${MODE}" \
  joint_name:="${JOINT_NAME}" \
  order:="${ORDER}" \
  count:="${COUNT}"
