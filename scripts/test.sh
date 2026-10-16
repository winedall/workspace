#!/bin/bash
#
# 导航模式转向测试脚本。
#
# 使用场景：
#   1. 先手动把机器人切到导航模式；
#   2. 再运行本脚本，让机器人按固定节奏向 /cmd_vel 发布转向速度；
#   3. 导航模式下 RL manager 使用 /cmd_vel，不再使用遥控器持续发布的 RobotIntent 速度，
#      因此可以避免脚本和遥控器同时向 /robot/input/intent 发速度指令造成冲突。
#
# 用法：
#   ./scripts/test.sh [总运行秒数]
#
# 示例：
#   ./scripts/test.sh 120
#   表示循环转向 120 秒。
#
# 默认动作序列：
#   第一次左转 2 秒：angular.z =  0.5 rad/s
#   之后循环：
#     右转 4 秒：angular.z = -0.5 rad/s
#     左转 4 秒：angular.z =  0.5 rad/s
#   直到达到指定总运行时长。
#
# 可选环境变量：
#   ZL_ROBOT_TEST_RATE_HZ       发布频率，默认 10 Hz
#   ZL_ROBOT_TEST_CMD_VEL_TOPIC /cmd_vel 话题名，默认 /cmd_vel
#   ZL_ROBOT_TEST_LINEAR_X      前进速度，默认 0.0 m/s
#   ZL_ROBOT_TEST_LINEAR_Y      侧向速度，默认 0.0 m/s
#   ZL_ROBOT_TEST_LEFT_YAW      左转角速度，默认 0.5 rad/s
#   ZL_ROBOT_TEST_RIGHT_YAW     右转角速度，默认 -0.5 rad/s

# 先不启用 set -u：colcon 生成的 install/setup.bash 会读取可能未定义的
# COLCON_TRACE 等变量，提前启用 nounset 会导致 source 失败。
set -eo pipefail

# 脚本位于 scripts/ 目录，仓库根目录是 scripts 的上一级。
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_PATH="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP_FILE="${PROJECT_PATH}/install/setup.bash"

# 第一个参数是总运行时长，未传参时默认运行 60 秒。
DURATION_SEC="${1:-60}"

# 下面这些参数可通过环境变量临时覆盖，便于现场调试。
RATE_HZ="${ZL_ROBOT_TEST_RATE_HZ:-10}"
TOPIC="${ZL_ROBOT_TEST_CMD_VEL_TOPIC:-/cmd_vel}"
LINEAR_X="${ZL_ROBOT_TEST_LINEAR_X:-0.0}"
LINEAR_Y="${ZL_ROBOT_TEST_LINEAR_Y:-0.0}"
LEFT_YAW="${ZL_ROBOT_TEST_LEFT_YAW:-0.5}"
RIGHT_YAW="${ZL_ROBOT_TEST_RIGHT_YAW:--0.5}"

if [ ! -f "${SETUP_FILE}" ]; then
    echo "[FATAL] 未找到 ROS 工作空间环境文件: ${SETUP_FILE}" >&2
    echo "        请先完成编译，确保仓库根目录下存在 install/setup.bash。" >&2
    exit 1
fi

# 这里只接受非负整数秒，避免 timeout 收到奇怪参数。
case "${DURATION_SEC}" in
    ''|*[!0-9]*)
        echo "[FATAL] 总运行秒数必须是非负整数，当前值: ${DURATION_SEC}" >&2
        exit 1
        ;;
esac

case "${RATE_HZ}" in
    ''|*[!0-9]*|0)
        echo "[FATAL] 发布频率必须是正整数，当前值: ${RATE_HZ}" >&2
        exit 1
        ;;
esac

# 加载 ROS 环境，使 ros2 命令能找到 robot_interfaces、geometry_msgs 等消息类型。
source "${SETUP_FILE}"

# ROS 环境加载完成后再启用未定义变量检查，保护脚本自身逻辑。
set -u

echo "[INFO] 将以 ${RATE_HZ} Hz 向 ${TOPIC} 发布 Twist，总时长 ${DURATION_SEC}s。"

# 使用单个 rclpy 进程发布，避免反复启动 ros2 CLI 或用 timeout 强杀 ROS 进程。
# 这样既不会卡在 ros2 命令加载，也能避免 rcl wait set 的退出竞态日志。
export ZL_ROBOT_TEST_DURATION_SEC="${DURATION_SEC}"
export ZL_ROBOT_TEST_RATE_HZ="${RATE_HZ}"
export ZL_ROBOT_TEST_CMD_VEL_TOPIC="${TOPIC}"
export ZL_ROBOT_TEST_LINEAR_X="${LINEAR_X}"
export ZL_ROBOT_TEST_LINEAR_Y="${LINEAR_Y}"
export ZL_ROBOT_TEST_LEFT_YAW="${LEFT_YAW}"
export ZL_ROBOT_TEST_RIGHT_YAW="${RIGHT_YAW}"

python3 - <<'PY'
import os
import signal
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist


duration_sec = int(os.environ["ZL_ROBOT_TEST_DURATION_SEC"])
rate_hz = int(os.environ["ZL_ROBOT_TEST_RATE_HZ"])
topic = os.environ["ZL_ROBOT_TEST_CMD_VEL_TOPIC"]
linear_x = float(os.environ["ZL_ROBOT_TEST_LINEAR_X"])
linear_y = float(os.environ["ZL_ROBOT_TEST_LINEAR_Y"])
left_yaw = float(os.environ["ZL_ROBOT_TEST_LEFT_YAW"])
right_yaw = float(os.environ["ZL_ROBOT_TEST_RIGHT_YAW"])

running = True


def request_stop(signum, frame):
    del signum, frame
    global running
    running = False


def make_twist(yaw: float) -> Twist:
    msg = Twist()
    msg.linear.x = linear_x
    msg.linear.y = linear_y
    msg.angular.z = yaw
    return msg


def make_zero_twist() -> Twist:
    return Twist()


def publish_for(pub, seconds: int, yaw: float, label: str, period: float) -> int:
    if seconds <= 0:
        return 0
    print(f"[INFO] {label}: {seconds}s, angular.z={yaw} rad/s", flush=True)
    ticks = seconds * rate_hz
    msg = make_twist(yaw)
    sent = 0
    next_time = time.monotonic()
    for _ in range(ticks):
        if not running:
            break
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        sent += 1
        next_time += period
        sleep_time = next_time - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)
    return sent


signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)
signal.signal(signal.SIGQUIT, request_stop)

rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node = rclpy.create_node("cmd_vel_turn_test")
pub = node.create_publisher(Twist, topic, 10)
period = 1.0 / rate_hz
remaining = duration_sec

try:
    step = min(remaining, 2)
    publish_for(pub, step, left_yaw, "初始左转", period)
    remaining -= step

    while running and remaining > 0:
        step = min(remaining, 4)
        publish_for(pub, step, right_yaw, "右转", period)
        remaining -= step

        step = min(remaining, 4)
        publish_for(pub, step, left_yaw, "左转", period)
        remaining -= step
finally:
    print("\n[INFO] 发送零速度 /cmd_vel。", flush=True)
    zero = make_zero_twist()
    if rclpy.ok():
        try:
            for _ in range(max(3, rate_hz // 2)):
                pub.publish(zero)
                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(period)
        except Exception as exc:
            print(f"[WARN] 发送零速度失败: {exc}", flush=True)
    try:
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if running:
    print("[INFO] 测试结束。", flush=True)
else:
    sys.exit(130)
PY
