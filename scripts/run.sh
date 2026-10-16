#!/bin/bash
#
# zl_robot 启动脚本（前台运行）
# 设置 ROS/DDS 环境、显示环境后启动机器人。
# 前提：install.sh 已执行（sudo 权限操作已完成），日常运行无需 sudo。
#
# 用法：
#   sudo systemctl start zl_robot    # systemd 启动（推荐）
#   ./run.sh                          # 直接前台启动（调试用）

# ── 基础路径 ────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_PATH="${SCRIPT_DIR}/.."
CONFIG_ROOT="${PROJECT_PATH}/config"
LOG_ROOT="${PROJECT_PATH}/logs"
DOMAIN_BRIDGE_CONFIG="${CONFIG_ROOT}/domain_bridge_robot.yaml"

# ── 用户可配置项 ────────────────────────────────────────────────
# 直接修改这里即可切换运行形态，避免外部环境变量残留影响真机启动。
#
# 可选值：
#   ROBOT_TYPE         humanoid | dog
#   HARDWARE_PROFILE   encos | tienkung | mix_canfd | mix_ethercat
#                      -> config/<robot>/hw_<profile>.yaml
#   DDS_MODE           localhost | bridge
#                      localhost: 只绑定本机 DDS
#                      bridge:    启用 domain_bridge 白名单对外通信
#   RUNTIME_PLATFORM   x86_64 | aarch64，默认可用 uname -m 自动识别
#   ENABLE_MUJOCO_SIM  true | false
#   ANKLE_ENV_PARALLEL true | false，仅人形仿真有效；真机人形固定并联
#   RECORDER_MODE      event_window | continuous
#                      event_window: 平时只保存故障前后窗口数据
#                      continuous:   持续记录全部高频数据，适合专项调试
#   STARTUP_DELAY      启动 launch 前等待秒数
#   PASSWD             sudo 密码，用于设备辅助配置
ROBOT_TYPE="humanoid"
HARDWARE_PROFILE="tienkung"
RUNTIME_PLATFORM="$(uname -m)"
ENABLE_MUJOCO_SIM="false"
ANKLE_ENV_PARALLEL="true"
DDS_MODE="localhost"
RECORDER_MODE="continuous"
ROS_DOMAIN_ID="10"
STARTUP_DELAY="5"
PASSWD="admin"
# PASSWD="123456"

# 缓存 sudo 密码，后续音频/设备辅助操作可直接使用 sudo 权限。
echo "$PASSWD" | sudo -S pwd

set -e

# 校验硬件配置 profile。
#
validate_hardware_profile_or_die() {
    case "$HARDWARE_PROFILE" in
        encos|tienkung|mix_canfd|mix_ethercat)
            ;;
        *)
            echo "[FATAL] HARDWARE_PROFILE must be encos|tienkung|mix_canfd|mix_ethercat, got: ${HARDWARE_PROFILE}"
            exit 1
            ;;
    esac
}

# 根据 DDS_MODE 选择 DDS 通信策略：
#   bridge：运控内部 domain 10 只绑定 lo，外部 domain 100 通过 domain_bridge 白名单访问。
#   localhost：只绑定 lo，不启动 domain_bridge，不向外部网络开放 ROS 图。
select_dds_mode() {
    case "$DDS_MODE" in
        bridge)
            CYCLONEDDS_CONFIG="${CONFIG_ROOT}/cyclonedds_bridge_whitelist.xml"
            ENABLE_DOMAIN_BRIDGE="true"
            ;;
        localhost)
            CYCLONEDDS_CONFIG="${CONFIG_ROOT}/cyclonedds_localhost.xml"
            ENABLE_DOMAIN_BRIDGE="false"
            ;;
        *)
            echo "[FATAL] DDS_MODE must be 'bridge' or 'localhost', got: $DDS_MODE"
            exit 1
            ;;
    esac
}

# 初始化本次运行的日志目录，并把脚本 stdout/stderr 统一写入 zl_robot.log。
# ROS 自身仍会在 ROS_LOG_DIR 下写 launch.log，recorder 输出也放到同一次 run 目录。
setup_run_logs() {
    RUN_ID="$(date +%Y%m%d_%H%M%S)"
    RUN_LOG_DIR="${LOG_ROOT}/${RUN_ID}"
    RUN_LOG_FILE="${RUN_LOG_DIR}/zl_robot.log"
    RECORDER_OUTPUT_DIR="${RUN_LOG_DIR}/recorder"

    mkdir -p "$RUN_LOG_DIR"
    export ROS_LOG_DIR="${RUN_LOG_DIR}/ros"
    mkdir -p "$ROS_LOG_DIR" "$RECORDER_OUTPUT_DIR"

    # Keep one operator-facing run log. ROS launch still writes launch.log under
    # ROS_LOG_DIR, while node logs are sent to stdout/stderr.
    exec > >(tee -a "$RUN_LOG_FILE") 2>&1
    echo "[INFO] Unified run log: $RUN_LOG_FILE"
    echo "[INFO] Recorder output: $RECORDER_OUTPUT_DIR"

    "${SCRIPT_DIR}/logs.sh" cleanup 30 51200 "$LOG_ROOT" "$RUN_LOG_DIR"
}

# 加载 ROS 2 和当前工作区环境，并清理 FastDDS 相关变量，确保本进程使用 CycloneDDS。
setup_ros_environment() {
    source /opt/ros/humble/setup.bash
    source "${PROJECT_PATH}/install/setup.bash"

    export ROS_DOMAIN_ID
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="file://${CYCLONEDDS_CONFIG}"
    export ROS_LOCALHOST_ONLY=0
    export RCUTILS_COLORIZED_OUTPUT=1
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${PROJECT_PATH}/install/yesense_std_ros2/lib/yesense_std_ros2/serial/"

    unset FASTRTPS_DEFAULT_PROFILES_FILE
    unset FASTDDS_DEFAULT_PROFILES_FILE
    unset RMW_FASTRTPS_USE_QOS_FROM_XML
    unset RMW_FASTRTPS_USE_SHM
    unset FASTDDS_BUILTIN_TRANSPORTS
}

# 确认 CycloneDDS RMW 已安装；脚本会强制使用该 RMW。
check_cyclonedds_rmw_or_die() {
    if ! ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
        echo "[FATAL] rmw_cyclonedds_cpp is not available. Please install ros-humble-rmw-cyclonedds-cpp."
        exit 1
    fi
}

# 输出本次生效的 DDS 配置，便于从日志确认当前通信模式。
print_dds_environment() {
    echo "[INFO] DDS mode: ${DDS_MODE}"
    echo "[INFO] Hardware profile: ${HARDWARE_PROFILE}"
    echo "[INFO] ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
    echo "[INFO] RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}"
    echo "[INFO] CYCLONEDDS_URI: ${CYCLONEDDS_URI}"
    echo "[INFO] Domain bridge enabled: ${ENABLE_DOMAIN_BRIDGE}"
}

# 设置 MuJoCo/GLFW 必需的图形显示环境与相关用户运行时环境。
# systemd 启动的服务没有图形会话，DISPLAY/XAUTHORITY 通常缺失，GLFW 无法创建窗口。
# 这里复用已有 X socket；若完全没有显示，则启动 Xvfb 虚拟显示兜底，保证仿真可视化可用。
# 同时设置 XDG_RUNTIME_DIR（对 PulseAudio/PipeWire/ALSA 设备解析及 X 会话很关键）。
# 全部为软处理（仅 WARN），不影响机器人主流程执行。
setup_display_environment() {
    export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"

    # 无显示环境下尝试复用已有 X socket
    if [ -z "$DISPLAY" ]; then
        for sock in /tmp/.X11-unix/X*; do
            [ -e "$sock" ] || continue
            export DISPLAY=":${sock##*/X}"
            break
        done
    fi
    # 仍然没有 DISPLAY 时，启动 Xvfb 虚拟显示兜底
    if [ -z "$DISPLAY" ]; then
        if command -v Xvfb >/dev/null 2>&1; then
            export DISPLAY=:99
            XVFB_PID_FILE="/tmp/zl_robot_xvfb.pid"
            if [ ! -f "$XVFB_PID_FILE" ] || ! kill -0 "$(cat "$XVFB_PID_FILE")" 2>/dev/null; then
                Xvfb :99 -screen 0 1024x768x24 &
                echo $! > "$XVFB_PID_FILE"
                sleep 1
            fi
        else
            echo "[WARN] DISPLAY 未设置且 Xvfb 未安装。MuJoCo 可视化可能不可用。"
            echo "        安装: sudo apt install xvfb"
            export DISPLAY=:99
        fi
    fi

    if [ -z "$XAUTHORITY" ]; then
        if [ -f "$HOME/.Xauthority" ]; then
            export XAUTHORITY="$HOME/.Xauthority"
        elif [ -f "/run/user/$(id -u)/gdm/Xauthority" ]; then
            export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
        fi
    fi

    echo "[INFO] DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR"
}

# 音频设备检查只作为启动前辅助处理，失败不应中断机器人主流程。
check_audio_environment_soft() {
    if ! pactl list sinks short 2>/dev/null | grep -qv "auto_null"; then
        echo "[WARN] 无音频输出设备，尝试重启 PulseAudio..."
        systemctl --user restart pulseaudio 2>/dev/null || true
        sleep 1
    fi
    return 0
}

# 等待 IMU 设备符号链接出现。仅真机模式需要；超时后不中断主流程。
wait_for_imu_device() {
    local device="/dev/robot_imu"
    local timeout="60"

    if [ "${ENABLE_MUJOCO_SIM}" = "true" ]; then
        echo "[INFO] Simulation mode enabled, skip waiting for IMU device."
        return 0
    fi

    for i in $(seq 1 "$timeout"); do
        if [ -e "$device" ]; then
            echo "[INFO] IMU device detected: $device"
            return 0
        fi
        echo "[WARN] Waiting for IMU device... ($i/$timeout)"
        sleep 1
    done

    echo "[ERROR] IMU device not found after ${timeout}s, continuing without IMU..."
    return 0
}

# 组装 ros2 launch 参数并用 exec 交给 systemd 跟踪。
# 这里集中传入 DDS 相关参数，保证 bridge 模式会使用白名单配置。
launch_robot() {
    echo "[INFO] Waiting ${STARTUP_DELAY}s before starting robot launch..."
    sleep "$STARTUP_DELAY"

    LAUNCH_ARGS=(
        "arg_enable_mujoco_sim:=${ENABLE_MUJOCO_SIM}"
        "arg_robot_type:=${ROBOT_TYPE}"
        "arg_config_root:=${CONFIG_ROOT}"
        "arg_hardware_profile:=${HARDWARE_PROFILE}"
        "arg_runtime_platform:=${RUNTIME_PLATFORM}"
        "arg_parallel_ankle:=${ANKLE_ENV_PARALLEL}"
        "arg_dir_recorder:=${RECORDER_OUTPUT_DIR}"
        "arg_recorder_mode:=${RECORDER_MODE}"
        "arg_enable_domain_bridge:=${ENABLE_DOMAIN_BRIDGE}"
        "arg_file_domain_bridge:=${DOMAIN_BRIDGE_CONFIG}"
        "arg_file_cyclonedds:=${CYCLONEDDS_CONFIG}"
    )
    exec ros2 launch robot_bringup zl_robot.launch.py "${LAUNCH_ARGS[@]}"
}

# 主流程入口：上面各函数按职责分段，入口只保留启动顺序。
validate_hardware_profile_or_die
select_dds_mode
setup_run_logs
setup_ros_environment
check_cyclonedds_rmw_or_die
print_dds_environment
setup_display_environment
check_audio_environment_soft
wait_for_imu_device
launch_robot
