#!/usr/bin/env bash
# motor_fp.sh - 单电机力位混合控制（真机调试工具）
#
# 与 motor_motion.sh（只做位置限速运动）互补：
#   本脚本以运控模式 (config_mode=false) 单独拉起 robot_driver，
#   然后用 motor_fp.py 交互选择关节，对单个电机做
#   位置伺服 / 力位混合 / 纯力矩 三模式控制（Tienkung 0x11 力位混控帧）。
#
# ⚠ 危险：同一时刻只使能一个关节，其余全部断使能；力矩模式会主动出力。
#   机器人必须吊起/台架固定，脚不承重，否则会倾倒或弹飞！
#
# 依赖: bash, python3 (+PyYAML), ros2, 已编译的 install/robot_driver
# 用法: ./motor_fp.sh [--profile encos|tienkung|mix_canfd|mix_ethercat]
#                     [--type humanoid] [--config PATH]
#                     [--rate HZ] [--effort-scale F] [--no-driver] [--dry-run]

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}/.."
CONFIG_ROOT="${PROJECT_ROOT}/config"

ROBOT_TYPE="humanoid"
HARDWARE_PROFILE="tienkung"
MOTION_RATE="200"
EFFORT_SCALE="0.3"
NO_DRIVER=0
DRY_RUN=0
DRIVER_PID=""
DRIVER_LOG_FILE="/tmp/motor_fp_driver.log"
CLEANED_UP=0

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }
log_step()  { echo -e "${CYAN}[STEP]${NC} $*"; }

usage() {
    cat <<USAGE
用法: $0 [选项]

选项:
  -p, --profile PROFILE   硬件配置: encos | tienkung | mix_canfd | mix_ethercat (默认: tienkung)
  -t, --type TYPE         机器人类型 (默认: humanoid)
  -c, --config PATH       配置根目录 (默认: ./config)
      --rate HZ           命令帧发布频率 (默认: 200; 天工电机 100ms 无新帧即失能，需高频)
      --effort-scale F    τ_ff 限幅比例, 上限 = max_effort × F (默认: 0.3)
      --no-driver         复用已在运行的 robot_driver，不自行拉起
      --dry-run           不启动 driver、不连接 ROS，仅演示交互菜单
  -h, --help              显示帮助

示例:
  $0                                  # tienkung 真机力位混控调试
  $0 --profile encos --effort-scale 0.2
  $0 --dry-run                        # 无硬件走一遍菜单
USAGE
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -p|--profile)          HARDWARE_PROFILE="${2:-}"; shift 2 ;;
        -t|--type)             ROBOT_TYPE="${2:-}"; shift 2 ;;
        -c|--config)           CONFIG_ROOT="${2:-}"; shift 2 ;;
        --rate)                MOTION_RATE="${2:-}"; shift 2 ;;
        --effort-scale)        EFFORT_SCALE="${2:-}"; shift 2 ;;
        --no-driver)           NO_DRIVER=1; shift ;;
        --dry-run)             DRY_RUN=1; NO_DRIVER=1; shift ;;
        -h|--help)             usage; exit 0 ;;
        *) log_error "未知参数: $1"; usage; exit 1 ;;
    esac
done

case "${HARDWARE_PROFILE}" in
    encos|tienkung|mix_canfd|mix_ethercat) ;;
    *) log_error "profile 必须是 encos、tienkung、mix_canfd 或 mix_ethercat，当前: ${HARDWARE_PROFILE}"; exit 1 ;;
esac

profile_vendor() {
    case "${HARDWARE_PROFILE}" in
        mix_*) echo "mix" ;;
        *) echo "${HARDWARE_PROFILE}" ;;
    esac
}

# ===================== 配置文件解析 =====================
VENDOR="$(profile_vendor)"
if [[ "${ROBOT_TYPE}" == "humanoid" ]]; then
    JOINTS_CONFIG="${CONFIG_ROOT}/${ROBOT_TYPE}/robot_joints_${VENDOR}.yaml"
else
    JOINTS_CONFIG="${CONFIG_ROOT}/${ROBOT_TYPE}/robot_joints.yaml"
fi
HARDWARE_CONFIG="${CONFIG_ROOT}/${ROBOT_TYPE}/hw_${HARDWARE_PROFILE}.yaml"

[[ -f "${JOINTS_CONFIG}" ]]   || { log_error "缺少关节配置: ${JOINTS_CONFIG}"; exit 1; }
[[ -f "${HARDWARE_CONFIG}" ]] || { log_error "缺少硬件配置: ${HARDWARE_CONFIG}"; exit 1; }

DRIVER_BIN="${PROJECT_ROOT}/install/robot_driver/lib/robot_driver/robot_driver_node"
DRIVER_PARAMS="${PROJECT_ROOT}/install/robot_bringup/share/robot_bringup/config/driver_params.yaml"

# ===================== 环境 =====================
setup_ros_environment() {
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        source /opt/ros/humble/setup.bash
    elif [[ "${DRY_RUN}" -ne 1 ]]; then
        log_error "未找到 /opt/ros/humble/setup.bash，请先安装 ROS 2 Humble"
        exit 1
    fi
    if [[ -f "${PROJECT_ROOT}/install/setup.bash" ]]; then
        source "${PROJECT_ROOT}/install/setup.bash"
    elif [[ "${DRY_RUN}" -ne 1 ]]; then
        log_error "未找到 ${PROJECT_ROOT}/install/setup.bash，请先 colcon build"
        exit 1
    fi
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"
    if [[ "${DRY_RUN}" -ne 1 ]]; then
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        export CYCLONEDDS_URI="file://${CONFIG_ROOT}/cyclonedds_localhost.xml"
    fi
}

# ===================== driver 生命周期 =====================
check_no_conflicting_driver() {
    # 整机 run.sh / 另一个 motor*.sh / 残留 driver 进程在运行时，重复拉起会造成双主站抢总线。
    if pgrep -f "robot_driver_node --ros-args" >/dev/null 2>&1 \
       || pgrep -f "ros2 launch robot_bringup zl_robot" >/dev/null 2>&1; then
        log_error "检测到已有 robot_driver 或整机系统在运行。"
        log_error "请先执行 scripts/stop.sh，或确认复用现有 driver 时用 --no-driver。"
        exit 1
    fi
}

start_driver() {
    log_step "以运控模式 (config_mode=false) 启动 robot_driver..."

    if [[ ! -x "${DRIVER_BIN}" ]]; then
        log_error "未找到 driver 可执行文件: ${DRIVER_BIN}，请先编译 robot_driver"
        exit 1
    fi
    # robot_driver 需要 SCHED_FIFO 实时调度权限（setcap），否则 EtherCAT 周期时序无保障。
    if command -v getcap >/dev/null && ! getcap "${DRIVER_BIN}" | grep -q "cap_sys_nice"; then
        log_warn "driver 二进制缺少 cap_sys_nice (SCHED_FIFO) 权限，实机周期可能不稳。"
        log_warn "恢复方法: sudo ./scripts/install.sh  或  sudo setcap cap_net_raw,cap_net_admin,cap_sys_nice=eip ${DRIVER_BIN}"
    fi

    : > "${DRIVER_LOG_FILE}"
    taskset -c 3 "${DRIVER_BIN}" --ros-args \
        -r __ns:=/robot \
        ${DRIVER_PARAMS:+--params-file "${DRIVER_PARAMS}"} \
        -p use_sim_time:=false \
        -p robot_joints_config:="${JOINTS_CONFIG}" \
        -p hardware_config:="${HARDWARE_CONFIG}" \
        -p config_mode:=false \
        > "${DRIVER_LOG_FILE}" 2>&1 &
    DRIVER_PID=$!
    disown "${DRIVER_PID}" 2>/dev/null || true

    log_info "driver PID=${DRIVER_PID}, 日志: ${DRIVER_LOG_FILE}"
    local timeout=20
    while (( timeout > 0 )); do
        if ! ps -p "${DRIVER_PID}" >/dev/null 2>&1; then
            log_error "driver 启动失败，日志如下:"
            tail -n 60 "${DRIVER_LOG_FILE}" || true
            exit 1
        fi
        if ros2 topic list 2>/dev/null | grep -q "^/robot/driver/joint_states$"; then
            log_info "driver 运控模式就绪 (/robot/driver/joint_states 已发布)"
            return 0
        fi
        sleep 1
        ((timeout--))
    done
    log_error "等待 driver 超时，查看日志: ${DRIVER_LOG_FILE}"
    tail -n 60 "${DRIVER_LOG_FILE}" || true
    exit 1
}

stop_driver() {
    [[ "${CLEANED_UP}" -eq 1 ]] && return
    CLEANED_UP=1
    if [[ -n "${DRIVER_PID}" ]] && ps -p "${DRIVER_PID}" >/dev/null 2>&1; then
        log_info "停止 robot_driver (PID=${DRIVER_PID})..."
        kill -TERM "${DRIVER_PID}" 2>/dev/null || true
        sleep 1
        kill -KILL "${DRIVER_PID}" 2>/dev/null || true
        wait "${DRIVER_PID}" 2>/dev/null || true
    fi
    log_info "清理完成"
}

trap stop_driver EXIT
trap 'stop_driver; exit 130' INT TERM

# ===================== 入口 =====================
main() {
    setup_ros_environment

    for cmd in python3 ros2; do
        command -v "${cmd}" >/dev/null || { log_error "缺少依赖: ${cmd}"; exit 1; }
    done
    python3 -c "import yaml" 2>/dev/null || { log_error "缺少 PyYAML: sudo apt install python3-yaml"; exit 1; }

    log_info "motor_fp.sh 配置:"
    log_info "  robot_type       = ${ROBOT_TYPE}"
    log_info "  hardware_profile = ${HARDWARE_PROFILE}"
    log_info "  robot_joints     = ${JOINTS_CONFIG}"
    log_info "  hardware_config  = ${HARDWARE_CONFIG}"
    log_info "  effort_scale     = ${EFFORT_SCALE}"

    if [[ "${DRY_RUN}" -eq 1 ]]; then
        log_warn "dry-run 模式：不启动 driver，不发送真实命令"
    elif [[ "${NO_DRIVER}" -eq 1 ]]; then
        log_warn "--no-driver：复用已运行的 robot_driver，请确认其为运控模式"
    else
        check_no_conflicting_driver
        start_driver
    fi

    python3 "${SCRIPT_DIR}/motor_fp.py" \
        --joints-config "${JOINTS_CONFIG}" \
        --hw-config "${HARDWARE_CONFIG}" \
        --rate "${MOTION_RATE}" \
        --effort-scale "${EFFORT_SCALE}" \
        $([[ "${DRY_RUN}" -eq 1 ]] && echo "--dry-run")
}

main "$@"
