#!/usr/bin/env bash
# motor.sh - 统一电机配置工具 (单脚本，运行时解析 YAML)
# 依赖: bash, python3 (+ PyYAML), ros2
# 用法: ./motor.sh [--profile encos|tienkung|mix_canfd|mix_ethercat] [--type humanoid] [--config PATH]

set -eo pipefail

# ===================== 配置参数 =====================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}/.."
CONFIG_ROOT="${PROJECT_ROOT}/config"

# 默认配置。可直接修改这里，也可通过命令行参数临时覆盖。
#
# 可选值：
#   ROBOT_TYPE  humanoid | dog
#   HARDWARE_PROFILE encos | tienkung | mix_canfd | mix_ethercat
#                    -> config/<robot>/hw_<profile>.yaml
#   CONFIG_ROOT 配置根目录，默认使用当前工程的 config
ROBOT_TYPE="humanoid"
HARDWARE_PROFILE="tienkung"

# 运行时变量
DRIVER_PID=""
DRIVER_TAIL_PID=""
DRIVER_LOG_FILE="/tmp/motor_driver.log"
JOINT_DATA=()
CLEANED_UP=0
MOTOR_LAST_POSITION_VALUE=""
MOTOR_LAST_CALL_OUTPUT=""

# ===================== 颜色输出 =====================
RED='\033[0;31m'
GREEN='\033[0;32m'
BRIGHT_GREEN='\033[92m'
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
  -p, --profile PROFILE 硬件配置: encos | tienkung | mix_canfd | mix_ethercat (默认: mix_canfd)
  -t, --type TYPE       机器人类型 (默认: humanoid)
  -c, --config PATH     配置根目录 (默认: ./config)
  -h, --help            显示帮助

示例:
  $0                                    # 默认混装 CANFD
  $0 --profile encos                    # 仅 Encos
  $0 --profile tienkung --type humanoid # 指定类型和配置
  $0 --profile mix_ethercat             # 混装 EtherCAT gateway 配置
USAGE
}

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case "$1" in
        -p|--profile)
            HARDWARE_PROFILE="${2:-}"
            shift 2
            ;;
        -t|--type)
            ROBOT_TYPE="${2:-}"
            shift 2
            ;;
        -c|--config)
            CONFIG_ROOT="${2:-}"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            log_error "未知参数: $1"
            usage
            exit 1
            ;;
    esac
done

validate_hardware_profile_or_die() {
    case "${HARDWARE_PROFILE}" in
        encos|tienkung|mix_canfd|mix_ethercat)
            ;;
        *)
            log_error "profile 必须是 encos、tienkung、mix_canfd 或 mix_ethercat，当前: ${HARDWARE_PROFILE}"
            usage
            exit 1
            ;;
    esac
}

profile_vendor() {
    case "${HARDWARE_PROFILE}" in
        mix_*) echo "mix" ;;
        *) echo "${HARDWARE_PROFILE}" ;;
    esac
}

validate_hardware_profile_or_die

# ===================== Python 解析 YAML 生成关节映射 =====================
load_joint_data() {
    log_step "解析配置文件..."

    # 创建临时 Python 脚本文件
    local tmp_py
    tmp_py=$(mktemp /tmp/motor_parse_XXXXXX.py)
    
    cat > "$tmp_py" << 'PYEOF'
import sys
import yaml
from pathlib import Path

CONFIG_ROOT = Path(sys.argv[1])
ROBOT_TYPE = sys.argv[2]
HARDWARE_PROFILE = sys.argv[3]
PROFILE_VENDOR = "mix" if HARDWARE_PROFILE.startswith("mix_") else HARDWARE_PROFILE

def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)

if ROBOT_TYPE == "humanoid":
    robot_joints_path = CONFIG_ROOT / ROBOT_TYPE / f"robot_joints_{PROFILE_VENDOR}.yaml"
else:
    robot_joints_path = CONFIG_ROOT / ROBOT_TYPE / "robot_joints.yaml"

if not robot_joints_path.exists():
    raise FileNotFoundError(f"robot joints config not found: {robot_joints_path}")

data = load_yaml(robot_joints_path)
robot_joints = data.get("robot_joints", {}).get("joints", [])

# 解析厂商硬件配置
hw_tienkung = {}
hw_encos = {}
hw_mix = {}

tienkung_path = CONFIG_ROOT / ROBOT_TYPE / "hw_tienkung.yaml"
if tienkung_path.exists():
    hw = load_yaml(tienkung_path).get("hardware", {})
    for j in hw.get("joints", []):
        name = j.get("name")
        tj = j.get("tienkung", {})
        hw_tienkung[name] = {
            "vendor": "tienkung",
            "motor_id": j.get("motor_id"),
            "direction": j.get("direction", 1),
            "offset": j.get("offset", 0.0),
            "ratio": j.get("ratio", 1.0),
            "slave_id": tj.get("slave_id"),
            "bus": tj.get("bus"),
            "slot": tj.get("slot"),
            "kt": tj.get("kt"),
        }

encos_path = CONFIG_ROOT / ROBOT_TYPE / "hw_encos.yaml"
if encos_path.exists():
    hw = load_yaml(encos_path).get("hardware", {})
    for j in hw.get("joints", []):
        name = j.get("name")
        ej = j.get("encos", {})
        hw_encos[name] = {
            "vendor": "encos",
            "motor_id": j.get("motor_id"),
            "direction": j.get("direction", 1),
            "offset": j.get("offset", 0.0),
            "ratio": j.get("ratio", 1.0),
            "slave_id": ej.get("slave_id"),
            "passage": ej.get("passage"),
            "kt": ej.get("kt"),
        }

mix_path = CONFIG_ROOT / ROBOT_TYPE / f"hw_{HARDWARE_PROFILE}.yaml"
if mix_path.exists():
    hw = load_yaml(mix_path).get("hardware", {})
    for j in hw.get("joints", []):
        name = j.get("name")
        vendor = j.get("vendor")
        base = {
            "vendor": vendor,
            "motor_id": j.get("motor_id"),
            "direction": j.get("direction", 1),
            "offset": j.get("offset", 0.0),
            "ratio": j.get("ratio", 1.0),
            "canfd_bus": j.get("canfd", {}).get("bus", ""),
            "slave_id": "",
            "bus": "",
            "slot": "",
            "passage": "",
            "kt": "",
        }
        if vendor == "tienkung":
            tj = j.get("tienkung", {})
            gw = j.get("gateway", {})
            base.update({
                "slave_id": gw.get("slave_id", tj.get("slave_id")),
                "bus": gw.get("bus", tj.get("bus")),
                "slot": gw.get("slot", tj.get("slot")),
                "kt": tj.get("kt"),
            })
        elif vendor == "encos":
            ej = j.get("encos", {})
            gw = j.get("gateway", {})
            base.update({
                "slave_id": gw.get("slave_id", ej.get("slave_id")),
                "bus": gw.get("bus", ""),
                "slot": gw.get("slot", ""),
                "passage": ej.get("passage"),
                "kt": ej.get("kt"),
            })
        hw_mix[name] = base

# 合并
all_mappings = []
for idx, rj in enumerate(robot_joints, 1):
    name = rj.get("name")
    group = rj.get("group", "")
    enable_default = bool(rj.get("enable_default", True))
    if PROFILE_VENDOR == "mix" and name in hw_mix:
        hw = hw_mix[name]
        all_mappings.append({
            "index": idx, "name": name, "group": group,
            "enable_default": enable_default,
            "vendor": hw["vendor"], "motor_id": hw["motor_id"],
            "direction": hw["direction"], "offset": hw["offset"],
            "ratio": hw["ratio"], "slave_id": hw["slave_id"],
            "bus": hw["bus"], "slot": hw["slot"], "passage": hw["passage"],
            "kt": hw["kt"], "canfd_bus": hw["canfd_bus"],
        })
        continue
    if name in hw_tienkung:
        hw = hw_tienkung[name]
        all_mappings.append({
            "index": idx, "name": name, "group": group,
            "enable_default": enable_default,
            "vendor": "tienkung", "motor_id": hw["motor_id"],
            "direction": hw["direction"], "offset": hw["offset"],
            "ratio": hw["ratio"], "slave_id": hw["slave_id"],
            "bus": hw["bus"], "slot": hw["slot"], "passage": "",
            "kt": hw["kt"], "canfd_bus": "",
        })
    if name in hw_encos:
        hw = hw_encos[name]
        all_mappings.append({
            "index": idx, "name": name, "group": group,
            "enable_default": enable_default,
            "vendor": "encos", "motor_id": hw["motor_id"],
            "direction": hw["direction"], "offset": hw["offset"],
            "ratio": hw["ratio"], "slave_id": hw["slave_id"],
            "bus": "", "slot": "", "passage": hw["passage"],
            "kt": hw["kt"], "canfd_bus": "",
        })

all_mappings.sort(key=lambda x: x["index"])

# 过滤 vendor
if PROFILE_VENDOR != "mix":
    all_mappings = [m for m in all_mappings if m["vendor"] == PROFILE_VENDOR]

# 输出 bash 数组格式
for m in all_mappings:
    enabled = "true" if m["enable_default"] else "false"
    line = f"{m['index']}|{m['name']}|{m['group']}|{m['vendor']}|{m['motor_id']}|{m['direction']}|{m['offset']}|{m['ratio']}|{m['slave_id']}|{m['bus']}|{m['slot']}|{m['passage']}|{m['kt']}|{enabled}|{m.get('canfd_bus', '')}"
    print(line)
PYEOF

    # 执行 Python 解析
    JOINT_DATA=()
    while IFS= read -r line; do
        JOINT_DATA+=("$line")
    done < <(python3 "$tmp_py" "$CONFIG_ROOT" "$ROBOT_TYPE" "$HARDWARE_PROFILE" 2>/dev/null)

    rm -f "$tmp_py"

    if [[ ${#JOINT_DATA[@]} -eq 0 ]]; then
        log_error "未找到匹配 ${HARDWARE_PROFILE} 的关节映射，请检查配置文件"
        exit 1
    fi
    log_info "已加载 ${#JOINT_DATA[@]} 个关节"
}

# ===================== 启动/停止 config_mode driver =====================
start_driver() {
    log_step "启动 config_mode driver (后台)..."

    if [[ -f "${PROJECT_ROOT}/install/setup.bash" ]]; then
        source "${PROJECT_ROOT}/install/setup.bash"
    else
        log_warn "未找到 install/setup.bash，尝试使用系统 ROS 环境"
    fi

    : > "${DRIVER_LOG_FILE}"
    local vendor_profile
    vendor_profile="$(profile_vendor)"

    log_info "driver 配置:"
    log_info "  robot_type      = ${ROBOT_TYPE}"
    log_info "  hardware_profile= ${HARDWARE_PROFILE}"
    log_info "  config_root     = ${CONFIG_ROOT}"
    if [[ "${ROBOT_TYPE}" == "humanoid" ]]; then
        ROBOT_JOINTS_CONFIG="${CONFIG_ROOT}/${ROBOT_TYPE}/robot_joints_${vendor_profile}.yaml"
    else
        ROBOT_JOINTS_CONFIG="${CONFIG_ROOT}/${ROBOT_TYPE}/robot_joints.yaml"
    fi

    log_info "  robot_joints    = ${ROBOT_JOINTS_CONFIG}"
    HARDWARE_CONFIG="${CONFIG_ROOT}/${ROBOT_TYPE}/hw_${HARDWARE_PROFILE}.yaml"
    log_info "  hardware_config = ${HARDWARE_CONFIG}"
    log_info "  log_file        = ${DRIVER_LOG_FILE}"

    tail -n0 -F "${DRIVER_LOG_FILE}" &
    DRIVER_TAIL_PID=$!

    ros2 launch robot_bringup zl_robot_driver.launch.py \
        arg_robot_type:="${ROBOT_TYPE}" \
        arg_hardware_profile:="${HARDWARE_PROFILE}" \
        arg_config_root:="${CONFIG_ROOT}" \
        > "${DRIVER_LOG_FILE}" 2>&1 &

    DRIVER_PID=$!

    log_info "等待 driver 启动 (PID: ${DRIVER_PID})..."
    sleep 3

    if ! ps -p "${DRIVER_PID}" >/dev/null 2>&1; then
        log_error "Driver 启动失败，查看日志: ${DRIVER_LOG_FILE}"
        tail -n 80 "${DRIVER_LOG_FILE}"
        exit 1
    fi

    local timeout=15
    while [[ ${timeout} -gt 0 ]]; do
        if ros2 service list 2>/dev/null | grep -q "tienkung\|encos\|mix"; then
            log_info "Driver 服务已就绪"
            log_info "  你现在可以在另一个终端直接调用 /robot/driver/encos/* srv"
            return 0
        fi
        sleep 1
        ((timeout--))
    done

    log_error "Driver 服务启动超时，查看日志: ${DRIVER_LOG_FILE}"
    tail -n 80 "${DRIVER_LOG_FILE}"
    exit 1
}

stop_driver() {
    if [[ "${CLEANED_UP}" -eq 1 ]]; then
        return
    fi
    CLEANED_UP=1

    log_info "停止 driver (PID: ${DRIVER_PID})..."
    if [[ -n "${DRIVER_PID}" ]]; then
        kill -TERM -- "-${DRIVER_PID}" 2>/dev/null || true
        kill -TERM "${DRIVER_PID}" 2>/dev/null || true
        sleep 1
        kill -KILL -- "-${DRIVER_PID}" 2>/dev/null || true
        kill -KILL "${DRIVER_PID}" 2>/dev/null || true
        wait "${DRIVER_PID}" 2>/dev/null || true
    fi

    # 兜底回收 launch、节点和日志 tail，避免 Ctrl+C 后留下孤儿进程。
    pkill -f "ros2 launch robot_bringup zl_robot_driver.launch.py" 2>/dev/null || true
    pkill -f "robot_driver/lib/robot_driver/robot_driver_node" 2>/dev/null || true
    pkill -f "robot_driver_node --ros-args" 2>/dev/null || true
    pkill -f "zl_robot_driver" 2>/dev/null || true
    if [[ -n "${DRIVER_TAIL_PID}" ]]; then
        kill -TERM "${DRIVER_TAIL_PID}" 2>/dev/null || true
        kill -KILL "${DRIVER_TAIL_PID}" 2>/dev/null || true
        wait "${DRIVER_TAIL_PID}" 2>/dev/null || true
    fi

    log_info "Driver 已停止"
}

cleanup() {
    stop_driver
    log_info "清理完成"
}

trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

# ===================== Service 调用封装 =====================
# 失败后最多额外重试 MOTOR_RETRY_COUNT 次。以后如果想增加重复次数，只改这个变量即可。
MOTOR_RETRY_COUNT=3
MOTOR_RETRY_DELAY_SEC=0.2

service_call_with_retry_impl() {
    local label="$1"
    local success_pattern="$2"
    local silent="${3:-0}"
    shift 3

    local attempt=0
    local rc=0
    local output=""
    local tmp_output=""

    while true; do
        tmp_output=$(mktemp /tmp/motor_call_XXXXXX.log)
        set +e
        if [[ "${silent}" -eq 1 ]]; then
            "$@" >"${tmp_output}" 2>&1
            rc=$?
        else
            "$@" 2>&1 | tee "${tmp_output}"
            rc=${PIPESTATUS[0]}
        fi
        set -e
        output="$(cat "${tmp_output}")"
        rm -f "${tmp_output}"
        MOTOR_LAST_CALL_OUTPUT="${output}"

        if [[ "${rc}" -eq 0 && "${output}" == *"${success_pattern}"* ]]; then
            return 0
        fi

        if (( attempt >= MOTOR_RETRY_COUNT )); then
            return 1
        fi

        ((attempt++))
        sleep "${MOTOR_RETRY_DELAY_SEC}"
    done
}

service_call_with_retry() {
    service_call_with_retry_impl "$1" "$2" 0 "${@:3}"
}

service_call_with_retry_silent() {
    service_call_with_retry_impl "$1" "$2" 1 "${@:3}"
}

joint_enable_default() {
    local line="$1"
    local enable_default
    enable_default=$(echo "${line}" | cut -d'|' -f14)
    [[ -z "${enable_default}" || "${enable_default}" == "true" || "${enable_default}" == "True" || "${enable_default}" == "1" ]]
}

skip_disabled_joint() {
    local line="$1"
    local operation="$2"
    local name
    name=$(echo "${line}" | cut -d'|' -f2)
    # log_warn "跳过 ${name}: enable_default=false，不执行${operation}"
}

call_tienkung_query_status() {
    local joint_name="$1"
    service_call_with_retry_silent "Tienkung 读取位置(${joint_name})" "success=True" \
        ros2 service call /robot/driver/tienkung/query_status \
        robot_interfaces/srv/TienkungQueryStatus \
        "{joint_name: '${joint_name}', timeout_sec: 2.0}"
}

call_tienkung_set_zero() {
    local joint_name="$1"
    service_call_with_retry_silent "Tienkung 标零(${joint_name})" "success=True" \
        ros2 service call /robot/driver/tienkung/set_zero \
        robot_interfaces/srv/TienkungSetZero \
        "{joint_name: '${joint_name}', timeout_sec: 3.0}"
}

call_tienkung_query_id() {
    service_call_with_retry "Tienkung 广播查询 ID" "success=True" \
        ros2 service call /robot/driver/tienkung/query_id \
        robot_interfaces/srv/TienkungQueryID \
        "{timeout_sec: 1.0}"
}

call_encos_get_motor_id() {
    local joint_name="$1"
    service_call_with_retry "Encos 读取电机 ID(${joint_name})" "success=True" \
        ros2 service call /robot/driver/encos/get_motor_id \
        robot_interfaces/srv/EncosGetMotorId \
        "{joint_name: '${joint_name}', timeout_sec: 1.0}"
}

call_encos_set_motor_zero() {
    local joint_name="$1"
    service_call_with_retry_silent "Encos 标零(${joint_name})" "success=True" \
        ros2 service call /robot/driver/encos/set_motor_zero \
        robot_interfaces/srv/EncosSetMotorZero \
        "{joint_name: '${joint_name}', timeout_sec: 3.0}"
}

call_encos_get_motor_param() {
    local joint_name="$1"
    local motor_id="$2"
    service_call_with_retry_silent "Encos 读取参数(${joint_name}, motor_id=${motor_id})" "success=True" \
        ros2 service call /robot/driver/encos/get_motor_param \
        robot_interfaces/srv/EncosGetMotorParam \
        "{joint_name: '${joint_name}', param_code: 1, timeout_sec: 1.0}"
}

call_mix_query_joint() {
    local joint_name="$1"
    service_call_with_retry_silent "Mix 查询关节(${joint_name})" "success=True" \
        ros2 service call /robot/driver/mix/query_joint \
        robot_interfaces/srv/MixQueryJoint \
        "{joint_name: '${joint_name}', timeout_sec: 2.0}"
}

call_mix_set_zero() {
    local joint_name="$1"
    service_call_with_retry_silent "Mix 标零(${joint_name})" "success=True" \
        ros2 service call /robot/driver/mix/set_zero \
        robot_interfaces/srv/MixSetZero \
        "{joint_name: '${joint_name}', timeout_sec: 3.0}"
}

call_mix_query_motor_id() {
    local joint_name="$1"
    service_call_with_retry "Mix 读取电机 ID(${joint_name})" "success=True" \
        ros2 service call /robot/driver/mix/query_motor_id \
        robot_interfaces/srv/MixQueryMotorId \
        "{joint_name: '${joint_name}', timeout_sec: 1.0}"
}

extract_reading_value() {
	local output="$1"
	local vendor="$2"
	local pattern
    local vendor_profile
    vendor_profile="$(profile_vendor)"

    if [[ "${vendor_profile}" == "mix" || "${vendor}" == "tienkung" ]]; then
        pattern='position=[-+]?[0-9]*([.][0-9]+)?([eE][-+]?[0-9]+)?'
    else
        pattern='value=[-+]?[0-9]*([.][0-9]+)?([eE][-+]?[0-9]+)?'
    fi

    local match
    match="$(grep -oE "${pattern}" <<< "${output}" | tail -1 || true)"
    if [[ -z "${match}" ]]; then
        return 1
    fi

    echo "${match#*=}"
}

format_position_value() {
    local value="$1"
    if [[ "${value}" == "N/A" ]]; then
        echo "N/A"
        return 0
    fi

    local sign="+"
    local abs_value="${value}"
    if [[ "${value}" == -* ]]; then
        sign="-"
        abs_value="${value#-}"
    elif [[ "${value}" == +* ]]; then
        abs_value="${value#+}"
    fi

    abs_value="$(awk -v v="${abs_value}" 'BEGIN { printf "%.6f", v }')"
    if [[ "${sign}" == "-" ]]; then
        printf "${RED}-${NC}%s" "${abs_value}"
    else
        printf "${BRIGHT_GREEN}+${NC}%s" "${abs_value}"
    fi
}

read_joint_position_value() {
    local line="$1"
    local vendor
    vendor=$(echo "${line}" | cut -d'|' -f4)

    local output value
    if ! get_position "${line}"; then
        return 1
    fi

    output="${MOTOR_LAST_CALL_OUTPUT}"

    if ! value="$(extract_reading_value "${output}" "${vendor}")"; then
        return 1
    fi

    MOTOR_LAST_POSITION_VALUE="${value}"
}

print_joint_position_summary() {
    local line="$1"
    local success="$2"
    local position="$3"

    print_read_result_summary "${line}" "${success}" "${position}"
}

print_zero_result_summary() {
    local line="$1"
    local success="$2"
    local before_pos="$3"
    local after_pos="$4"

    local index name vendor motor_id
    index=$(echo "${line}" | cut -d'|' -f1)
    name=$(echo "${line}" | cut -d'|' -f2)
    vendor=$(echo "${line}" | cut -d'|' -f4)
    motor_id=$(echo "${line}" | cut -d'|' -f5)

    [[ -n "${before_pos}" ]] || before_pos="N/A"
    [[ -n "${after_pos}" ]] || after_pos="N/A"
    before_pos="$(format_position_value "${before_pos}")"
    after_pos="$(format_position_value "${after_pos}")"

    printf "${CYAN}[RESULT]${NC} #%s %-20s %-10s motor_id=%-4s success=%-3s before=%-12s after=%s\n" \
        "${index}" "${name}" "${vendor^^}" "${motor_id}" "${success}" "${before_pos}" "${after_pos}"
}

print_read_result_summary() {
    local line="$1"
    local success="$2"
    local position="$3"

    local index name vendor motor_id
    index=$(echo "${line}" | cut -d'|' -f1)
    name=$(echo "${line}" | cut -d'|' -f2)
    vendor=$(echo "${line}" | cut -d'|' -f4)
    motor_id=$(echo "${line}" | cut -d'|' -f5)

    [[ -n "${position}" ]] || position="N/A"
    position="$(format_position_value "${position}")"

    printf "${CYAN}[RESULT]${NC} #%s %-20s %-10s motor_id=%-4s success=%-3s position=%s\n" \
        "${index}" "${name}" "${vendor^^}" "${motor_id}" "${success}" "${position}"
}

# ===================== Vendor 无关封装 =====================
get_position() {
	local line="$1"
	local vendor=$(echo "${line}" | cut -d'|' -f4)
	local name=$(echo "${line}" | cut -d'|' -f2)
    local vendor_profile
    vendor_profile="$(profile_vendor)"

    if [[ "${vendor_profile}" == "mix" ]]; then
        call_mix_query_joint "${name}"
    elif [[ "${vendor}" == "tienkung" ]]; then
        call_tienkung_query_status "${name}"
    else
        local motor_id=$(echo "${line}" | cut -d'|' -f5)
        call_encos_get_motor_param "${name}" "${motor_id}"
    fi
}

set_zero() {
	local line="$1"
	local vendor=$(echo "${line}" | cut -d'|' -f4)
	local name=$(echo "${line}" | cut -d'|' -f2)
    local vendor_profile
    vendor_profile="$(profile_vendor)"

    if [[ "${vendor_profile}" == "mix" ]]; then
        call_mix_set_zero "${name}"
    elif [[ "${vendor}" == "tienkung" ]]; then
        call_tienkung_set_zero "${name}"
    else
        call_encos_set_motor_zero "${name}"
    fi
}

# ===================== 颜色/工具函数 =====================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }
log_step()  { echo -e "${CYAN}[STEP]${NC} $*"; }

select_joint() {
    local prompt="${1:-输入关节号}"
    local filter_vendor="${2:-}"

    while true; do
        show_joint_list "${prompt}" "${filter_vendor}" >&2
        read -r -p "${prompt}: " sel
        [[ "${sel}" == "q" || "${sel}" == "Q" ]] && return 1

        if [[ "${sel}" =~ ^[0-9]+$ ]]; then
            for line in "${JOINT_DATA[@]}"; do
                local idx=$(echo "${line}" | cut -d'|' -f1)
                local vendor=$(echo "${line}" | cut -d'|' -f4)
                if [[ "${idx}" == "${sel}" && ( -z "${filter_vendor}" || "${vendor}" == "${filter_vendor}" ) ]]; then
                    echo "${line}"
                    return 0
                fi
            done
        fi
        log_error "无效输入！请输入有效关节号"
    done
}

show_joint_list() {
    local title="${1:-关节选择}"
    local filter_vendor="${2:-}"

    echo -e "${CYAN}=============================================${NC}"
    echo -e "${CYAN}        关节选择 - ${title}${NC}"
    echo -e "${CYAN}=============================================${NC}"
    printf "  %-5s %-32s %-10s %-10s %-8s %-8s %-8s %-8s %-8s %s\n" \
        "序号" "关节名" "分组" "厂商" "motor_id" "方向" "偏移" "比例" "使能" "硬件信息"

    local printed=0
    for line in "${JOINT_DATA[@]}"; do
        IFS='|' read -r index name group vendor motor_id direction offset ratio slave_id bus slot passage kt enable_default canfd_bus <<< "${line}"

        if [[ -n "${filter_vendor}" && "${vendor}" != "${filter_vendor}" ]]; then
            continue
        fi

        local hw_info=""
        if [[ "${vendor}" == "tienkung" ]]; then
            hw_info="slave=${slave_id} bus=${bus} slot=${slot}"
        else
            hw_info="slave=${slave_id} passage=${passage}"
        fi
        if [[ "${HARDWARE_PROFILE}" == mix_* ]]; then
            hw_info="canfd.bus=${canfd_bus} ${hw_info}"
        fi

        printf "  %2d.   %-28s %-10s %-10s %-8s %-8s %-8s %-8s %-8s %s\n" \
            "${index}" "${name}" "${group}" "${vendor^^}" \
            "${motor_id}" "${direction}" "${offset}" "${ratio}" "${enable_default}" "${hw_info}"
        ((printed++))
    done

    if [[ ${printed} -eq 0 ]]; then
        echo "  (无匹配的 ${filter_vendor} 关节)"
    fi

    echo -e "${CYAN}=============================================${NC}"
    echo "q 返回主菜单"
}

# ===================== 功能实现 =====================
action_read_position() {
    while true; do
        local line
        if ! line=$(select_joint "选择关节读取位置"); then return; fi

        log_step "读取位置..."
        local name=$(echo "${line}" | cut -d'|' -f2)
        local vendor=$(echo "${line}" | cut -d'|' -f4)

        echo -e "${CYAN}=============================================${NC}"
        echo -e "       关节位置查询: ${name} (${vendor^^})"
        echo -e "       读取 ${name} (${vendor^^}) 位置"
        echo -e "${CYAN}=============================================${NC}"

        local position=""
        local read_success="no"
        if ! joint_enable_default "${line}"; then
            skip_disabled_joint "${line}" "读取位置"
            read_success="skip"
        elif read_joint_position_value "${line}"; then
            read_success="yes"
            position="${MOTOR_LAST_POSITION_VALUE}"
        else
            log_warn "读取失败，可在当前菜单重试"
        fi
        print_joint_position_summary "${line}" "${read_success}" "${position}"
    done
}

action_set_zero() {
    while true; do
        local line
        if ! line=$(select_joint "选择关节标零"); then return; fi

        local vendor=$(echo "${line}" | cut -d'|' -f4)
        local name=$(echo "${line}" | cut -d'|' -f2)

        if ! joint_enable_default "${line}"; then
            skip_disabled_joint "${line}" "标零"
            print_zero_result_summary "${line}" "skip" "N/A" "N/A"
            continue
        fi

        echo -e "${CYAN}=============================================${NC}"
        echo -e "       关节零位设置: ${name} (${vendor^^})"
        echo -e "       ${name} 关节标零"
        echo -e "${CYAN}=============================================${NC}"
        echo -e "${YELLOW}⚠ 即将对 ${name} 执行标零操作${NC}"
        read -r -p "确认继续? (y/N): " confirm
        [[ "${confirm}" =~ ^[Yy]$ ]] || continue

        log_step "执行标零..."
        local before_pos=""
        local after_pos=""
        local zero_success="no"

        if read_joint_position_value "${line}"; then
            before_pos="${MOTOR_LAST_POSITION_VALUE}"
        else
            log_warn "标零前读取失败"
        fi

        if set_zero "${line}"; then
            zero_success="yes"
        else
            log_error "标零失败"
        fi

        if read_joint_position_value "${line}"; then
            after_pos="${MOTOR_LAST_POSITION_VALUE}"
        else
            log_warn "标零后读取失败"
        fi

        print_zero_result_summary "${line}" "${zero_success}" "${before_pos}" "${after_pos}"
    done
}

action_read_motor_id() {
    echo -e "${CYAN}=============================================${NC}"
    echo -e "       查询电机 ID"
    echo -e "${CYAN}=============================================${NC}"

    local vendor_profile
    vendor_profile="$(profile_vendor)"

    if [[ "${vendor_profile}" == "mix" ]]; then
        local line name
        if ! line=$(select_joint "选择关节查询电机 ID"); then return; fi
        name=$(echo "${line}" | cut -d'|' -f2)
        if ! call_mix_query_motor_id "${name}"; then
            log_warn "Mix 查询电机 ID 失败"
        fi
    elif [[ "${vendor_profile}" == "tienkung" ]]; then
        echo -e "\n[${GREEN}Tienkung${NC}] 广播查询..."
        if ! call_tienkung_query_id; then
            log_warn "Tienkung 广播查询失败"
        fi
    elif [[ "${vendor_profile}" == "encos" ]]; then
        local line name
        if ! line=$(select_joint "选择关节查询电机 ID" "encos"); then return; fi
        name=$(echo "${line}" | cut -d'|' -f2)
        if ! call_encos_get_motor_id "${name}"; then
                log_warn "Encos 读取电机 ID 失败"
        fi
    fi

    echo
    read -r -p "按回车继续..."
}

action_batch_read_position() {
    log_step "批量读取关节位置..."

    echo -e "${CYAN}=============================================${NC}"
    echo -e "       批量读取位置 (${#JOINT_DATA[@]} 个关节)"
    echo -e "${CYAN}=============================================${NC}"
    printf "  %-4s %-20s %-10s %-8s %s\n" "序号" "关节名" "厂商" "motor_id" "位置"

    for line in "${JOINT_DATA[@]}"; do
        local index=$(echo "${line}" | cut -d'|' -f1)
        local name=$(echo "${line}" | cut -d'|' -f2)
        local vendor=$(echo "${line}" | cut -d'|' -f4)
        local motor_id=$(echo "${line}" | cut -d'|' -f5)
        local position=""
        local read_success="no"
        if ! joint_enable_default "${line}"; then
            skip_disabled_joint "${line}" "读取位置"
            read_success="skip"
        elif read_joint_position_value "${line}"; then
            read_success="yes"
            position="${MOTOR_LAST_POSITION_VALUE}"
        fi
        print_joint_position_summary "${line}" "${read_success}" "${position}"
        sleep 0.05
    done

    echo
    read -r -p "按回车继续..."
}

action_batch_set_zero() {
    echo -e "${RED}⚠⚠⚠ 警告: 将对所有 ${#JOINT_DATA[@]} 个关节执行标零操作 ⚠⚠⚠${NC}"
    echo "这会导致机器人关节位置归零，请确保机器人处于安全状态！"
    read -r -p "输入 'YES' 确认执行: " confirm
    [[ "${confirm}" == "YES" ]] || { log_error "已取消"; return; }

    log_step "批量关节标零..."
    printf "  %-4s %-20s %-10s %-8s %s\n" "序号" "关节名" "厂商" "motor_id" "状态"

    for line in "${JOINT_DATA[@]}"; do
        local before_pos=""
        local after_pos=""
        local zero_success="no"

        if ! joint_enable_default "${line}"; then
            skip_disabled_joint "${line}" "标零"
            print_zero_result_summary "${line}" "skip" "N/A" "N/A"
            sleep 0.5
            continue
        fi

        if read_joint_position_value "${line}"; then
            before_pos="${MOTOR_LAST_POSITION_VALUE}"
        fi

        if set_zero "${line}"; then
            zero_success="yes"
        fi

        if read_joint_position_value "${line}"; then
            after_pos="${MOTOR_LAST_POSITION_VALUE}"
        fi

        print_zero_result_summary "${line}" "${zero_success}" "${before_pos}" "${after_pos}"
        sleep 0.5
    done

    log_info "批量关节标零完成"
    read -r -p "按回车继续..."
}

# ===================== 主菜单循环 =====================
main_loop() {
    while true; do
        echo
        echo -e "${CYAN}=============================================${NC}"
        echo -e "${CYAN}        zl_robot 电机配置工具${NC}"
        echo -e "${CYAN}        Robot: ${ROBOT_TYPE} | Profile: ${HARDWARE_PROFILE}${NC}"
        echo -e "${CYAN}=============================================${NC}"
        echo
        echo -e "  ${GREEN}1.${NC} 📍 读取关节位置"
        echo -e "  ${GREEN}2.${NC} 🔧 设置关节标零"
        echo -e "  ${GREEN}3.${NC} 📋 批量读取关节位置"
        echo -e "  ${GREEN}4.${NC} 🔄 批量设置关节标零 (需确认)"
        echo -e "  ${GREEN}5.${NC} 🆔 读取电机 ID (广播查询)"
        echo -e "  ${RED}q.${NC} 退出程序"
        echo -e "${CYAN}=============================================${NC}"
        echo -n "请选择功能: "

        read -r opt
        case "${opt}" in
            1) action_read_position ;;
            2) action_set_zero ;;
            3) action_batch_read_position ;;
            4) action_batch_set_zero ;;
            5) action_read_motor_id ;;
            q|Q) log_info "退出程序"; exit 0 ;;
            *) log_error "无效选项"; sleep 1 ;;
        esac
    done
}

# ===================== 入口 =====================
main() {
    log_info "motor.sh 启动中..."

    for cmd in python3 ros2; do
        if ! command -v "${cmd}" >/dev/null; then
            log_error "缺少依赖: ${cmd}"
            exit 1
        fi
    done

    # 检查 PyYAML
    if ! python3 -c "import yaml" 2>/dev/null; then
        log_error "缺少 PyYAML，请安装: sudo apt install python3-yaml"
        exit 1
    fi

    load_joint_data
    start_driver
    main_loop
}

main "$@"
