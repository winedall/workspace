#!/bin/bash
#
# zl_robot 运行日志管理脚本。
#
# 用法：
#   ./scripts/logs.sh list
#   ./scripts/logs.sh cleanup [retention_days] [max_size_mb] [log_root] [current_run_dir]

set -euo pipefail

# 默认日志布局：
#   logs/<RUN_ID>/zl_robot.log
#   logs/<RUN_ID>/ros/
#   logs/<RUN_ID>/recorder/
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_PATH="${SCRIPT_DIR}/.."
LOG_ROOT="${PROJECT_PATH}/logs"
CURRENT_RUN_DIR=""

usage() {
    echo "Usage: $0 list | cleanup [retention_days] [max_size_mb] [log_root] [current_run_dir]"
}

ensure_log_root() {
    mkdir -p "$LOG_ROOT"
}

run_dirs() {
    # 只管理一级运行目录，避免误删 ros/recorder 这类运行目录内部文件夹。
    find "$LOG_ROOT" -mindepth 1 -maxdepth 1 -type d | sort
}

dir_size_bytes() {
    du -sb "$1" | awk '{print $1}'
}

is_protected_dir() {
    local dir="$1"
    # 当前运行目录可能仍有节点写日志，清理时必须跳过。
    [ -n "$CURRENT_RUN_DIR" ] && [ "$(readlink -f "$dir")" = "$(readlink -f "$CURRENT_RUN_DIR")" ]
}

list_logs() {
    ensure_log_root
    local found=0
    while IFS= read -r dir; do
        found=1
        local size
        size="$(du -sh "$dir" | awk '{print $1}')"
        printf "%s\t%s\n" "$size" "$dir"
    done < <(run_dirs)
    if [ "$found" -eq 0 ]; then
        echo "No log runs under $LOG_ROOT"
    fi
}

cleanup_by_age() {
    local retention_days="$1"
    local deleted=0

    # retention_days < 0 表示关闭按时间清理。
    if [ "$retention_days" -lt 0 ]; then
        return 0
    fi

    while IFS= read -r dir; do
        if is_protected_dir "$dir"; then
            continue
        fi
        # retention_days = 0 表示清理所有非当前运行目录；
        # 大于 0 时清理修改时间超过指定天数的运行目录。
        if [ "$retention_days" -eq 0 ] || [ "$(find "$dir" -maxdepth 0 -mtime +"$retention_days" -print)" ]; then
            rm -rf -- "$dir"
            deleted=$((deleted + 1))
        fi
    done < <(run_dirs)

    echo "$deleted"
}

cleanup_by_size() {
    local max_size_mb="$1"
    local deleted=0

    # max_size_mb <= 0 表示关闭按容量清理。
    if [ "$max_size_mb" -le 0 ]; then
        return 0
    fi

    local max_bytes=$((max_size_mb * 1024 * 1024))
    local total
    total="$(dir_size_bytes "$LOG_ROOT")"

    while [ "$total" -gt "$max_bytes" ]; do
        local oldest=""
        # run_dirs 已按目录名排序；当前 RUN_ID 使用时间前缀，因此最前面的是最旧运行目录。
        while IFS= read -r dir; do
            if is_protected_dir "$dir"; then
                continue
            fi
            oldest="$dir"
            break
        done < <(run_dirs)

        if [ -z "$oldest" ]; then
            break
        fi

        rm -rf -- "$oldest"
        deleted=$((deleted + 1))
        total="$(dir_size_bytes "$LOG_ROOT")"
    done

    echo "$deleted"
}

cleanup_logs() {
    # 默认策略：保留 30 天，且日志根目录不超过 1024MB。
    local retention_days="${1:-30}"
    local max_size_mb="${2:-1024}"
    LOG_ROOT="${3:-$LOG_ROOT}"
    CURRENT_RUN_DIR="${4:-}"
    ensure_log_root

    local age_deleted
    local size_deleted
    age_deleted="$(cleanup_by_age "$retention_days")"
    size_deleted="$(cleanup_by_size "$max_size_mb")"
    echo "Deleted $((age_deleted + size_deleted)) log run directories from $LOG_ROOT"
}

cmd="${1:-list}"
shift || true

case "$cmd" in
    list)
        list_logs
        ;;
    cleanup)
        cleanup_logs "$@"
        ;;
    *)
        usage
        exit 2
        ;;
esac
