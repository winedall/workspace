#!/bin/bash
# CAN 诊断工具。
#
# 用法：
#   ./scripts/can.sh monitor [-i "can0 can1"] [-n 1] [-w 80]
#   ./scripts/can.sh diag [-i "can0 can1 can2 can3"] [-n 0.2] [-o /tmp/can_diag]
#
# monitor: 实时显示 byte_queue_limits inflight/limit，兼容旧用法。
# diag:    长时间采集 CAN 链路状态、内核日志、error frame、SocketCAN 统计。

set -u

CAN_IFACES="can0 can1 can2 can3"
INTERVAL=1
WARN_PCT=80
OUT_DIR=""
PIDS=""

usage() {
  cat <<EOF
用法:
  $0 monitor [-i "can0 can1"] [-n 1] [-w 80]
  $0 diag    [-i "can0 can1 can2 can3"] [-n 0.2] [-o /tmp/can_diag]

兼容旧用法:
  $0 [-i "can0 can1"] [-n 1] [-w 80]
EOF
}

# 清屏函数
clear_screen() {
  printf '\033[2J\033[H'
}

monitor_loop() {
  while true; do
    clear_screen
    echo "CAN TX Queue Monitor  |  $(date '+%F %T')  |  interval=${INTERVAL}s  warn=${WARN_PCT}%"
    echo "--------------------------------------------------------------"
    printf "%-8s %12s %12s %8s   %s\n" "IFACE" "INFLIGHT" "LIMIT" "USAGE" "STATUS"
    echo "--------------------------------------------------------------"

    for iface in $CAN_IFACES; do
      base="/sys/class/net/${iface}/queues/tx-0/byte_queue_limits"
      inflight=$(cat "$base/inflight" 2>/dev/null || true)
      limit=$(cat "$base/limit" 2>/dev/null || true)

      if [ -z "$inflight" ] || [ -z "$limit" ]; then
        printf "%-8s %12s %12s %8s   %s\n" "$iface" "N/A" "N/A" "-" "路径不可用"
        continue
      fi

      if [ "$limit" -le 0 ]; then
        printf "%-8s %12s %12s %8s   %s\n" "$iface" "$inflight" "$limit" "-" "limit=0(不可靠)"
        continue
      fi

      pct=$(( inflight * 100 / limit ))
      if [ "$inflight" -ge "$limit" ]; then
        status="[已满!]"
      elif [ "$pct" -ge "$WARN_PCT" ]; then
        status="[${pct}% 接近满]"
      else
        status="[${pct}%]"
      fi

      printf "%-8s %12s %12s %7s%%   %s\n" "$iface" "$inflight" "$limit" "$pct" "$status"
    done

    echo "--------------------------------------------------------------"
    echo "按 Ctrl+C 退出"
    sleep "$INTERVAL"
  done
}

add_pid() {
  PIDS="$PIDS $1"
}

stop_diag() {
  trap - INT TERM EXIT
  echo
  echo "停止 CAN 诊断采集..."
  for pid in $PIDS; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true

  if [ -n "$OUT_DIR" ] && [ -d "$OUT_DIR" ]; then
    {
      echo "ended_at=$(date '+%F %T.%3N')"
      echo "interfaces=$CAN_IFACES"
      echo "interval=$INTERVAL"
    } >> "$OUT_DIR/summary.txt"
    tar -czf "${OUT_DIR}.tar.gz" -C "$(dirname "$OUT_DIR")" "$(basename "$OUT_DIR")" 2>/dev/null || true
    echo "诊断目录: $OUT_DIR"
    [ -f "${OUT_DIR}.tar.gz" ] && echo "压缩包: ${OUT_DIR}.tar.gz"
  fi
}

stop_diag_and_exit() {
  stop_diag
  exit 0
}

collect_link_stats() {
  while true; do
    {
      echo "===== $(date '+%F %T.%3N') ====="
      for iface in $CAN_IFACES; do
        echo "----- $iface -----"
        ip -details -statistics link show "$iface" 2>&1
        echo "tx_queue_len=$(cat "/sys/class/net/${iface}/tx_queue_len" 2>/dev/null || echo N/A)"
        base="/sys/class/net/${iface}/queues/tx-0/byte_queue_limits"
        if [ -d "$base" ]; then
          echo "bql_inflight=$(cat "$base/inflight" 2>/dev/null || echo N/A)"
          echo "bql_limit=$(cat "$base/limit" 2>/dev/null || echo N/A)"
          echo "bql_limit_max=$(cat "$base/limit_max" 2>/dev/null || echo N/A)"
        else
          echo "bql=unavailable"
        fi
      done
    } >> "$OUT_DIR/link_stats.log"
    sleep "$INTERVAL"
  done
}

collect_proc_stats() {
  while true; do
    {
      echo "===== $(date '+%F %T.%3N') ====="
      echo "--- /proc/net/can/stats ---"
      cat /proc/net/can/stats 2>&1 || true
      echo "--- /proc/net/can/rcvlist_all ---"
      cat /proc/net/can/rcvlist_all 2>&1 || true
      echo "--- /proc/net/sockstat ---"
      cat /proc/net/sockstat 2>&1 || true
      echo "--- robot_driver pids ---"
      pgrep -af "robot_driver|robot_driver_node" 2>&1 || true
    } >> "$OUT_DIR/socket_stats.log"
    sleep 0.5
  done
}

collect_dmesg() {
  if command -v dmesg >/dev/null 2>&1; then
    dmesg -wT 2>&1 | grep -i --line-buffered 'can\|mttcan\|socketcan\|bus-off\|error passive\|error warning\|tx\|skb\|buffer' >> "$OUT_DIR/dmesg_can.log" &
    add_pid "$!"
  else
    echo "dmesg not found" >> "$OUT_DIR/dmesg_can.log"
  fi
}

collect_candump() {
  if command -v candump >/dev/null 2>&1; then
    candump -ta -e $CAN_IFACES >> "$OUT_DIR/candump_error.log" 2>&1 &
    add_pid "$!"
  else
    echo "candump not found; install can-utils to capture CAN error frames" >> "$OUT_DIR/candump_error.log"
  fi
}

run_diag() {
  if [ -z "$OUT_DIR" ]; then
    OUT_DIR="/tmp/can_diag_$(date '+%Y%m%d_%H%M%S')"
  fi
  mkdir -p "$OUT_DIR"

  {
    echo "started_at=$(date '+%F %T.%3N')"
    echo "interfaces=$CAN_IFACES"
    echo "interval=$INTERVAL"
    echo "host=$(hostname 2>/dev/null || true)"
    echo "kernel=$(uname -a)"
  } > "$OUT_DIR/summary.txt"

  trap stop_diag_and_exit INT TERM
  trap stop_diag EXIT

  collect_link_stats &
  add_pid "$!"
  collect_proc_stats &
  add_pid "$!"
  collect_dmesg
  collect_candump

  echo "CAN 诊断采集中，按 Ctrl+C 结束并打包。"
  echo "诊断目录: $OUT_DIR"
  echo "监控接口: $CAN_IFACES"
  echo "采样间隔: ${INTERVAL}s"

  while true; do
    sleep 3600 &
    wait "$!" || true
  done
}

MODE="monitor"
case "${1:-}" in
  monitor|diag)
    MODE="$1"
    shift
    ;;
  -h|--help|help)
    usage
    exit 0
    ;;
esac

while getopts "i:n:w:o:h" opt; do
  case $opt in
    i) CAN_IFACES="$OPTARG" ;;
    n) INTERVAL="$OPTARG" ;;
    w) WARN_PCT="$OPTARG" ;;
    o) OUT_DIR="$OPTARG" ;;
    h) usage; exit 0 ;;
    *) usage; exit 1 ;;
  esac
done

case "$MODE" in
  monitor)
    monitor_loop
    ;;
  diag)
    run_diag
    ;;
  *)
    usage
    exit 1
    ;;
esac
