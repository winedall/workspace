#!/usr/bin/env python3
"""
zl_robot IMU 数据读取工具（只读，不发送任何控制命令）

与 read_motor.py 同款思路：run.sh 启动的 IMU 驱动节点 (yesense_publisher) 会
周期发布 /robot/imu/nav_all (yesense_interface/msg/NavAll, ~200Hz)，本脚本纯订阅
该话题即可在 run.sh 运行过程中读取 IMU 数据，不会重复启动驱动，也不干扰控制链路。

用法:
  终端1: ./scripts/run.sh                       (机器人正常运行中)
  终端2: ./read_imu.py                          (自动补齐 ROS/DDS 环境)
         ./read_imu.py --once                   (打印一次快照后退出)
         ./read_imu.py --watch                  (持续刷新监视)
         ./read_imu.py --topic /xxx             (自定义话题名)

菜单:
  [1] 读取一次 IMU 快照
  [2] 连续监视 (姿态/角速度/加速度)
  [3] 完整字段 (含 GNSS/速度/气压/UTC)
  [4] 导出 CSV
"""

import argparse
import csv
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR
CONFIG_ROOT = PROJECT_ROOT / "config"

DEFAULT_ROS_DOMAIN_ID = "10"
DEFAULT_IMU_TOPIC = "/robot/imu/nav_all"


# ── 环境准备: 独立终端运行时自动 source ROS + 本工程 install ──

def ensure_ros_env():
    """若当前 shell 未 source ROS 环境或本工程 install, 借助 bash 导入同款环境变量。

    不能只判断 rclpy: 若终端只 source 了 /opt/ros/humble 而没 source 本工程
    install/setup.bash, rclpy 可用但 yesense_interface 缺失, 此时仍需补 source。
    """
    try:
        import rclpy  # noqa: F401
        import yesense_interface.msg  # noqa: F401
        return
    except ImportError:
        pass

    setup_cmds = (
        f"source /opt/ros/humble/setup.bash; "
        f"source {PROJECT_ROOT}/install/setup.bash; env"
    )
    ret = subprocess.run(["bash", "-c", setup_cmds], capture_output=True, text=True)
    if ret.returncode != 0:
        print("[ERROR] source ROS 环境失败, 请先手动执行:\n"
              "  source /opt/ros/humble/setup.bash && source install/setup.bash")
        print(ret.stderr)
        sys.exit(1)
    for line in ret.stdout.splitlines():
        if "=" in line:
            k, _, v = line.partition("=")
            if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", k):
                os.environ[k] = v

    # 当前解释器 sys.path 已固定, 仅写 os.environ 无法导入消息包;
    # 用补齐后的环境重新 exec 本脚本, 哨兵变量防止无限重启。
    if os.environ.get("_ZL_READ_IMU_REEXEC") != "1":
        os.environ["_ZL_READ_IMU_REEXEC"] = "1"
        os.execve(sys.executable,
                  [sys.executable, os.path.abspath(__file__)] + sys.argv[1:],
                  os.environ)


def ensure_dds_env():
    """补齐 run.sh 中的 DDS 关键变量, 保证能与 run.sh 进程域内通信。"""
    os.environ.setdefault("ROS_DOMAIN_ID", DEFAULT_ROS_DOMAIN_ID)
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    os.environ.setdefault(
        "CYCLONEDDS_URI",
        f"file://{CONFIG_ROOT}/cyclonedds_localhost.xml")
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "0")


# ── Node: 只订阅, 不发布 ──

class ImuReader:
    def __init__(self, topic):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        from yesense_interface.msg import NavAll

        self.rclpy = rclpy
        self.node = Node("read_imu")
        # 驱动端 create_publisher(..., 10) 默认 RELIABLE, 订阅须兼容用 RELIABLE
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=10)
        self.lock = threading.Lock()
        self.snap = {"msg": None, "t": 0.0, "count": 0}
        self.node.create_subscription(NavAll, topic, self._on_msg, qos)

    def _on_msg(self, msg):
        with self.lock:
            # 缓存原始消息引用即可 (NavAll 为 POD, 回调内不会被改写)
            self.snap = {"msg": msg, "t": time.time(), "count": self.snap["count"] + 1}

    def spin_once(self, timeout=0.1):
        self.rclpy.spin_once(self.node, timeout_sec=timeout)

    def wait_first(self, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.spin_once(0.1)
            with self.lock:
                if self.snap["msg"] is not None:
                    return True
        return False

    def get(self):
        with self.lock:
            return dict(self.snap)


# ── 格式化输出 ──
# 注: 单位以厂家 NavAll 原始定义为准 (欧拉角 deg, 角速度 deg/s, 加速度 m/s^2)。
#     本工具只原样展示, 不做坐标/单位换算。

def _f(v):
    return f"{v:+9.4f}"


def fmt_core(msg):
    return (
        f"  姿态  roll={_f(msg.euler.roll)}° pitch={_f(msg.euler.pitch)}° "
        f"yaw={_f(msg.euler.yaw)}°\n"
        f"  角速度 gyro x={_f(msg.gyro.x)} y={_f(msg.gyro.y)} z={_f(msg.gyro.z)}  deg/s\n"
        f"  加速度 acc  x={_f(msg.acc.x)} y={_f(msg.acc.y)} z={_f(msg.acc.z)}  m/s^2\n"
        f"  四元数 q=({msg.quat.q0:+.4f}, {msg.quat.q1:+.4f}, "
        f"{msg.quat.q2:+.4f}, {msg.quat.q3:+.4f})\n"
        f"  温度 {msg.temp.temp:.1f}℃  fusion_status={msg.status.fusion_status} "
        f"gnss_status={msg.status.gnss_status}  tid={msg.tid.tid}"
    )


def fmt_full(msg):
    lines = [fmt_core(msg)]
    lines.append(
        f"  GNSS  lon={msg.pos.longitude:.7f} lat={msg.pos.latitude:.7f} "
        f"alt={msg.pos.altitude:.3f}"
    )
    lines.append(
        f"  速度  vel_e={_f(msg.vel.vel_e)} vel_n={_f(msg.vel.vel_n)} "
        f"vel_u={_f(msg.vel.vel_u)}"
    )
    lines.append(f"  气压  {msg.pressure.val:.3f}")
    u = msg.utc
    lines.append(
        f"  UTC   {u.year:04d}-{u.month:02d}-{u.day:02d} "
        f"{u.hour:02d}:{u.min:02d}:{u.sec:02d}.{u.ms:03d}"
    )
    return "\n".join(lines)


def print_snap(reader, full=False):
    snap = reader.get()
    msg = snap["msg"]
    age = time.time() - snap["t"] if snap["t"] else -1
    title = "IMU 完整快照" if full else "IMU 快照"
    print(f"\n{'=' * 78}")
    print(f" {title}  帧计数={snap['count']}  采集于 {age * 1000:.0f}ms 前  "
          f"(单位以厂家定义为准)")
    print('=' * 78)
    if msg is None:
        print("  (尚无数据)")
    else:
        print(fmt_full(msg) if full else fmt_core(msg))
    print('=' * 78)


def export_csv(reader):
    snap = reader.get()
    msg = snap["msg"]
    if msg is None:
        print("  ❌ 尚无数据")
        return
    log_dir = PROJECT_ROOT / "logs"
    log_dir.mkdir(exist_ok=True)
    path = log_dir / f"imu_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["acc_x", "acc_y", "acc_z",
                    "gyro_x", "gyro_y", "gyro_z",
                    "euler_roll", "euler_pitch", "euler_yaw",
                    "quat_q0", "quat_q1", "quat_q2", "quat_q3",
                    "temp", "fusion_status", "gnss_status", "tid"])
        w.writerow([msg.acc.x, msg.acc.y, msg.acc.z,
                    msg.gyro.x, msg.gyro.y, msg.gyro.z,
                    msg.euler.roll, msg.euler.pitch, msg.euler.yaw,
                    msg.quat.q0, msg.quat.q1, msg.quat.q2, msg.quat.q3,
                    msg.temp.temp, msg.status.fusion_status,
                    msg.status.gnss_status, msg.tid.tid])
    print(f"  ✅ 已导出: {path}")


# ── 交互动作 ──

def act_watch(reader, interval):
    print("连续监视 (Ctrl+C 退出)...")
    try:
        while True:
            cnt_before = reader.get()["count"]
            t0 = time.time()
            while time.time() - t0 < interval:
                reader.spin_once(0.05)
            print_snap(reader)
            if reader.get()["count"] == cnt_before:
                print("  ⚠ 话题无更新, 请确认 run.sh 正在运行且 DDS 配置一致")
    except KeyboardInterrupt:
        print("\n  退出监视")


MENU = """
┌─ read_imu (只读) ────────────────┐
│ [1] 读取一次 IMU 快照            │
│ [2] 连续监视 (姿态/角速度)       │
│ [3] 完整字段 (GNSS/速度/气压)    │
│ [4] 导出当前快照为 CSV           │
│ [q] 退出                         │
└──────────────────────────────────┘"""


def main():
    ap = argparse.ArgumentParser(description="zl_robot IMU 数据只读工具 (运行于 run.sh 期间)")
    ap.add_argument("--topic", default=DEFAULT_IMU_TOPIC,
                    help=f"IMU 话题 (默认: {DEFAULT_IMU_TOPIC})")
    ap.add_argument("--interval", type=float, default=0.5, help="监视刷新周期秒")
    ap.add_argument("--once", action="store_true", help="打印一次快照后退出")
    ap.add_argument("--watch", action="store_true", help="进入连续监视 (非交互)")
    ap.add_argument("--full", action="store_true", help="显示完整字段 (含 GNSS/速度/气压)")
    args = ap.parse_args()

    ensure_ros_env()
    ensure_dds_env()
    import rclpy
    rclpy.init()

    reader = ImuReader(args.topic)
    print(f"[INFO] 订阅 {args.topic} (ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']})")
    print("[INFO] 等待 IMU 数据 (确认 run.sh 及 IMU 驱动正在运行)...", end="", flush=True)
    if not reader.wait_first(timeout=15.0):
        print("\n[ERROR] 超时未收到数据。请确认:")
        print("  1. run.sh 已启动且 yesense_publisher 正常运行 (ros2 topic list | grep nav_all)")
        print("  2. 本终端 DDS 配置与 run.sh 一致 (cyclonedds_localhost.xml, domain 10)")
        print("  3. 仿真模式下由仿真节点提供 nav_all, 需确认仿真已启动")
        sys.exit(1)
    print(" ✅")

    try:
        if args.once:
            print_snap(reader, full=args.full)
            return
        if args.watch:
            act_watch(reader, args.interval)
            return
        while True:
            reader.spin_once(0.05)
            try:
                choice = input(MENU + "\n选择: ").strip().lower()
            except EOFError:
                break
            if choice == "q":
                break
            elif choice == "1":
                print_snap(reader, full=False)
            elif choice == "2":
                act_watch(reader, args.interval)
            elif choice == "3":
                print_snap(reader, full=True)
            elif choice == "4":
                export_csv(reader)
    except KeyboardInterrupt:
        pass
    finally:
        reader.node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
