#!/usr/bin/env python3
"""
zl_robot 电机位置读取工具（只读，不发送任何控制命令）

仿照 bipedal_humanoid_pro/interactive_test.py（订阅反馈 + 交互菜单）和
zl_robot/scripts/motor.sh（解析 YAML 获取关节映射）编写。

原理: run.sh 启动的 robot_driver_node 会周期发布 /robot/driver/joint_states
(robot_interfaces/msg/JointStates)，本脚本纯订阅该话题即可在 run.sh 运行
过程中读取所有电机位置，不会重复启动 driver，也不会干扰控制链路。

用法:
  终端1: ./scripts/run.sh                       (机器人正常运行中)
  终端2: ./read_motor.py                        (自动补齐 ROS/DDS 环境)
         ./read_motor.py --once                 (打印一次全部关节后退出)
         ./read_motor.py --watch                (持续刷新监视)
         ./read_motor.py --topic /xxx          (自定义话题名)

菜单:
  [1] 读取全部关节位置   一次性打印表格
  [2] 连续监视           按分组滚动刷新, Ctrl+C 退出监视
  [3] 单关节详情         选择关节后高频刷新位置/速度/力矩/温度
  [4] 导出 CSV           将当前快照保存到 logs/ 下
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

# 与 run.sh 保持一致的默认运行形态（如需其他 profile 用命令行覆盖）
DEFAULT_ROBOT_TYPE = "humanoid"
DEFAULT_HARDWARE_PROFILE = "tienkung"
DEFAULT_ROS_DOMAIN_ID = "10"


# ── 环境准备: 独立终端运行时自动 source ROS + 本工程 install ──

def ensure_ros_env():
    """若当前 shell 未 source ROS 环境或本工程 install, 借助 bash 导入 run.sh 同款环境变量。

    注意: 不能只判断 rclpy 是否可导入。若终端只 source 了 /opt/ros/humble 而
    没 source 本工程 install/setup.bash, rclpy 可用但 robot_interfaces 缺失,
    此时仍需补 source, 否则后续导入自定义消息包会 ModuleNotFoundError。
    """
    try:
        import rclpy  # noqa: F401
        import robot_interfaces.msg  # noqa: F401
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

    # 关键: 当前解释器的 sys.path 已在启动时固定, 仅写 os.environ 无法让本进程
    # 导入 rclpy/robot_interfaces。需用补齐后的环境重新 exec 本脚本, 让新解释器
    # 启动时读取 AMENT_PYTHONPATH 建立正确搜索路径。哨兵变量防止无限重启。
    if os.environ.get("_ZL_READ_MOTOR_REEXEC") != "1":
        os.environ["_ZL_READ_MOTOR_REEXEC"] = "1"
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


# ── 关节映射解析 (仿 motor.sh: robot_joints + hw_<profile>) ──

def load_joint_map(robot_type, hardware_profile):
    """返回 {joint_name: {"index", "group", "motor_id", "vendor"}}, 解析失败返回 {}。"""
    vendor = "mix" if hardware_profile.startswith("mix_") else hardware_profile
    if robot_type == "humanoid":
        joints_path = CONFIG_ROOT / robot_type / f"robot_joints_{vendor}.yaml"
    else:
        joints_path = CONFIG_ROOT / robot_type / "robot_joints.yaml"
    hw_path = CONFIG_ROOT / robot_type / f"hw_{hardware_profile}.yaml"

    for p in (joints_path, hw_path):
        if not p.exists():
            print(f"[WARN] 配置文件不存在: {p}")
            return {}
    try:
        import yaml
    except ImportError:
        print("[WARN] 缺少 PyYAML, 不解析关节映射 (仅显示关节名)")
        return {}

    with open(joints_path) as f:
        robot_joints = yaml.safe_load(f).get("robot_joints", {}).get("joints", [])
    with open(hw_path) as f:
        hw_joints = yaml.safe_load(f).get("hardware", {}).get("joints", [])
    hw_by_name = {j.get("name"): j for j in hw_joints}

    joint_map = {}
    for idx, rj in enumerate(robot_joints, 1):
        name = rj.get("name")
        hw = hw_by_name.get(name, {})
        joint_map[name] = {
            "index": idx,
            "group": rj.get("group", ""),
            "motor_id": hw.get("motor_id", "?"),
            "vendor": hw.get("vendor", vendor),
        }
    return joint_map


# ── Node: 只订阅, 不发布 ──

class MotorReader:
    def __init__(self, topic):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        from robot_interfaces.msg import JointStates

        self.rclpy = rclpy
        self.node = Node("read_motor")
        # driver 端用 SensorDataQoS(BEST_EFFORT), 订阅必须兼容
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        self.lock = threading.Lock()
        self.snap = {"states": [], "seq": 0, "t": 0.0}
        self.node.create_subscription(JointStates, topic, self._on_state, qos)

    def _on_state(self, msg):
        with self.lock:
            self.snap = {
                "states": [(s.joint_name, s.position, s.velocity, s.effort,
                            s.motor_temp, s.mos_temp, s.motor_error,
                            s.joint_error, s.enabled, s.error_message)
                           for s in msg.states],
                "seq": msg.sequence,
                "t": time.time(),
            }

    def spin_once(self, timeout=0.1):
        self.rclpy.spin_once(self.node, timeout_sec=timeout)

    def wait_first(self, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.spin_once(0.1)
            with self.lock:
                if self.snap["states"]:
                    return True
        return False

    def get(self):
        with self.lock:
            return dict(self.snap)


# ── 格式化输出 ──

RAD2DEG = 180.0 / 3.14159265

def fmt_row(s, jmap):
    name = s[0]
    info = jmap.get(name, {})
    mid = info.get("motor_id", "?")
    group = info.get("group", "")
    idx = info.get("index", "?")
    pos_deg = s[1] * RAD2DEG
    err = ""
    if s[7] != 0 or s[6] != 0:
        err = f" ⚠err(j={s[7]},m={s[6]})"
    if not s[8]:
        err += " [未使能]"
    return (f"  #{idx:<3} M{str(mid):<4} {group:<10} {name:<28} "
            f"pos={pos_deg:+9.3f}° ({s[1]:+8.4f} rad) "
            f"vel={s[2]:+7.3f} eff={s[3]:+6.2f} T={s[4]:.0f}/{s[5]:.0f}℃{err}")


def print_table(reader, jmap, highlight=None):
    snap = reader.get()
    age = time.time() - snap["t"] if snap["t"] else -1
    print(f"\n{'=' * 78}")
    print(f" 电机位置快照  seq={snap['seq']}  采集于 {age * 1000:.0f}ms 前  "
          f"共 {len(snap['states'])} 关节")
    print('=' * 78)
    for s in snap["states"]:
        line = fmt_row(s, jmap)
        if highlight and s[0] == highlight:
            print("\033[0;36m" + line + "\033[0m")
        else:
            print(line)
    print('=' * 78)


def export_csv(reader, jmap):
    snap = reader.get()
    if not snap["states"]:
        print("  ❌ 尚无数据")
        return
    log_dir = PROJECT_ROOT / "logs"
    log_dir.mkdir(exist_ok=True)
    path = log_dir / f"motor_pos_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["index", "motor_id", "group", "joint_name",
                    "position_rad", "position_deg", "velocity", "effort",
                    "motor_temp", "mos_temp", "motor_error", "joint_error", "enabled"])
        for s in snap["states"]:
            info = jmap.get(s[0], {})
            w.writerow([info.get("index", ""), info.get("motor_id", ""),
                        info.get("group", ""), s[0],
                        f"{s[1]:.6f}", f"{s[1] * RAD2DEG:.4f}",
                        f"{s[2]:.4f}", f"{s[3]:.3f}",
                        s[4], s[5], s[6], s[7], int(s[8])])
    print(f"  ✅ 已导出: {path}")


# ── 交互动作 ──

def act_watch(reader, jmap, interval):
    print("连续监视 (Ctrl+C 退出)...")
    try:
        while True:
            snap_before = reader.get()["seq"]
            t0 = time.time()
            while time.time() - t0 < interval:
                reader.spin_once(0.05)
            print_table(reader, jmap)
            if reader.get()["seq"] == snap_before:
                print("  ⚠ 话题无更新, 请确认 run.sh 正在运行且 DDS 配置一致")
    except KeyboardInterrupt:
        print("\n  退出监视")


def act_single_detail(reader, jmap):
    names = [s[0] for s in reader.get()["states"]]
    if not names:
        print("  ❌ 尚无关节数据")
        return
    print("\n可监视关节:")
    for i, n in enumerate(names, 1):
        info = jmap.get(n, {})
        print(f"  [{i}] {n}  (M{info.get('motor_id', '?')}, {info.get('group', '')})")
    try:
        c = input("选择序号 (q 返回): ").strip()
        if c.lower() == 'q':
            return
        name = names[int(c) - 1]
    except (ValueError, IndexError, EOFError):
        print("  无效输入")
        return
    print(f"\n[{name}] 详情 (Ctrl+C 返回):")
    try:
        while True:
            reader.spin_once(0.05)
            snap = reader.get()
            for s in snap["states"]:
                if s[0] == name:
                    info = jmap.get(name, {})
                    msg = f"\r  M{info.get('motor_id', '?')} pos={s[1] * RAD2DEG:+9.3f}° ({s[1]:+8.4f} rad) vel={s[2]:+7.3f} eff={s[3]:+6.2f} T={s[4]:.0f}/{s[5]:.0f}℃"
                    if s[9]:
                        msg += f" [{s[9]}]"
                    print(msg, end="", flush=True)
    except KeyboardInterrupt:
        print()


MENU = """
┌─ read_motor (只读) ──────────────┐
│ [1] 读取全部关节位置 (一次快照)  │
│ [2] 连续监视                     │
│ [3] 单关节详情 (高频刷新)        │
│ [4] 导出当前快照为 CSV           │
│ [q] 退出                         │
└──────────────────────────────────┘"""


def main():
    ap = argparse.ArgumentParser(description="zl_robot 电机位置只读工具 (运行于 run.sh 期间)")
    ap.add_argument("--topic", default="/robot/driver/joint_states",
                    help="关节状态话题 (默认: /robot/driver/joint_states)")
    ap.add_argument("--robot-type", default=DEFAULT_ROBOT_TYPE)
    ap.add_argument("--profile", default=DEFAULT_HARDWARE_PROFILE,
                    help="硬件 profile, 用于解析电机 ID/分组映射")
    ap.add_argument("--interval", type=float, default=1.0, help="监视刷新周期秒")
    ap.add_argument("--once", action="store_true", help="打印一次快照后退出")
    ap.add_argument("--watch", action="store_true", help="进入连续监视 (非交互)")
    args = ap.parse_args()

    ensure_ros_env()
    ensure_dds_env()
    import rclpy
    rclpy.init()

    jmap = load_joint_map(args.robot_type, args.profile)
    if jmap:
        print(f"[INFO] 已加载 {len(jmap)} 个关节映射 ({args.robot_type}/{args.profile})")

    reader = MotorReader(args.topic)
    print(f"[INFO] 订阅 {args.topic} (ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']})")
    print("[INFO] 等待关节反馈 (确认 run.sh 正在运行)...", end="", flush=True)
    if not reader.wait_first(timeout=15.0):
        print("\n[ERROR] 超时未收到数据。请确认:")
        print("  1. run.sh 已启动且 robot_driver_node 正常运行 (ros2 topic list | grep joint_states)")
        print("  2. 本终端 DDS 配置与 run.sh 一致 (cyclonedds_localhost.xml, domain 10)")
        sys.exit(1)
    print(" ✅")

    try:
        if args.once:
            print_table(reader, jmap)
            return
        if args.watch:
            act_watch(reader, jmap, args.interval)
            return
        while True:
            # 菜单期间也持续 spin, 保证快照最新
            reader.spin_once(0.05)
            try:
                choice = input(MENU + "\n选择: ").strip().lower()
            except EOFError:
                break
            if choice == "q":
                break
            elif choice == "1":
                print_table(reader, jmap)
            elif choice == "2":
                act_watch(reader, jmap, args.interval)
            elif choice == "3":
                act_single_detail(reader, jmap)
            elif choice == "4":
                export_csv(reader, jmap)
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
