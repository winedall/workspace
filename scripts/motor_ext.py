#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""motor_ext.py - 借用运行中的完整链路做单电机位置覆盖调试（真机）。

设计（与 motor_fp.py 的 standalone-driver 路线不同）：
  本工具【不启动 driver】，假设 ./run.sh 的完整系统（router→control→driver→电机）
  已在运行，通过官方外部注入通道对指定关节做位置伺服覆盖：
    1. 让 router 进入 MIX 模式（intent: config/mode/mix）；
    2. 向 /robot/external/desired_commands 发布单关节 JointCommands；
    3. router 在 MIX 下把外部指令 merge 进每一帧 RL desired_commands
       （覆盖 position/kp/kd，enable 取 AND），下游 control 过滤后按 200Hz 落 driver。
  这样电机走的是正常使能 + 持续 RUN/MIT 流，绕开 standalone driver 使能不了的死结。

前提（缺一不可，否则命令不会透传）：
  - 完整系统已启动：./scripts/run.sh（或整机 bringup）；
  - 机器人处于 RL 会持续吐 desired_commands 的状态（READY/STANDBY/RUN 等，
    即 RL 已把该关节 enable=true 命令着）；MIX 的 merge 只覆盖 RL 帧里已有的关节，
    且 enable = RL ∧ 外部，所以本工具【不能】使能 RL 没开的关节；
  - router 处于 MIX 模式（用遥控器 mode 菜单切 mix，或本工具的 mix 命令）。

⚠ 安全：这是在【活的整体系统】上覆盖单个关节目标位置。机器人必须吊起/离地/卸载，
  否则会破坏 RL 平衡；x/退出 会把关节交还 RL，它会拉回自己的位姿（可能突然回位）。
  本通道只能覆盖 position/kp/kd，effort/τ_ff、velocity 不透传，故无纯力矩模式。

用法：
  ./scripts/motor_ext.sh                       # tienkung humanoid，自动对齐 DDS 环境
  python3 motor_ext.py --joints-config config/humanoid/robot_joints_tienkung.yaml
"""

import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from robot_interfaces.msg import (
    JointCommand,
    JointCommands,
    JointStates,
    RobotIntent,
)

GAIN_MAX = 1000.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def load_joints(path):
    """解析 robot_joints_*.yaml，返回带编号与限位的关节表（编号与 motor.sh 一致）。"""
    import yaml

    with open(path, "r") as f:
        data = yaml.safe_load(f)
    joints = data.get("robot_joints", {}).get("joints", [])
    table = []
    for idx, j in enumerate(joints, 1):
        table.append({
            "index": idx,
            "name": j["name"],
            "group": j.get("group", ""),
            "min_pos": float(j.get("min_pos", -math.pi)),
            "max_pos": float(j.get("max_pos", math.pi)),
            "max_vel": float(j.get("max_vel", 1.0)),
        })
    return table


class ExternalOverrideNode(Node):
    """持续向 external/desired_commands 发布当前单关节覆盖；缓存由 router 每帧合并。"""

    def __init__(self, ns, rate_hz, intent_source):
        super().__init__("motor_ext", namespace=ns)
        self.dt = 1.0 / rate_hz
        self.intent_source = intent_source

        self.lock = threading.Lock()
        self.feedback = {}          # name -> {position, velocity, effort, stamp}
        self.override = None        # {joint, goal, cmd_pos, kp, kd}
        self.seq = 0

        best = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.ext_pub = self.create_publisher(JointCommands, "external/desired_commands", best)
        self.intent_pub = self.create_publisher(
            RobotIntent, "input/intent",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.states_sub = self.create_subscription(
            JointStates, "control/hardware_states", self.on_states, best)

        self.create_timer(self.dt, self.publish_tick)
        self.get_logger().info(
            f"外部覆盖就绪 ns=/{ns.lstrip('/')}/external/desired_commands rate={1.0 / self.dt:.0f}Hz")

    def on_states(self, msg):
        now = time.monotonic()
        with self.lock:
            for s in msg.states:
                self.feedback[s.joint_name] = {
                    "position": float(s.position),
                    "velocity": float(s.velocity),
                    "effort": float(getattr(s, "effort", 0.0)),
                    "stamp": now,
                }

    def current_position(self, name, max_age=0.5):
        with self.lock:
            fb = self.feedback.get(name)
            if fb is None or (time.monotonic() - fb["stamp"]) > max_age:
                return None
            return fb["position"]

    def snapshot(self, name):
        with self.lock:
            fb = self.feedback.get(name)
            return dict(fb) if fb else None

    def publish_tick(self):
        """有覆盖就发该关节，没有就发空帧（清空 router 缓存，关节交还 RL）。"""
        msg = JointCommands()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "external"
        msg.stage = "desired"
        self.seq += 1
        msg.sequence = self.seq
        ov = None
        with self.lock:
            if self.override is not None:
                ov = dict(self.override)
        if ov is not None:
            j = ov["joint"]
            step = max(ov["speed"] * self.dt, 1e-6)
            err = ov["goal"] - ov["cmd_pos"]
            ov["cmd_pos"] += clamp(err, -step, step)
            with self.lock:
                self.override["cmd_pos"] = ov["cmd_pos"]
            c = JointCommand()
            c.joint_name = j["name"]
            c.position = ov["cmd_pos"]
            c.kp = ov["kp"]
            c.kd = ov["kd"]
            c.enable = True
            msg.commands.append(c)
        self.ext_pub.publish(msg)

    def start_override(self, joint, pos, kp, kd, speed):
        with self.lock:
            self.override = {
                "joint": joint, "goal": pos, "cmd_pos": pos,
                "kp": kp, "kd": kd, "speed": speed,
            }

    def release(self):
        with self.lock:
            self.override = None

    def update(self, **kw):
        with self.lock:
            if self.override is not None:
                self.override.update(kw)

    def send_mode(self, action):
        """发送 config/mode/<action> 意图（rl/mix/app/spec）；能否过准入取决于系统状态。"""
        msg = RobotIntent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = self.intent_source
        msg.intent_type = "config"
        msg.command = "mode"
        msg.action = action
        msg.priority = 0
        self.intent_pub.publish(msg)


def print_help():
    print("""命令:
  【一键】go <相对位移rad> [kp] [kd] [速度]  例: go 0.3 60 2 0.3
           选中即从当前位置起步覆盖，一条命令相对移动并立即出力
  <编号>       选择关节（从当前实测位置起步覆盖；自动交还上一个关节）
  mix / rl     发送切 MIX / RL 模式意图（也可用遥控器 mode 菜单切 mix，更稳）
  g <rad>      绝对位置目标     r <rad>  相对位置目标
  s <rad/s>    覆盖爬升速度(默认0.3)   p <kp> <kd>  增益(上限1000)
  x            释放：清空覆盖，把关节交还 RL（注意 RL 会拉回自己位姿）
  c            看当前关节反馈(pos/vel/effort)   l  关节列表
  q / Ctrl+C   退出（先清空覆盖）
提示: 关节不动 → 确认 ①run.sh 在跑 ②机器人处于 RL 吐命令状态 ③已切 MIX ④有反馈(见 c)。""")


def print_table(joints, active):
    print("=" * 70)
    print(f"{'编号':<5}{'关节名':<24}{'分组':<12}{'限位[rad]'}")
    for j in joints:
        mark = "  <== 覆盖中" if active and j["name"] == active["name"] else ""
        print(f"{j['index']:<6}{j['name']:<25}{j['group']:<13}"
              f"[{j['min_pos']:+.2f}, {j['max_pos']:+.2f}]{mark}")
    print("=" * 70)


def main(args=None):
    ap = argparse.ArgumentParser(description="借用运行时链路的外部单电机位置覆盖工具")
    ap.add_argument("--joints-config", required=True, help="robot_joints_*.yaml 路径")
    ap.add_argument("--ns", default="/robot", help="命名空间（默认 /robot，与整机 bringup 一致）")
    ap.add_argument("--rate", type=float, default=50.0, help="外部覆盖发布频率 Hz（默认50，router 每帧合并）")
    ap.add_argument("--intent-source", default="app", help="模式切换意图来源（默认 app）")
    ap.add_argument("--yes", action="store_true", help="跳过危险确认（脚本化测试用）")
    a = ap.parse_args()

    joints = load_joints(a.joints_config)
    if not joints:
        print("未解析到关节，检查 --joints-config")
        sys.exit(1)
    by_idx = {j["index"]: j for j in joints}

    if not a.yes:
        print("\n" + "!" * 70)
        print("! 危险：本工具在【正在运行的完整系统】上覆盖单关节目标位置。")
        print("! 必须先 ./scripts/run.sh 起整机、把机器人吊起/离地卸载，")
        print("! 并处于 RL 持续吐命令的站立/待命态、router 已切 MIX。")
        print("! x/退出会把关节交还 RL，它会拉回自身位姿（可能突然回位）。")
        print("!" * 70)
        if input("输入 yes 继续，其他退出: ").strip().lower() != "yes":
            print("已取消")
            sys.exit(0)

    rclpy.init()
    node = ExternalOverrideNode(a.ns, a.rate, a.intent_source)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    cur = None
    kp, kd, speed = 60.0, 2.0, 0.3
    print_help()
    try:
        while rclpy.ok():
            sel = cur["name"] if cur else "未选"
            snap = node.snapshot(sel) if cur else None
            if snap and (time.monotonic() - snap["stamp"]) <= 0.5:
                fb = f"pos={snap['position']:+.4f} vel={snap['velocity']:+.3f} effort={snap['effort']:+.2f}"
            else:
                fb = "无反馈(检查 run.sh/MIX/状态/环境)"
            ov = "覆盖中" if (cur and _is_override(node, sel)) else "空闲"
            print(f"[{sel}] {ov}  kp={kp:.0f} kd={kd:.1f} v={speed:.2f}  {fb}")
            try:
                line = input("(编号/go/mix/rl/g/r/s/p/x/c/l/q) > ").strip()
            except EOFError:
                break
            if not line:
                continue
            parts = line.split()
            cmd = parts[0].lower()
            try:
                if cmd == "q":
                    break
                elif cmd == "l":
                    print_table(joints, cur)
                elif cmd == "help":
                    print_help()
                elif cmd in ("mix", "rl"):
                    node.send_mode(cmd)
                    print(f"已发送 mode/{cmd} 意图（source={a.intent_source}）；若不动看 router 是否 Rejected。")
                elif cmd == "c":
                    if not cur:
                        print("未选关节。")
                    elif snap:
                        print(f"  pos={snap['position']:+.4f} vel={snap['velocity']:+.4f} effort={snap['effort']:+.3f}")
                    else:
                        print("  无反馈数据。")
                elif cmd == "x":
                    node.release()
                    print("已清空覆盖，关节交还 RL（RL 可能拉回自身位姿）。")
                elif cmd == "p":
                    kp = clamp(float(parts[1]), 0, GAIN_MAX)
                    kd = clamp(float(parts[2]), 0, GAIN_MAX) if len(parts) > 2 else kd
                    node.update(kp=kp, kd=kd)
                    print(f"增益 kp={kp:.0f} kd={kd:.1f}")
                elif cmd == "s":
                    speed = clamp(float(parts[1]), 0.01, 5.0)
                    node.update(speed=speed)
                    print(f"爬升速度 {speed:.2f} rad/s")
                elif cmd == "go":
                    if len(parts) < 2:
                        print("用法: go <相对位移rad> [kp] [kd] [速度]")
                        continue
                    if not cur:
                        print("先选关节编号（或 g/r 前也要先选），再 go。")
                        continue
                    rel = float(parts[1])
                    kp = clamp(float(parts[2]), 0, GAIN_MAX) if len(parts) > 2 else kp
                    kd = clamp(float(parts[3]), 0, GAIN_MAX) if len(parts) > 3 else kd
                    speed = clamp(float(parts[4]), 0.01, cur["max_vel"]) if len(parts) > 4 else speed
                    j = cur
                    base = node.current_position(j["name"])
                    if base is None:
                        print("无反馈位置，起步不安全，先确认系统/反馈在流。")
                        continue
                    node.start_override(j, base, kp, kd, speed)
                    new_goal = clamp(base + rel, j["min_pos"], j["max_pos"])
                    node.update(goal=new_goal)
                    print(f"[go] {j['name']} {base:+.4f}->{new_goal:+.4f} rad "
                          f"kp={kp:.0f} kd={kd:.1f} v={speed:.2f}")
                elif cmd in ("g", "r"):
                    if not cur:
                        print("先选关节编号（或直接用 go）。")
                        continue
                    if len(parts) < 2:
                        print("需要参数，例: g 0.5 / r -0.2")
                        continue
                    base = node.current_position(cur["name"])
                    if base is None:
                        print("无反馈位置，起步不安全，先确认系统/反馈在流。")
                        continue
                    val = float(parts[1])
                    new_goal = clamp(val if cmd == "g" else base + val, cur["min_pos"], cur["max_pos"])
                    if not _is_override(node, cur["name"]):
                        node.start_override(cur, base, kp, kd, speed)
                    node.update(goal=new_goal)
                    print(f"{cur['name']} 目标 -> {new_goal:+.4f} rad")
                elif cmd.isdigit():
                    j = by_idx.get(int(cmd))
                    if not j:
                        print("无此编号，用 l 查表。")
                        continue
                    pos = j
                    base = node.current_position(j["name"])
                    if base is None:
                        print(f"选中 {j['name']}，但暂无反馈；先确认 run.sh 在跑且已切 MIX。")
                        cur = j
                        continue
                    cur = j
                    node.start_override(j, base, kp, kd, speed)
                    print(f"已选中并覆盖 {j['name']}（从 {base:+.4f} 起步）。可直接 go/g/r/p/s。")
                else:
                    print("未知命令，help 看帮助。")
            except (ValueError, IndexError) as e:
                print(f"参数错误: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        node.release()
        time.sleep(0.1)
        node.destroy_node()
        rclpy.shutdown()
        print("已清空覆盖并退出。")


def _is_override(node, name):
    with node.lock:
        return node.override is not None and node.override["joint"]["name"] == name


if __name__ == "__main__":
    main()
