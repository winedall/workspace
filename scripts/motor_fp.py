#!/usr/bin/env python3
"""motor_fp.py - 单电机力位混合控制交互调试工具。

针对 Tienkung 0x11 力位混合控制帧（固件控制律 τ = kp·(p_cmd-p) + kd·(v_cmd-v) + τ_ff），
向 robot_driver 的 `/robot/control/hardware_commands` 下发 JointCommands 全量帧，
对单个电机做 位置伺服 / 力位混合 / 纯力矩 三种模式的控制调试。

与 motor_motion.py（只做位置限速运动）互补：本工具额外支持前馈力矩 τ_ff，
用于台架上验证"位置保持 + 叠加力矩"或"纯力矩出力"等力位混合行为。

控制模式（切换命令 m）：
  pos     位置伺服：kp/kd 闭环跟位置，τ_ff=0
  hybrid  力位混合：kp/kd 闭环跟位置，同时叠加 τ_ff（前馈力矩）
  tor     纯力矩  ：kp=0、kd=阻尼，位置命令冻结，仅输出 τ_ff

安全设计：
  - 同一时刻只使能一个关节，其余每帧显式 enable=false（驱动清槽断使能）；
  - 位置目标按限速爬升，前馈力矩按"力矩变化率"爬升，避免阶跃被 kp 放大成冲击；
  - τ_ff 限幅到 ±(max_effort × effort_scale)，默认 0.3，防止误设满量程；
  - 无反馈位置时拒绝使能，避免从 0 跳变；
  - 模式切换瞬间冻结命令，切换前后位置/力矩命令保持一致，不产生突变；
  - 退出（q / Ctrl+C / EOF）先发全关帧再结束，driver 缓存不残留运动命令。

仅限吊机/台架调试！站立的机器人会因其余关节断使能而倾倒。

通常由 scripts/motor_fp.sh 包装启动（自动拉起运控模式 driver）。
也可单独运行：
  python3 motor_fp.py --joints-config config/humanoid/robot_joints_tienkung.yaml \
      --hw-config config/humanoid/hw_tienkung.yaml
"""

import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile

from robot_interfaces.msg import JointCommand, JointCommands, JointStates

CMD_TOPIC = "/robot/control/hardware_commands"
STATE_TOPIC = "/robot/driver/joint_states"

MODES = ("pos", "hybrid", "tor")

# kp/kd 允许上限（调试工具放宽到 1000；天工 0x11 帧 kp×10 编码到 uint16，1000→10000 安全）。
GAIN_MAX = 1000.0


def sensor_qos() -> QoSProfile:
    """与 robot_driver 的 SensorDataQoS 对齐（best_effort / keep_last 2）。"""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=2,
    )


def load_joints(args) -> list:
    """解析 robot_joints / hw 配置，返回带编号、限位、力矩上限的关节表。"""
    import yaml

    with open(args.joints_config, "r") as f:
        data = yaml.safe_load(f)
    joints = data.get("robot_joints", {}).get("joints", [])

    motor_ids = {}
    if args.hw_config:
        try:
            with open(args.hw_config, "r") as f:
                hw = yaml.safe_load(f).get("hardware", {})
            for j in hw.get("joints", []):
                motor_ids[j.get("name")] = j.get("motor_id")
        except FileNotFoundError:
            pass

    table = []
    for idx, j in enumerate(joints, 1):
        table.append({
            "index": idx,
            "name": j["name"],
            "group": j.get("group", ""),
            "min_pos": float(j.get("min_pos", -math.pi)),
            "max_pos": float(j.get("max_pos", math.pi)),
            "max_vel": float(j.get("max_vel", 1.0)),
            "max_effort": float(j.get("max_effort", 80.0)),
            "motor_id": motor_ids.get(j["name"], "-"),
        })
    return table


class ForcePositionNode(Node):
    """周期发布全量 JointCommands 帧，实现单关节力位混合控制。"""

    def __init__(self, joints: list, rate_hz: float, effort_scale: float, dry_run: bool):
        super().__init__("motor_fp_debug")
        self.joints = joints
        self.dry_run = dry_run
        self.effort_scale = effort_scale
        self.dt = 1.0 / rate_hz

        self.lock = threading.Lock()
        self.feedback = {}          # name -> JointState msg

        self.active = None          # 当前使能的关节 dict
        self.armed = None           # 选中待自动使能的关节（反馈一到即起 MIT）
        self.mode = "pos"           # pos / hybrid / tor

        # 位置命令（限速爬升）
        self.pos_target = 0.0       # 当前下发位置命令
        self.pos_goal = 0.0         # 期望到达位置
        self.speed = 0.3            # 位置爬升速度 rad/s

        # 力矩命令（力矩变化率爬升）
        self.tau_cmd = 0.0          # 当前下发前馈力矩 τ_ff
        self.tau_goal = 0.0         # 期望前馈力矩
        self.tau_rate = 20.0        # 力矩爬升速率 Nm/s

        # 增益
        self.kp = 20.0              # 位置环比例增益（pos/hybrid）
        self.kd = 1.0               # 位置环阻尼增益（pos/hybrid）
        self.damp_kd = 0.5          # 纯力矩模式下的速度阻尼增益

        self.sequence = 0

        if not dry_run:
            self.pub = self.create_publisher(JointCommands, CMD_TOPIC, sensor_qos())
            self.create_subscription(JointStates, STATE_TOPIC, self.on_states, sensor_qos())
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_frame)

    # ---------- 反馈 ----------
    def on_states(self, msg: JointStates):
        with self.lock:
            for s in msg.states:
                self.feedback[s.joint_name] = s

    def _fb(self, name: str, field: str):
        with self.lock:
            s = self.feedback.get(name)
            if s is None:
                # dry-run 无真实反馈时合成一个 0 值，便于无硬件演练菜单流程。
                return 0.0 if self.dry_run else None
            return float(getattr(s, field))

    def current_position(self, name: str):
        return self._fb(name, "position")

    def current_effort(self, name: str):
        return self._fb(name, "effort")

    def state_snapshot(self, name: str):
        """返回电机侧反馈真相（enabled/motor_error/joint_error/temp），无反馈返回 None。"""
        with self.lock:
            s = self.feedback.get(name)
        if s is None:
            return None
        return {
            "position": float(s.position), "velocity": float(s.velocity),
            "effort": float(s.effort),
            "motor_temp": float(getattr(s, "motor_temp", 0.0)),
            "enabled": bool(getattr(s, "enabled", False)),
            "motor_error": int(getattr(s, "motor_error", 0)),
            "joint_error": int(getattr(s, "joint_error", 0)),
            "error_message": str(getattr(s, "error_message", "")),
        }

    # ---------- 使能管理 ----------
    def effort_limit(self) -> float:
        """当前关节允许的 τ_ff 绝对上限。"""
        if self.active is None:
            return 0.0
        return abs(self.active["max_effort"]) * self.effort_scale

    def enable_at(self, joint: dict, pos: float):
        """使能指定关节，位置/力矩命令都从当前实测位置、0 力矩起步。"""
        # 先填齐命令字段再把 active 置上，避免发布线程读到半初始化状态。
        self.pos_target = pos
        self.pos_goal = pos
        self.tau_cmd = 0.0
        self.tau_goal = 0.0
        self.mode = "pos"
        self.armed = joint
        self.active = joint

    def arm(self, joint: dict):
        """选中关节：登记为待自动使能，收到反馈的第一时间起持续 MIT 保持，不停帧。"""
        self.armed = joint
        pos = self.current_position(joint["name"])
        if pos is not None:
            self.enable_at(joint, pos)

    def disable(self):
        self.active = None
        self.armed = None
        self.pos_target = 0.0
        self.pos_goal = 0.0
        self.tau_cmd = 0.0
        self.tau_goal = 0.0

    def set_mode(self, mode: str) -> str:
        """切换控制模式；切换时冻结命令避免突变。返回提示文本。"""
        if mode == self.mode:
            return f"已是 {self.mode} 模式"
        if mode == "tor":
            # 进入纯力矩：把位置命令冻结在当前下发值，力矩从当前 τ_cmd 起步（此时应为 0）
            self.pos_goal = self.pos_target
            self.tau_goal = self.tau_cmd
            self.mode = "tor"
            return "切到纯力矩(τ_ff)，位置命令冻结，kp=0 kd=阻尼"
        if self.mode == "tor":
            # 从纯力矩切回位置/混合：先把 τ 目标归零再恢复位置闭环，避免力矩残留突变
            self.tau_goal = 0.0
        if mode == "pos":
            self.tau_goal = 0.0
            self.mode = "pos"
            return "切到位置伺服(τ_ff=0)"
        # hybrid
        self.mode = "hybrid"
        return "切到力位混合(位置闭环 + τ_ff)"

    # ---------- 命令帧 ----------
    def publish_frame(self):
        # 选中但尚未使能（反馈刚到）：第一时间起 MIT 保持，消除停帧窗口。
        if self.armed is not None and self.active is None:
            pos = self.current_position(self.armed["name"])
            if pos is not None:
                self.enable_at(self.armed, pos)
        msg = JointCommands()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.sequence += 1
        msg.sequence = self.sequence
        msg.source = "motor_fp_debug"
        msg.type = JointCommands.TYPE_NONE

        active_name = self.active["name"] if self.active else None
        for j in self.joints:
            c = JointCommand()
            c.joint_name = j["name"]
            if j["name"] == active_name:
                self._fill_active(c)
            else:
                c.enable = False
            msg.commands.append(c)

        if not self.dry_run:
            self.pub.publish(msg)

    def _fill_active(self, c: JointCommand):
        # 位置命令限速爬升（tor 模式下 goal==target，实际不动）
        max_step = max(abs(self.speed) * self.dt, 1e-9)
        err = self.pos_goal - self.pos_target
        self.pos_target += max(-max_step, min(max_step, err))

        # 力矩命令按变化率爬升并限幅
        lim = self.effort_limit()
        self.tau_goal = max(-lim, min(lim, self.tau_goal))
        max_tstep = max(abs(self.tau_rate) * self.dt, 1e-9)
        terr = self.tau_goal - self.tau_cmd
        self.tau_cmd += max(-max_tstep, min(max_tstep, terr))
        self.tau_cmd = max(-lim, min(lim, self.tau_cmd))

        c.position = self.pos_target
        c.velocity = 0.0
        if self.mode == "pos":
            c.kp = self.kp
            c.kd = self.kd
            c.effort = 0.0
        elif self.mode == "hybrid":
            c.kp = self.kp
            c.kd = self.kd
            c.effort = self.tau_cmd
        else:  # tor
            c.kp = 0.0
            c.kd = self.damp_kd
            c.effort = self.tau_cmd
        c.enable = True


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def print_table(joints: list, active_name):
    print("=" * 82)
    print(f"{'编号':<5}{'关节名':<26}{'分组':<12}{'motor_id':<10}{'限位[rad]':<20}{'max_effort':<10}")
    for j in joints:
        mark = "  <== 当前选中" if j["name"] == active_name else ""
        print(f"{j['index']:<6}{j['name']:<27}{j['group']:<13}{str(j['motor_id']):<11}"
              f"[{j['min_pos']:+.2f}, {j['max_pos']:+.2f}]  {j['max_effort']:<8.1f}{mark}")
    print("=" * 82)


def print_help():
    print("""命令:
  【一键】go <位移> [kp] [kd] [力矩] [速度]   例: go 0.5 20 1 5 0.3
           自动 使能+设增益+选模式(有力矩→hybrid否则pos)+相对位移+立即运动
  <编号>          选择关节并自动使能 MIT 保持（自动断开上一个；选中即出力，谨防误选）
  e               重新按当前位置使能（选中已自动使能，一般无需手按）
  m pos|hybrid|tor 切换控制模式: 位置伺服 / 力位混合 / 纯力矩
  g <rad>         绝对位置目标 (例: g 0.5)  [pos/hybrid]
  r <rad>         相对位置移动 (例: r -0.2) [pos/hybrid]
  s <rad/s>       位置爬升速度 (默认 0.3, 受 max_vel 限制)
  t <Nm>          前馈力矩 τ_ff 目标 (例: t 5 / t -5) [hybrid/tor]
  w <Nm/s>        力矩爬升速率 (默认 20, τ_ff 限幅到 max_effort×scale)
  p <kp> <kd>     位置环增益 (例: p 20 1.0; 上限 1000)
  k <damp_kd>     纯力矩模式速度阻尼增益 (默认 0.5, 0 表示零阻尼)
  c               查看当前关节反馈 (pos/vel/effort)
  l               显示关节列表
  q / Ctrl+C      退出 (退出前发布全关帧)""")


def joint_loop(node: ForcePositionNode, joints: list, joint: dict):
    """选中某个关节后的子菜单；返回退出原因或新选编号字符串。"""
    j = joint
    while True:
        pos = node.current_position(j["name"])
        enabled = node.active is not None and node.active["name"] == j["name"]
        state = "使能" if enabled else "未使能"
        pos_str = f"{pos:+.4f}" if pos is not None else "无反馈"
        lim = node.effort_limit()
        # 按模式显示实际下发的增益，避免纯力矩模式误读成位置闭环仍开启。
        if node.mode == "tor":
            gain_str = f"kp=0 kd阻尼={node.damp_kd:.1f}"
        else:
            gain_str = f"kp={node.kp:.0f} kd={node.kd:.1f}"
        tau_show = node.tau_cmd if node.mode != "pos" else 0.0
        snap = node.state_snapshot(j["name"])
        if snap is None:
            truth = "电机:?无反馈"
        else:
            truth = (f"电机:en={'Y' if snap['enabled'] else 'N'}"
                     f" merr={snap['motor_error']} jerr={snap['joint_error']}"
                     f" T={snap['motor_temp']:.0f}C")
            if snap["error_message"]:
                truth += f" msg={snap['error_message']}"
        print(f"[{j['index']}] {j['name']}  模式={node.mode}  pos={pos_str}  "
              f"cmd: pos={node.pos_target:+.4f} tau={tau_show:+.2f}(限±{lim:.1f}) "
              f"{gain_str}  {state}  {truth}")
        try:
            line = input("(#编号/go/e/m/g/r/s/t/w/p/k/c/l/q) > ").strip()
        except EOFError:
            return "q"
        if not line:
            continue
        parts = line.split()
        cmd = parts[0].lower()
        try:
            if cmd == "q":
                return "q"
            elif cmd == "l":
                print_table(joints, j["name"])
            elif cmd == "c":
                snap = node.state_snapshot(j["name"])
                if snap is None:
                    print(f"{j['name']}: 无反馈（driver 未发回该关节？检查总线/网卡/driver 日志）")
                else:
                    print(f"{j['name']}: pos={snap['position']:+.4f} vel={snap['velocity']:+.3f} "
                          f"effort={snap['effort']:+.3f} T={snap['motor_temp']:.1f}C | "
                          f"enabled={snap['enabled']} motor_error={snap['motor_error']} "
                          f"joint_error={snap['joint_error']} msg='{snap['error_message']}'")
                    if not snap["enabled"]:
                        print("  ⚠ 电机侧 enabled=False：0x10 使能未锁存（可能启动握手失败/已看门狗失能）→ 重启 driver 重握手")
                    if snap["motor_temp"] <= 0.0 and abs(snap["position"]) < 1e-6:
                        print("  ⚠ temp=0 且 pos 恒 0：总线可能未真正上返（从站未到 OP/网卡错/setcap 丢）")
            elif cmd == "e":
                if pos is None:
                    print("未收到该关节反馈，先确认 driver 已启动？暂不敢使能。")
                else:
                    node.enable_at(j, pos)
                    print(f"已使能 {j['name']} (pos 模式)，保持 {pos:+.4f}")
            elif cmd == "go":
                # 一键运动: go <相对位移rad> [kp] [kd] [力矩Nm] [速度rad/s]
                # 自动完成 使能 + 设增益 + 选模式(有力矩则hybrid否则pos) + 相对位移 + 立即运动。
                if len(parts) < 2:
                    print("用法: go <相对位移rad> [kp] [kd] [力矩Nm] [速度rad/s]  例: go 0.5 20 1 5 0.3")
                    continue
                rel = float(parts[1])
                g_kp = float(parts[2]) if len(parts) > 2 else node.kp
                g_kd = float(parts[3]) if len(parts) > 3 else node.kd
                g_tau = float(parts[4]) if len(parts) > 4 else None
                g_speed = float(parts[5]) if len(parts) > 5 else node.speed
                base = pos
                if not enabled:
                    if pos is None:
                        print("无反馈位置，不敢使能。")
                        continue
                    node.enable_at(j, pos)
                    base = pos
                # 位移基准用当前命令值，避免从反馈跳变；先更新增益/速度。
                node.kp = clamp(g_kp, 0.0, GAIN_MAX)
                node.kd = clamp(g_kd, 0.0, GAIN_MAX)
                node.speed = clamp(g_speed, 0.01, j["max_vel"])
                # 模式: 给了非零力矩 -> hybrid; 否则确保离开 tor(位置冻结)回到 pos。
                if g_tau is not None and abs(g_tau) > 1e-9:
                    node.mode = "hybrid"
                    lim = node.effort_limit()
                    node.tau_goal = clamp(g_tau, -lim, lim)
                else:
                    if node.mode == "tor":
                        node.set_mode("pos")
                    if g_tau is not None:      # 显式 t 0 -> 力矩归零
                        node.tau_goal = 0.0
                new_goal = clamp(base + rel, j["min_pos"], j["max_pos"])
                node.pos_goal = new_goal
                print(f"[go] {j['name']} 模式={node.mode} 目标 {base:+.4f}->{new_goal:+.4f} rad "
                      f"(幅度 {abs(new_goal-base):.3f}) kp={node.kp:.0f} kd={node.kd:.1f} "
                      f"τ={node.tau_goal:+.2f} 速度={node.speed:.2f}")
            elif cmd == "m":
                if not enabled:
                    print("请先用 e 使能再切模式。")
                    continue
                if len(parts) < 2 or parts[1].lower() not in MODES:
                    print("用法: m pos | m hybrid | m tor")
                    continue
                print(node.set_mode(parts[1].lower()))
            elif cmd in ("g", "r"):
                if len(parts) < 2:
                    print("需要参数，例: g 0.5 / r -0.2")
                    continue
                val = float(parts[1])
                if not enabled:
                    if pos is None:
                        print("无反馈位置，不敢使能。")
                        continue
                    node.enable_at(j, pos)
                    print(f"自动使能 {j['name']}，保持 {pos:+.4f}")
                if node.mode == "tor":
                    print("纯力矩模式位置命令已冻结，先 m pos/hybrid 再改位置。")
                    continue
                new_goal = (node.pos_goal + val) if cmd == "r" else val
                new_goal = clamp(new_goal, j["min_pos"], j["max_pos"])
                node.pos_goal = new_goal
                print(f"位置目标 -> {new_goal:+.4f} rad (当前命令 {node.pos_target:+.4f})")
            elif cmd == "s":
                node.speed = clamp(float(parts[1]), 0.01, j["max_vel"])
                print(f"位置爬升速度 {node.speed:.2f} rad/s")
            elif cmd == "t":
                if not enabled:
                    print("请先用 e 使能。")
                    continue
                if node.mode == "pos":
                    print("pos 模式 τ_ff 恒为 0，先 m hybrid 或 m tor 再设力矩。")
                    continue
                if len(parts) < 2:
                    print("需要参数，例: t 5 / t -5")
                    continue
                lim = node.effort_limit()
                val = clamp(float(parts[1]), -lim, lim)
                node.tau_goal = val
                print(f"前馈力矩目标 -> {val:+.2f} Nm (限幅 ±{lim:.1f}, 爬升 {node.tau_rate:.0f} Nm/s)")
            elif cmd == "w":
                node.tau_rate = clamp(float(parts[1]), 0.1, 1000.0)
                print(f"力矩爬升速率 {node.tau_rate:.1f} Nm/s")
            elif cmd == "p":
                node.kp = clamp(float(parts[1]), 0.0, GAIN_MAX)
                node.kd = clamp(float(parts[2]), 0.0, GAIN_MAX) if len(parts) > 2 else node.kd
                print(f"位置环增益 kp={node.kp:.1f} kd={node.kd:.1f}（上限 {GAIN_MAX:.0f}；kp 越大越易放大命令尖峰）")
            elif cmd == "k":
                node.damp_kd = clamp(float(parts[1]), 0.0, GAIN_MAX)
                print(f"纯力矩速度阻尼 kd={node.damp_kd:.1f}（上限 {GAIN_MAX:.0f}，0=零阻尼，易飞车，慎用）")
            elif cmd.isdigit():
                idx = int(cmd)
                nxt = next((x for x in joints if x["index"] == idx), None)
                if nxt is None:
                    print("无效编号")
                else:
                    if node.active is not None:
                        node.disable()
                        print("上一个关节已断使能")
                    return str(idx)
            else:
                print("未知命令，输入 l 看列表，q 退出")
        except (ValueError, IndexError):
            print("参数解析失败，例: g 0.5 / t 5 / p 20 1.0")


def main():
    parser = argparse.ArgumentParser(description="单电机力位混合控制调试工具")
    parser.add_argument("--joints-config", required=True, help="robot_joints_*.yaml 路径")
    parser.add_argument("--hw-config", default="", help="hw_*.yaml 路径（仅用于显示 motor_id）")
    parser.add_argument("--rate", type=float, default=200.0,
                        help="命令帧发布频率 Hz（默认 200，对齐生产链路；天工电机 100ms 无新帧即失能，需高频持续供帧）")
    parser.add_argument("--effort-scale", type=float, default=0.3,
                        help="τ_ff 限幅比例，实际上限 = max_effort × 该值 (默认 0.3)")
    parser.add_argument("--dry-run", action="store_true", help="不连接 driver，仅演示菜单")
    parser.add_argument("--yes", action="store_true", help="跳过危险确认（仅供脚本化测试）")
    args = parser.parse_args()

    joints = load_joints(args)
    if not joints:
        print("未解析到任何关节，请检查 --joints-config")
        sys.exit(1)

    if not args.dry_run and not args.yes:
        print("\n" + "!" * 82)
        print("! 危险确认：本工具直接驱动真实电机。现在“选中即自动使能”，")
        print("! 输入编号后该关节会立即 MIT 保持(出力变硬)，无需再按 e，谨防误选。")
        print("! 只有单关节使能、其余全部断使能 —— 站立的机器人会立即倾倒。")
        print("! 纯力矩/力位混合模式会使电机主动出力，务必台架固定、轮子/脚不承重。")
        print("!" * 82)
        ans = input("输入 yes 继续，其他任意键退出: ").strip().lower()
        if ans != "yes":
            print("已取消")
            sys.exit(0)

    rclpy.init()
    node = ForcePositionNode(joints, args.rate, args.effort_scale, args.dry_run)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while True:
            print_table(joints, node.active["name"] if node.active else None)
            print_help()
            try:
                line = input("选择关节编号 (q 退出): ").strip().lower()
            except EOFError:
                break
            if line in ("q", "quit", "exit"):
                break
            if line.isdigit():
                idx = int(line)
                current = next((x for x in joints if x["index"] == idx), None)
                if current is None:
                    print("无效编号")
                    continue
                if node.active is not None:
                    node.disable()
                    print("上一个关节已断使能")
                node.arm(current)
                print(f"已选中 {current['name']}，持续 MIT 保持中（不停帧）。可直接 g/r/m/t 或 go 运动。")
                while current is not None:
                    ret = joint_loop(node, joints, current)
                    if ret == "q":
                        current = None
                        break
                    nxt_idx = int(ret)
                    if nxt_idx == current["index"]:
                        continue
                    current = next((x for x in joints if x["index"] == nxt_idx), None)
                    if current is not None:
                        node.disable()
                        node.arm(current)
                        print(f"已切换到 {current['name']}，持续 MIT 保持中。")
            else:
                print("请输入列表中的编号或 q")
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在安全退出...")
    finally:
        node.disable()
        time.sleep(0.3)
        if args.dry_run:
            for _ in range(5):
                node.publish_frame()
        print("已发布全关节断使能帧")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
