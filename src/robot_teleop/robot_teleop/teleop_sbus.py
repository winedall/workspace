"""云卓 G12 遥控器专用远程控制节点：SBUS 通道直接生成 RobotIntent。
按 G12 自身的按键/摇杆语义解析 SBUS 通道，发布 /robot/input/intent。
"""

from __future__ import annotations

import glob
import os
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import RobotIntent, RobotState

from robot_teleop.intent_builder import IntentBuilder, IntentSpec

# 标准 SBUS 协议常量
SBUS_FRAME_LEN = 25
SBUS_HEADER = 0x0F
SBUS_TAIL = 0x00
SBUS_CH_COUNT = 16
SBUS_BITS_PER_CH = 11

# 自动探测串口时限：SBUS 约 14ms 一帧，能抓至少一帧即可判定为遥控器接收机。
SBUS_PROBE_TIMEOUT_SEC = 0.5

# action 动作配置菜单默认动作清单（平铺，注册名须与 config/humanoid/rl_models.yaml 一一对应）。
# 覆盖动作类（单次/持续）、舞蹈类、特技类；长按确认键触发 model/switch_model/<action>。
DEFAULT_ACTION_OPTIONS = [
    # 动作类（单次）
    "t4_heart", "t4_salute", "t4_hello", "t4_high_five",
    # 动作类（持续）
    "t4_rhello", "t4_clap", "t4_handshake", "t4_standby",
    # 舞蹈类
    "t4_taichi", "t4_taichi_w", "t4_mj_dance", "t4_xqxa123_dance", "t4_apt", "t4_rhythm",
    # 特技类
    "t4_long_jump_l", "t4_side_flip", "t4_front_flip",
    "t4_high_jump_02", "t4_high_jump_03", "t4_jd_jump",
]

# 一级菜单序列（循环顺序）。
DEFAULT_CONFIG_PRIMARIES = ["getup", "speed", "nav", "mode", "action"]

# config 二级菜单：一级名 -> 可选值列表。mode 含 app（应用模式）。
DEFAULT_CONFIG_MENU_OPTIONS = {
    "getup": ["stand", "lie", "face"],
    "speed": ["std", "high"],
    "nav": ["manual", "auto", "follow"],
    "mode": ["rl", "mix", "spec", "app"],
}

# 配置/动作菜单采用「一级菜单 + 二级菜单」结构。
#   - 一级菜单：getup(起身) / speed(速度) / nav(导航) / mode(模式) / action(动作配置)，顺序可配。
#   - 二级菜单：每项一级菜单下的可选项（config 类：stand/lie/face 等；action 类：动作清单）。
# 一级菜单序列/二级选项/动作清单默认取上面的代码常量，也可由 yaml 参数覆盖。
# 交互：G/H 滚轮浏览（右=下/左=上/循环）；仅 E 高档+F 低档(总开关)时全部操作有效；
#       action 菜单内长按 D(3s) 触发所选中动作；连续 3 次 A/B/C/D 切换状态。

# 三级段开关档位阈值（低/高判定复用三段开关 650/1350 基准）。
GROUP_LOW_RAW = 650.0
GROUP_HIGH_RAW = 1350.0


def g12_switch_high(raw: int, low: float = GROUP_LOW_RAW, high: float = GROUP_HIGH_RAW) -> bool:
    """三段开关是否处于高档（raw >= 高档阈值）。"""
    return raw >= high


def g12_switch_low(raw: int, low: float = GROUP_LOW_RAW) -> bool:
    """三段开关是否处于低档（raw <= 低档阈值）。"""
    return raw <= low


def g12_switch_middle(raw: int, low: float = GROUP_LOW_RAW, high: float = GROUP_HIGH_RAW) -> bool:
    """三段开关是否处于中档（介于低/高阈值之间）。"""
    return low < raw < high


def sbus_frame_to_channels(frame: bytes) -> list[int]:
    """把 25 字节标准 SBUS 帧解析为 16 通道的 11-bit 原始值（0..2047）。"""
    if len(frame) < SBUS_FRAME_LEN:
        raise ValueError(f"SBUS frame too short: {len(frame)}")
    if frame[0] != SBUS_HEADER:
        raise ValueError("SBUS frame missing header 0x0F")
    if frame[-1] != SBUS_TAIL:
        raise ValueError("SBUS frame missing tail 0x00")

    channels = [0] * SBUS_CH_COUNT
    bit_index = 0
    for ch in range(SBUS_CH_COUNT):
        value = 0
        shift = 0
        for _ in range(SBUS_BITS_PER_CH):
            byte_index = bit_index // 8
            bit_in_byte = bit_index % 8
            if frame[1 + byte_index] & (1 << bit_in_byte):
                value |= 1 << shift
            bit_index += 1
            shift += 1
        channels[ch] = value
    return channels


class _SbusDecoder:
    """串口字节流到 SBUS 帧的滑动窗口解析器。"""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[bytes]:
        frames: list[bytes] = []
        self._buffer.extend(data)
        while True:
            start = self._buffer.find(SBUS_HEADER)
            if start < 0:
                if len(self._buffer) > 0:
                    self._buffer = self._buffer[-1:]
                break
            if start > 0:
                del self._buffer[:start]
            if len(self._buffer) < SBUS_FRAME_LEN:
                break
            candidate = bytes(self._buffer[:SBUS_FRAME_LEN])
            if candidate[-1] == SBUS_TAIL:
                frames.append(candidate)
                del self._buffer[:SBUS_FRAME_LEN]
            else:
                del self._buffer[0]
        return frames


class G12InputConfig:
    """G12 通道映射与按键阈值配置（纯数据，便于测试）。"""

    def __init__(
        self,
        axis_map: list[int],
        axis_invert: list[bool],
        max_linear_x_mps: float,
        max_linear_y_mps: float,
        max_linear_x_high_mps: float,
        max_linear_y_high_mps: float,
        max_angular_z_radps: float,
        deadzone: float,
        gh_channel_g: int,
        gh_channel_h: int,
        gh_center: float,
        gh_active_threshold: float,
        auth_a_channel: int,
        auth_b_channel: int,
        auth_c_channel: int,
        auth_d_channel: int,
        button_press_threshold: float,
        e_channel: int,
        f_channel: int,
        config_primaries: list[str],
        config_menu_options: dict[str, list[str]],
        action_options: list[str],
        confirm_key: str = "D",
        long_press_timeout_sec: float = 3.0,
        triple_timeout_sec: float = 1.5,
        tap_count: int = 3,
    ) -> None:
        self.axis_map = list(axis_map)
        self.axis_invert = list(axis_invert)
        self.max_linear_x_mps = max_linear_x_mps
        self.max_linear_y_mps = max_linear_y_mps
        self.max_linear_x_high_mps = max_linear_x_high_mps
        self.max_linear_y_high_mps = max_linear_y_high_mps
        self.max_angular_z_radps = max_angular_z_radps
        self.deadzone = deadzone
        self.gh_channel_g = gh_channel_g
        self.gh_channel_h = gh_channel_h
        self.auth_a_channel = auth_a_channel
        self.auth_b_channel = auth_b_channel
        self.auth_c_channel = auth_c_channel
        self.auth_d_channel = auth_d_channel
        self.button_press_threshold = button_press_threshold
        self.gh_center = gh_center
        self.gh_active_threshold = gh_active_threshold
        self.e_channel = e_channel
        self.f_channel = f_channel
        # 一级菜单序列（顺序即循环顺序），默认含 action(动作配置)。
        self.config_primaries = list(config_primaries or [])
        # config 二级菜单：一级名 -> 可选值列表。
        self.config_menu_options = {
            name: list(opts or []) for name, opts in (config_menu_options or {}).items()
        }
        # action 一级菜单二级：动作清单（平铺，安全+危险）。
        self.action_options = list(action_options or [])
        # 动作确认键（默认 C）与长按/连按时序阈值。
        self.confirm_key = confirm_key
        self.long_press_timeout_sec = long_press_timeout_sec
        self.triple_timeout_sec = triple_timeout_sec
        self.tap_count = int(max(tap_count, 1))


@dataclass
class G12Input:
    """一帧 G12 通道原始信号（只做分类，不含控制器状态机）。

    字段：
      - control_enabled：E(CH5) 高档 且 F(CH6) 低档 时总开关开启，其余操作才有效。
      - g_deflect / h_deflect：G/H 滚轮当前偏转方向（-1 左 / 1 右 / 0 中），
        状态机用「偏转 -> 回中」沿判一次拨动。
      - a/b/c/d：当前 A/B/C/D 是否按下。
      - axes：归一化摇杆轴 [linear_y(前进), linear_x(横移), angular_z(转身)]。
    """
    control_enabled: bool
    g_deflect: int
    h_deflect: int
    a: bool
    b: bool
    c: bool
    d: bool
    axes: list[float]


def _wheel_deflect(raw: int, conf: G12InputConfig) -> int:
    """弹簧回中滚轮偏转方向：偏离中值即视为拨向一边，回中为 0。

    - 右偏（raw > center + threshold）-> 1
    - 左偏（raw < center - threshold）-> -1
    - 其余（含中值附近死区）-> 0
    """
    if raw > conf.gh_center + conf.gh_active_threshold:
        return 1
    if raw < conf.gh_center - conf.gh_active_threshold:
        return -1
    return 0


def classify_input(channels: list[int], conf: G12InputConfig) -> G12Input:
    """从一帧通道值提取分类信号（不含控制器状态机）。

    返回值用于节点状态机做拨动边沿、连按、长按等处理。
    """
    def ch_value(c: int) -> int:
        return channels[c - 1] if 1 <= c <= len(channels) else conf.gh_center

    def pressed(ch: int) -> bool:
        return 1 <= ch <= len(channels) and channels[ch - 1] >= conf.button_press_threshold

    # 总开关：E 高档 且 F 低档 才可控制。
    control_enabled = (
        g12_switch_high(ch_value(conf.e_channel))
        and g12_switch_low(ch_value(conf.f_channel))
    )

    g_deflect = _wheel_deflect(ch_value(conf.gh_channel_g), conf)
    h_deflect = _wheel_deflect(ch_value(conf.gh_channel_h), conf)

    # 归一化摇杆轴。
    axes = [0.0, 0.0, 0.0]
    for i, ch in enumerate(conf.axis_map):
        if i >= len(axes) or not (1 <= ch <= len(channels)):
            continue
        val = max(-1.0, min(1.0, (channels[ch - 1] - 1023.5) / 1023.5))
        if i < len(conf.axis_invert) and conf.axis_invert[i]:
            val = -val
        if abs(val) < conf.deadzone:
            val = 0.0
        axes[i] = round(val, 6)

    return G12Input(
        control_enabled=control_enabled,
        g_deflect=g_deflect,
        h_deflect=h_deflect,
        a=pressed(conf.auth_a_channel),
        b=pressed(conf.auth_b_channel),
        c=pressed(conf.auth_c_channel),
        d=pressed(conf.auth_d_channel),
        axes=axes,
    )


class TeleopSbusNode(Node):
    """读 G12 接收机 SBUS 串口，直接按 G12 语义生成 RobotIntent。"""

    # A+B 长按运动停止/恢复循环序列（stop -> resume -> stop ...）。
    _MOTION_STOP_SEQUENCE = ("stop", "resume")

    def __init__(self) -> None:
        super().__init__("teleop_sbus_node")

        # ---- 基础参数 ----
        self._source_name = str(self.declare_parameter("source_name", "teleop").value)
        self._serial_port = str(self.declare_parameter("serial_port", "/dev/ttyUSB0").value)
        self._baudrate = int(self.declare_parameter("baudrate", 100000).value)
        self._publish_frequency_hz = float(self.declare_parameter("publish_frequency_hz", 50.0).value)
        self._timeout_sec = float(self.declare_parameter("timeout_sec", 0.3).value)

        # ---- 运动轴参数 ----
        # axis_map 值为遥控器通道号（1 起），依次对应：
        #   axes[0]（前进 linear_y）、axes[1]（横移 linear_x）、axes[2]（转身 angular_z）。
        self._axis_map = list(map(int, self.declare_parameter(
            "axis_map", [4, 3, 1, 2]).value))
        self._axis_invert = list(map(bool, self.declare_parameter(
            "axis_invert", [True, False, True, False]).value))
        self._deadzone = float(self.declare_parameter("joy_deadzone", 0.08).value)

        # ---- 速度参数（与旧 teleop_joy 语义一致：linear_y=前进, linear_x=横移）----
        self._max_linear_x_mps = float(self.declare_parameter("max_linear_x_mps", 1.0).value)
        self._max_linear_y_mps = float(self.declare_parameter("max_linear_y_mps", 0.5).value)
        self._max_linear_x_high_mps = float(
            self.declare_parameter("max_linear_x_high_mps", 3.0).value)
        self._max_linear_y_high_mps = float(
            self.declare_parameter("max_linear_y_high_mps", 1.0).value)
        self._max_angular_z_radps = float(
            self.declare_parameter("max_angular_z_radps", 1.5).value)
        # 速度模式：std/高速，由 speed 菜单二级选项切换（config/speed/std|high）。
        self._speed_mode = "std"

        # ---- G12 按键通道 ----
        # GH 拨动 = G(CH11) 或 H(CH12) 任一偏离中值即视为拨动（左右均算，回中自动取消）。
        self._gh_channel_g = int(self.declare_parameter("gh_channel_g", 11).value)
        self._gh_channel_h = int(self.declare_parameter("gh_channel_h", 12).value)
        self._gh_center = float(self.declare_parameter("gh_center", 1002.0).value)
        self._gh_active_threshold = float(
            self.declare_parameter("gh_active_threshold", 120.0).value)
        # 模式按键（瞬时非自锁：按下~1722，松开~282）。
        self._auth_a_channel = int(self.declare_parameter("auth_a_channel", 7).value)
        self._auth_b_channel = int(self.declare_parameter("auth_b_channel", 8).value)
        self._auth_c_channel = int(self.declare_parameter("auth_c_channel", 9).value)
        self._auth_d_channel = int(self.declare_parameter("auth_d_channel", 10).value)
        self._button_press_threshold = float(
            self.declare_parameter("button_press_threshold", 1350.0).value)

        # ---- 配置/动作菜单（一级 + 二级，G/H 滚轮浏览）----
        # 总开关 E(CH5) 高档 + F(CH6) 低档 才可控制。
        self._e_channel = int(self.declare_parameter("e_channel", 5).value)
        self._f_channel = int(self.declare_parameter("f_channel", 6).value)
        # 一级菜单序列（默认含 action 动作配置）。
        self._config_primaries = [str(x) for x in self.declare_parameter(
            "config_primaries", DEFAULT_CONFIG_PRIMARIES).value]
        # config 二级菜单：一级名 -> 可选值列表（默认取代码常量，也可由 yaml 块字符串覆盖）。
        self._config_menu_options = self._declare_list_map(
            "config_menu_options", DEFAULT_CONFIG_MENU_OPTIONS)
        # action 一级菜单二级：动作清单（默认取代码常量，也可由 yaml 列表覆盖）。
        self._action_options = [
            str(x) for x in self.declare_parameter(
                "action_options", DEFAULT_ACTION_OPTIONS).value]
        # 动作确认键与长按/连按时序。
        self._confirm_key = str(self.declare_parameter("confirm_key", "D").value)
        self._long_press_timeout_sec = float(
            self.declare_parameter("long_press_timeout_sec", 3.0).value)
        self._triple_timeout_sec = float(
            self.declare_parameter("triple_timeout_sec", 1.5).value)
        self._tap_count = int(self.declare_parameter("tap_count", 3).value)

        # 汇总成可测的输入配置。
        self._conf = G12InputConfig(
            axis_map=self._axis_map,
            axis_invert=self._axis_invert,
            max_linear_x_mps=self._max_linear_x_mps,
            max_linear_y_mps=self._max_linear_y_mps,
            max_linear_x_high_mps=self._max_linear_x_high_mps,
            max_linear_y_high_mps=self._max_linear_y_high_mps,
            max_angular_z_radps=self._max_angular_z_radps,
            deadzone=self._deadzone,
            gh_channel_g=self._gh_channel_g,
            gh_channel_h=self._gh_channel_h,
            gh_center=self._gh_center,
            gh_active_threshold=self._gh_active_threshold,
            auth_a_channel=self._auth_a_channel,
            auth_b_channel=self._auth_b_channel,
            auth_c_channel=self._auth_c_channel,
            auth_d_channel=self._auth_d_channel,
            button_press_threshold=self._button_press_threshold,
            e_channel=self._e_channel,
            f_channel=self._f_channel,
            config_primaries=self._config_primaries,
            config_menu_options=self._config_menu_options,
            action_options=self._action_options,
            confirm_key=self._confirm_key,
            long_press_timeout_sec=self._long_press_timeout_sec,
            triple_timeout_sec=self._triple_timeout_sec,
            tap_count=self._tap_count,
        )

        # 发布器 / 意图构造。
        self._builder = IntentBuilder(source_name=self._source_name, priority=0)
        self._intent_pub = self.create_publisher(RobotIntent, "/robot/input/intent", 10)

        # 连续运动缓存（timer 按固定频率发布）；模式/事件意图直接 emit。
        self._latest_motion: Optional[IntentSpec] = None

        # 运动授权状态（连续 B 短按 xN = walk，授予后可推杆运动）。
        self._motion_enabled = False
        self._motion_center_required = False

        # 控制器状态机（_process_controller）：
        #   总开关(E 高+F 低) -> G/H 滚轮浏览菜单 -> 长按(A+B 急停/恢复、A 循环结束) -> 连按(A/B/C 状态)。
        self._control_enabled = False          # 上一帧总开关状态
        self._primary_index = 0          # 当前一级菜单在 config_primaries 中的索引
        self._option_index = 0           # 当前一级菜单已选的二级选项索引
        self._prev_g_deflect = 0         # 上次 G 滚轮偏转（-1/0/1），用于「偏转->回中」沿
        self._prev_h_deflect = 0         # 上次 H 滚轮偏转（-1/0/1）
        # 连按（连续 N 次短按切换状态）跟踪。
        self._tap_key: Optional[str] = None   # 当前正在累计连按的键（A/B/C）
        self._tap_count_now = 0               # 已累计的连按次数
        self._tap_window_until = 0.0          # 连按超时时间戳（超时清零）
        # 长按确认触发跟踪（动作菜单内长按 confirm_key 3s）。
        self._press_start_sec: Optional[float] = None  # 当前键按下起始时间戳
        self._longpress_handled = False               # 本次按下是否已触发长按动作
        # 上一帧 A/B/C/D 按键状态（连按按下沿检测用）。
        self._prev_a = False
        self._prev_b = False
        self._prev_c = False
        self._prev_d = False
        # 长按（hold）跟踪：A+B 长按 = 运动停止/恢复，A 单独长按 = 循环结束。
        self._hold_key: Optional[str] = None   # 当前长按组合："ab" / "a" / None
        self._hold_start_sec: Optional[float] = None
        self._hold_triggered = False
        # A+B 长按运动停止/恢复循环索引（stop -> resume -> stop ...）。
        self._motion_stop_index = -1

        # 失联停车条件。
        self._system_state = RobotState.STATE_BOOTING
        self._state_sub = self.create_subscription(
            RobotState, "/robot/state", self._handle_state, 10)
        self._last_read_time = 0.0
        self._published_timeout_stop = False

        # SBUS 串口读取。
        self._serial = None
        self._decoder = _SbusDecoder()
        self._stop = threading.Event()
        # 串口打不开/无数据时的日志限频（避免刷屏），两次日志最小间隔秒数。
        self._serial_error_log_interval_sec = 120.0
        self._last_open_error_log_time = 0.0
        self._last_no_data_log_time = 0.0
        self._open_serial()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        period_sec = 1.0 / max(self._publish_frequency_hz, 1.0)
        self._timer = self.create_timer(period_sec, self._handle_timer)

        self.get_logger().info(
            f"teleop_sbus node started: {self._serial_port}@{self._baudrate} "
            f"axis_map={self._axis_map} invert={self._axis_invert}"
        )

    # ---------- 串口 ----------

    def _declare_list_map(
        self, param_name: str, default: Optional[dict[str, list[str]]] = None,
    ) -> dict[str, list[str]]:
        """读取 `{一级: [可选值]}` 参数为 {一级名: [可选值]}。

        - 默认取代码常量（default，dict），也可由 yaml 块字符串覆盖，例如：
          ```
          config_menu_options: |
            getup: [stand, lie, face]
            speed: [std, high]
            nav: [manual, auto, follow]
            mode: [rl, mix, spec, app]
          ```
        """
        import yaml

        # ROS 参数只接受标量/数组，dict 默认值需先序列化成 yaml 字符串。
        default_raw = ""
        if isinstance(default, dict):
            default_raw = yaml.safe_dump(default, default_flow_style=False, sort_keys=False)
        raw = self.declare_parameter(param_name, default_raw).value
        result: dict[str, list[str]] = {}
        if isinstance(raw, str) and raw.strip():
            try:
                parsed = yaml.safe_load(raw) or {}
            except yaml.YAMLError:
                self.get_logger().error(f"failed to parse {param_name} as yaml")
                return result
            if isinstance(parsed, dict):
                for name, opts in parsed.items():
                    if isinstance(opts, list):
                        result[str(name)] = [str(o) for o in opts]
        return result

    def _open_serial(self) -> None:
        try:
            import serial
        except ImportError as exc:
            self.get_logger().error(f"pyserial not available: {exc}")
            self._serial = None
            return
        # 解析最终的串口设备名（含绝对映射/自动探测）。
        resolved_port = self._resolve_port()
        if resolved_port is None:
            if self._serial is not None:
                self._serial = None
            self._log_rate_limited(
                "_last_open_error_log_time",
                f"no SBUS receiver serial found (configured='{self._serial_port}')",
            )
            return
        try:
            self._serial = serial.Serial(
                port=resolved_port,
                baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_EVEN,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,
            )
            self._serial.reset_input_buffer()
            self.get_logger().info(
                f"opened {resolved_port} @ {self._baudrate} bps 8E1"
            )
        except Exception as exc:  # noqa: BLE001
            # 串口反复打不开时按最小间隔限频报错，避免刷屏。
            self._last_open_error_log_time = 0.0
            self._log_rate_limited(
                "_last_open_error_log_time",
                f"failed to open {resolved_port}: {exc}",
            )
            self._serial = None

    def _resolve_port(self) -> Optional[str]:
        """返回当前实际应使用的串口设备；找不到时返回 None。

        依次尝试：
          1. yaml 中的 serial_port 若指向 /dev/serial/by-id/ 绝对链接，直接用它；
          2. 若指向普通 /dev/ttyUSB* 且该设备存在，直接用它（保留旧行为）；
          3. 否则从 /dev/serial/by-id 里挑一个能解析出标准 SBUS 帧的串口（自动探测）。
        """
        configured = self._serial_port
        # 1) 显式绝对链接：直接使用。
        is_byid = "/serial/by-id/" in configured or "/serial/by-path/" in configured
        if is_byid and os.path.exists(configured):
            self.get_logger().info(
                f"using stable serial link: {configured}"
            )
            return configured
        # 2) 显式 /dev/tty* 且存在：沿用。
        if configured.startswith("/dev/") and os.path.exists(configured):
            return configured
        # 3) 自动探测：遍历 by-id 串口，按 SBUS 帧特征匹配遥控器。
        return self._probe_serial()

    def _probe_serial(self) -> Optional[str]:
        """在常见串口里自动探测 G12 SBUS 遥控器接收机。"""
        candidates: list[str] = []
        for pattern in (
            "/dev/serial/by-id/*",
            "/dev/serial/by-path/*",
            "/dev/ttyUSB*",
            "/dev/ttyACM*",
        ):
            candidates.extend(sorted(glob.glob(pattern)))
        for cand in candidates:
            if self._detect_sbus_on(cand):
                self.get_logger().info(f"auto-detected SBUS receiver on {cand}")
                return cand
        return None

    def _detect_sbus_on(self, path: str) -> bool:
        """短暂打开串口，能解析出标准 SBUS 帧（0x0F 头 / 0x00 尾）即视为遥控器接收机。"""
        try:
            import serial
        except ImportError:
            return False
        try:
            cand = serial.Serial(
                port=path,
                baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_EVEN,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,
            )
        except Exception:  # noqa: BLE001
            return False
        try:
            cand.reset_input_buffer()
            decoder = _SbusDecoder()
            deadline = time.monotonic() + SBUS_PROBE_TIMEOUT_SEC
            while time.monotonic() < deadline:
                data = cand.read(64)
                if data and decoder.feed(data):
                    return True
        except Exception:  # noqa: BLE001
            return False
        finally:
            try:
                cand.close()
            except Exception:  # noqa: BLE001
                pass
        return False

    def _log_rate_limited(self, attr: str, message: str) -> None:
        """按 _serial_error_log_interval_sec 限频记录错误日志，避免刷屏。"""
        now = time.monotonic()
        last = getattr(self, attr, None) or 0.0
        if now - last < self._serial_error_log_interval_sec:
            return
        setattr(self, attr, now)
        self.get_logger().error(message)

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            if self._serial is None or not self._serial.is_open:
                time.sleep(0.5)
                self._open_serial()
                continue
            try:
                data = self._serial.read(64)
                if data:
                    for frame in self._decoder.feed(data):
                        self._handle_frame(frame)
                else:
                    self._no_data_waiver()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().debug(f"read error: {exc}")
                time.sleep(0.01)

    def _no_data_waiver(self) -> None:
        if self._last_read_time == 0.0:
            port = self._serial.port if self._serial is not None else self._serial_port
            self.get_logger().warn(
                f"no SBUS data on {port} (baud={self._baudrate}). "
                "Check GR01 SBUS wiring and baudrate."
            )

    # ---------- 帧处理 ----------

    def _handle_frame(self, frame: bytes) -> None:
        try:
            channels = sbus_frame_to_channels(frame)
        except ValueError as exc:
            self.get_logger().warn_once(f"invalid SBUS frame: {exc}")
            return
        self._last_read_time = time.monotonic()
        self._published_timeout_stop = False
        self._process_channels(channels)

    def _handle_state(self, msg: RobotState) -> None:
        self._system_state = msg.state

    # ---------- 通道 -> 意图 ----------

    def _process_channels(self, channels: list[int]) -> None:
        sig = classify_input(channels, self._conf)
        self._process_controller(sig)

    def _process_controller(self, sig: G12Input) -> None:
        """G12 控制器状态机（总开关 -> G/H 菜单 -> 长按/连按 -> 运动）。

        总开关 control_enabled(E 高档 且 F 低档)关闭时：遥控器完全无控制，
        挂起菜单滑动与按键逻辑，也不产生运动（唯一例外由失联停车 timer 负责）。
        """
        self._control_enabled = sig.control_enabled

        if not sig.control_enabled:
            # 关闭时挂起所有键/滚轮逻辑（清空中间态，避免重新打开时误触发）。
            self._reset_controller_state()
            self._hold_key = None
            self._hold_start_sec = None
            self._hold_triggered = False
            # 暂停运动输出，但保留 walk 授权；重新开启后需先回中一次才恢复运动（安全）。
            self._latest_motion = None
            self._motion_center_required = True
            self._prev_g_deflect = sig.g_deflect
            self._prev_h_deflect = sig.h_deflect
            self._prev_a = sig.a
            self._prev_b = sig.b
            self._prev_c = sig.c
            self._prev_d = sig.d
            return

        # 时钟（便于测试注入）。
        now = self._now()

        # 0) 运动摇杆轴。
        axes = sig.axes

        # 1) 菜单浏览（G 滚一级 / H 滚二级），切换二级选项即播报 config 意图。
        self._update_menu(sig)

        # 2) 动作触发（action 菜单内长按 confirm_key 3s）。
        self._update_action_longpress(sig, now)

        # 3) 长按（A+B 运动停止/恢复、A 单独循环结束）。
        self._update_holds(sig, now)

        # 4) 连续短按（tap_count 次）切换状态/授权。
        self._update_taps(sig, now)

        # 更新上一帧 A/B/C/D 按键状态（边沿检测基准）。
        self._prev_a = sig.a
        self._prev_b = sig.b
        self._prev_c = sig.c
        self._prev_d = sig.d

        # 5) 连续运动：需要 walk 授权且已回中。
        if self._motion_enabled:
            if self._motion_center_required:
                if self._axes_centered(axes):
                    self._motion_center_required = False
                else:
                    self._latest_motion = IntentSpec(intent_type="motion", command="walk")
                    return
            self._latest_motion = self._build_motion(axes)

    def _reset_controller_state(self) -> None:
        self._tap_key = None
        self._tap_count_now = 0
        self._press_start_sec = None
        self._longpress_handled = False

    def _update_menu(self, sig: G12Input) -> None:
        """G 滚一级 / H 滚当前一级的二级（含 action 动作清单）；「偏转->回中」沿判一次拨动。"""
        if not self._config_primaries:
            self._prev_g_deflect = sig.g_deflect
            self._prev_h_deflect = sig.h_deflect
            return

        # G 滚轮拨动 -> 一级菜单步进并复位二级。
        if self._prev_g_deflect != 0 and sig.g_deflect == 0:
            step = self._prev_g_deflect  # 右(+1)=下一个 / 左(-1)=上一个
            prev_idx = self._primary_index
            self._primary_index = (self._primary_index + step) % len(self._config_primaries)
            if self._primary_index != prev_idx:
                self._option_index = 0
                primary = self._config_primaries[self._primary_index]
                self._emit_intent(self._config_intent(primary, primary))

        # H 滚轮拨动 -> 当前一级的二级/动作选项步进。
        primary = self._config_primaries[self._primary_index]
        if primary == "action":
            options = self._action_options
        else:
            options = self._config_menu_options.get(primary, [])
        if self._prev_h_deflect != 0 and sig.h_deflect == 0 and options:
            step = self._prev_h_deflect
            self._option_index = (self._option_index + step) % len(options)
            selected = options[self._option_index]
            self._emit_intent(self._config_intent(primary, selected))
            # speed 菜单二级切换时同步本地速度模式，运动缩放立即生效。
            if primary == "speed" and selected in ("std", "high"):
                self._speed_mode = selected

        self._prev_g_deflect = sig.g_deflect
        self._prev_h_deflect = sig.h_deflect

    def _in_action_menu(self) -> bool:
        return self._config_primaries and self._config_primaries[self._primary_index] == "action"

    def _update_action_longpress(self, sig: G12Input, now: float) -> None:
        """长按确认键触发动作：仅在 action 菜单内且选到具体动作时才有效。"""
        key = self._conf.confirm_key
        pressed_here = getattr(sig, key.lower())

        if not self._in_action_menu():
            self._press_start_sec = None
            self._longpress_handled = False
            return

        selected = self._current_action()
        if selected is None:
            self._press_start_sec = None
            self._longpress_handled = False
            return

        if pressed_here:
            if self._press_start_sec is None:
                # 按下起始（长按起点，不计入连按序列）。
                self._press_start_sec = now
                self._longpress_handled = False
            elif (not self._longpress_handled
                    and now - self._press_start_sec >= self._conf.long_press_timeout_sec):
                self._longpress_handled = True
                self._emit_intent(IntentSpec(
                    intent_type="model", command="switch_model", action=selected))
        else:
            self._press_start_sec = None
            self._longpress_handled = False

    def _current_action(self) -> Optional[str]:
        if not self._action_options:
            return None
        idx = self._option_index % len(self._action_options)
        return self._action_options[idx]

    def _update_holds(self, sig: G12Input, now: float) -> None:
        """长按（hold）处理：A+B 长按 = 运动停止/恢复，A 单独长按 = 循环结束。

        - A+B 同时按住满 long_press_timeout_sec：motion_stop stop/resume 循环切换
          （第一次=运动停止 stop，下一次=运动恢复 resume，如此往复）。
        - A 单独按住（B 未按）满 long_press_timeout_sec -> 循环结束 model/finish_loop。
        """
        a_pressed = sig.a
        b_pressed = sig.b

        if a_pressed and b_pressed:
            key = "ab"
        elif a_pressed and not b_pressed:
            key = "a"
        else:
            key = None

        if key is None:
            self._hold_key = None
            self._hold_start_sec = None
            self._hold_triggered = False
            return

        if self._hold_key != key:
            # 长按组合变化 -> 重新计时。
            self._hold_key = key
            self._hold_start_sec = now
            self._hold_triggered = False
            return

        if self._hold_triggered:
            return

        if now - self._hold_start_sec >= self._conf.long_press_timeout_sec:
            self._hold_triggered = True
            if key == "ab":
                # 运动停止/恢复循环切换：stop -> resume -> stop -> ...
                self._motion_stop_index = (self._motion_stop_index + 1) % len(self._MOTION_STOP_SEQUENCE)
                action = self._MOTION_STOP_SEQUENCE[self._motion_stop_index]
                self._emit_intent(IntentSpec(intent_type="model", command="motion_stop", action=action))
            elif key == "a":
                # 循环结束：优雅退出当前循环动作。
                self._emit_intent(IntentSpec(intent_type="model", command="finish_loop"))

    def _update_taps(self, sig: G12Input, now: float) -> None:
        """连续短按 N 次同一键切换状态/授权（按下沿计数，长按不计入连按）。

        映射（tap_count=3 时）：A=ready、B=walk、C=damp、D=reinit。
        急停(motion_stop)由 A+B 长按触发（见 _update_holds）。
        动作菜单内 D 的长按（触发动作）与连按 3 次（reinit）靠按下沿区分：
        长按仅 1 次按下沿、连按为多次按下沿，互不冲突。
        """
        # 按下沿（0->1）才计入连按，避免长按被误判为连按。
        rising: list[str] = []
        if sig.a and not self._prev_a:
            rising.append("A")
        if sig.b and not self._prev_b:
            rising.append("B")
        if sig.c and not self._prev_c:
            rising.append("C")
        if sig.d and not self._prev_d:
            rising.append("D")

        if not rising:
            if self._tap_key is not None and now > self._tap_window_until:
                self._reset_controller_state()
            return

        # 多键同时按下沿时取第一个（与旧语义一致：不特别处理）。
        key = rising[0]

        if self._tap_key is None:
            self._tap_key = key
            self._tap_count_now = 1
            self._tap_window_until = now + self._conf.triple_timeout_sec
        elif key == self._tap_key and now <= self._tap_window_until:
            self._tap_count_now += 1
            self._tap_window_until = now + self._conf.triple_timeout_sec
        else:
            self._tap_key = key
            self._tap_count_now = 1
            self._tap_window_until = now + self._conf.triple_timeout_sec

        # 达到 N 次 -> 触发并清零。
        if self._tap_count_now >= self._conf.tap_count:
            self._tap_key = None
            self._tap_count_now = 0
            self._fire_state_tap(key)

    def _fire_state_tap(self, key: str) -> None:
        # 状态切换映射：A=ready、B=walk、C=damp、D=reinit（急停 motion_stop 由 A+B 长按触发）。
        mapping = {"A": "ready", "B": "walk", "C": "damp", "D": "reinit"}
        command = mapping.get(key)
        if command is None:
            return
        if command == "damp":
            # 阻尼最高优先：立即停车。
            self._motion_enabled = False
            self._motion_center_required = False
            self._latest_motion = None
            self._emit_intent(IntentSpec(intent_type="status", command="damp"))
            return
        if command == "walk":
            self._motion_enabled = True
            self._motion_center_required = True
            self._emit_intent(IntentSpec(intent_type="status", command="walk"))
            return
        if command == "ready":
            self._motion_enabled = False
            self._motion_center_required = False
            self._latest_motion = None
            self._emit_intent(IntentSpec(intent_type="status", command="ready"))
            return
        if command == "reinit":
            self._motion_enabled = False
            self._motion_center_required = False
            self._latest_motion = None
            self._emit_intent(IntentSpec(intent_type="status", command="reinit"))
            return

    @staticmethod
    def _config_intent(primary: str, option: str) -> IntentSpec:
        return IntentSpec(intent_type="config", command=primary, action=option)

    def _axes_centered(self, axes: list[float]) -> bool:
        return all(abs(a) < self._conf.deadzone for a in axes)

    def _build_motion(self, axes: list[float]) -> IntentSpec:
        # 高速模式下前进、横移均使用高速上限（与 teleop_joy 语义一致）。
        if self._speed_mode == "high":
            max_x = self._conf.max_linear_x_high_mps
            max_y = self._conf.max_linear_y_high_mps
        else:
            max_x = self._conf.max_linear_x_mps
            max_y = self._conf.max_linear_y_mps
        return IntentSpec(
            intent_type="motion",
            command="walk",
            linear_x_mps=round(axes[1] * max_x, 6),  # 横移 CH3
            linear_y_mps=round(axes[0] * max_y, 6),  # 前进 CH4
            angular_z_radps=round(axes[2] * self._conf.max_angular_z_radps, 6),  # 转身 CH1
        )

    def _emit_intent(self, spec: IntentSpec) -> None:
        intent = self._builder.build(spec)
        intent.header.stamp = self.get_clock().now().to_msg()
        self._intent_pub.publish(intent)

    def _now(self) -> float:
        return time.monotonic()

    # ---------- 定时发布 ----------

    def _handle_timer(self) -> None:
        now = time.monotonic()
        # 失联停车：收到过数据后超时且系统处于运行态 -> 发布全零停止。
        if (self._last_read_time > 0
                and now - self._last_read_time > self._timeout_sec
                and self._system_state == RobotState.STATE_RL_ACTIVE):
            if not self._published_timeout_stop:
                self._published_timeout_stop = True
                self._emit_intent(IntentSpec(intent_type="motion", command="walk"))
            return

        if self._latest_motion is not None:
            spec = self._latest_motion
        else:
            return
        self._emit_intent(spec)

    def destroy_node(self) -> bool:
        self._stop.set()
        if self._serial is not None and self._serial.is_open:
            try:
                self._serial.close()
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = TeleopSbusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
