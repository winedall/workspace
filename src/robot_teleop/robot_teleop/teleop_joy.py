"""将 Logitech F710 的 sensor_msgs/Joy 归一化为 RobotIntent 中间描述。"""

from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import Optional

from sensor_msgs.msg import Joy

from robot_teleop.intent_builder import IntentSpec
from robot_teleop.model_button_map import ModelButtonMap, create_model_button_map


@dataclass(frozen=True)
class JoyTeleopConfig:
    """Joy 解析参数、速度上限和阈值。"""
    # 标准速度左摇杆前后最大线速度。
    max_linear_x_mps: float = 1.0
    # 标准速度左摇杆左右最大横移速度。
    max_linear_y_mps: float = 0.5
    # 高速模式左摇杆前后最大线速度。
    max_linear_x_high_mps: float = 3.0
    # 高速模式左摇杆左右最大横移速度。
    max_linear_y_high_mps: float = 1.0
    # 右摇杆左右最大偏航角速度。
    max_angular_z_radps: float = 1.5
    # 摇杆死区，小于该绝对值的输入按 0 处理。
    deadzone: float = 0.08
    # 方向键阈值，用于判断上下左右方向。
    dpad_threshold: float = 0.5


@dataclass
class _MenuState:
    """LT/RT 三连击配置菜单的内部状态。"""
    # 一级菜单索引：getup / speed / nav。
    primary_index: int = 0
    # 每个一级菜单下当前选中的二级选项索引。
    option_indices: dict[str, int] = field(
        default_factory=lambda: {"getup": 0, "speed": 0, "nav": 0, "mode": 0}
    )
    # LT 按住 + RT 上升沿计数，用于切换一级菜单。
    lt_rt_count: int = 0
    # RT 按住 + LT 上升沿计数，用于切换当前菜单选项并发布 config。
    rt_lt_count: int = 0


@dataclass
class _ResetFaultHoldState:
    """Start 长按触发 reset_fault 的内部状态。"""
    # 本次 Start 单独按下的起始时间，单位秒。
    hold_start_sec: Optional[float] = None
    # 同一次长按是否已经触发过 reset_fault，避免持续按住时重复发布。
    triggered_for_hold: bool = False


@dataclass
class _FinishLoopHoldState:
    """Back 长按触发 finish_loop 的内部状态。"""
    # 本次 Back 单独按下的起始时间，单位秒。
    hold_start_sec: Optional[float] = None
    # 同一次长按是否已经触发过 finish_loop，避免持续按住时重复发布。
    triggered_for_hold: bool = False


@dataclass
class _MotionStopState:
    """Back+Start 长按触发 motion_stop 停/恢复的内部状态。"""
    # 本次 Back+Start 同时按下的起始时间，单位秒。
    hold_start_sec: Optional[float] = None
    # 同一次长按是否已经触发过 motion_stop，避免持续按住时重复发布。
    triggered_for_hold: bool = False
    # 当前循环到的动作索引；初始 -1 表示第一次触发切到 stop。
    action_index: int = -1


class TeleopJoyMapper:
    """根据 docs/RobotInter对应关系.md 将 F710 Joy 帧映射为 IntentSpec。"""

    _PRIMARY_MENUS = ("getup", "speed", "nav", "mode")
    _MOTION_STOP_SEQUENCE = ("stop", "resume")
    _MOTION_STOP_HOLD_DURATION_SEC = 3.0
    _RESET_FAULT_HOLD_DURATION_SEC = 1.0
    _EXIT_LOOP_HOLD_DURATION_SEC = 1.0
    # F710 校准失败检测阈值：三个轴同时超过此值则判定为异常。
    _CALIB_FAIL_THRESHOLD = 0.98
    _MENU_OPTIONS = {
        "getup": ("stand", "lie", "face"),
        "speed": ("std", "high"),
        # "nav": ("manual", "auto", "follow"),
        "nav": ("manual", "auto"),
        "mode": ("rl", "mix", "spec", "app"),
    }

    def __init__(self, config: JoyTeleopConfig, robot_type: str = "humanoid") -> None:
        self._config = config
        self._model_map: ModelButtonMap = create_model_button_map(robot_type)
        # 保存上一帧 buttons，用于检测 LT/RT 三连击的上升沿。
        self._previous_buttons = [0] * 13
        # status/model 等组合键事件的锁存 key；同一组合键持续按住时不重复触发。
        self._active_event_key: Optional[str] = None
        # 只有收到 status + walk 后才允许输出 motion 类意图。
        self._motion_enabled = False
        self._menu = _MenuState()
        # 速度模式：std / high，由 config/speed 意图切换，影响 _map_motion_axes 的缩放上限。
        self._speed_mode = "std"
        self._mode = "rl"
        self._motion_stop = _MotionStopState()
        self._reset_fault = _ResetFaultHoldState()
        self._exit_loop = _FinishLoopHoldState()
        # F710 校准异常锁存：一旦检测到三个轴同时卡死最大值，锁定为零输出，
        # 直到外部调用 clear_calib_error() 恢复。
        self._calib_error = False
        # walk 授权后必须先观察到运动轴回中一次，避免 F710 休眠唤醒缓存值
        # 被当作真实运动输入持续发布。
        self._motion_center_required = False

    def map_joy(self, msg: Joy) -> Optional[IntentSpec]:
        """将一帧 Joy 映射为 IntentSpec；菜单切换中间帧返回 None。"""
        buttons = list(msg.buttons)
        axes = list(msg.axes)

        # Back+Start 长按属于运动停止/恢复切换，优先级高于普通状态、模型和运动输入。
        result = self._map_motion_stop_combo(msg, buttons)
        if result is not None:
            self._previous_buttons = buttons
            return result
        result = self._map_reset_fault_combo(msg, buttons)
        if result is not None:
            self._previous_buttons = buttons
            return result
        result = self._map_exit_loop_combo(msg, buttons)
        if result is not None:
            self._previous_buttons = buttons
            return result
        if self._button(buttons, 8) or self._button(buttons, 9):
            self._previous_buttons = buttons
            return None

        # 按文档要求的优先级解析，避免 LB+RB+DPad 被误判为 LB+DPad 或 RB+DPad。
        result = (
            self._map_status_combo(buttons)
            or self._map_double_button_combo(buttons, axes)
            or self._map_single_button_combo(buttons, axes)
        )
        if result is not None:
            event_key = self._event_key(result)
            if event_key == self._active_event_key:
                self._previous_buttons = buttons
                return None
            self._update_motion_authorization(result)
            self._active_event_key = event_key
            self._previous_buttons = buttons
            return result

        self._active_event_key = None
        if result is None:
            result = self._map_config_combo(buttons)
            # LT/RT 菜单操作期间不落到 motion stop，避免菜单按键生成误停止意图。
            if result is None and (self._button(buttons, 6) or self._button(buttons, 7)):
                self._previous_buttons = buttons
                return None
        if result is None:
            result = self._map_motion_axes(axes) if self._motion_enabled else None
        self._previous_buttons = buttons
        return result

    def _map_status_combo(self, buttons: list[int]) -> Optional[IntentSpec]:
        """解析 LB+RB+A/B/X/Y 状态切换组合键。"""
        if self._button(buttons, 4) and self._button(buttons, 5):
            if self._button(buttons, 1):
                return IntentSpec(intent_type="status", command="ready")
            if self._button(buttons, 2):
                return IntentSpec(intent_type="status", command="walk")
            if self._button(buttons, 3):
                return IntentSpec(intent_type="status", command="damp")
            if self._button(buttons, 0):
                return IntentSpec(intent_type="status", command="reinit")
        return None

    def _map_motion_stop_combo(self, msg: Joy, buttons: list[int]) -> Optional[IntentSpec]:
        """解析 Back+Start 长按 motion_stop stop/resume 组合键。

        Back 和 Start 同时按住满 3 秒后只发布一次 model/motion_stop；
        松开任一按键后清空本次长按状态，下一次长按再继续 stop/resume 循环。
        """
        both_pressed = self._button(buttons, 8) and self._button(buttons, 9)
        if not both_pressed:
            self._motion_stop.hold_start_sec = None
            self._motion_stop.triggered_for_hold = False
            return None

        now_sec = self._joy_time_sec(msg)
        if self._motion_stop.hold_start_sec is None:
            self._motion_stop.hold_start_sec = now_sec
            self._motion_stop.triggered_for_hold = False
            return None

        if now_sec < self._motion_stop.hold_start_sec:
            self._motion_stop.hold_start_sec = now_sec
            self._motion_stop.triggered_for_hold = False
            return None

        elapsed_sec = now_sec - self._motion_stop.hold_start_sec
        if (
            not self._motion_stop.triggered_for_hold
            and elapsed_sec >= self._MOTION_STOP_HOLD_DURATION_SEC
        ):
            self._motion_stop.triggered_for_hold = True
            self._motion_stop.action_index = (
                self._motion_stop.action_index + 1
            ) % len(self._MOTION_STOP_SEQUENCE)
            return IntentSpec(
                intent_type="model",
                command="motion_stop",
                action=self._MOTION_STOP_SEQUENCE[self._motion_stop.action_index],
            )
        return None

    def _map_reset_fault_combo(self, msg: Joy, buttons: list[int]) -> Optional[IntentSpec]:
        """解析 Start 单独长按故障复位组合键。"""
        start_pressed = self._button(buttons, 9)
        back_pressed = self._button(buttons, 8)
        if not start_pressed or back_pressed:
            self._reset_fault.hold_start_sec = None
            self._reset_fault.triggered_for_hold = False
            return None

        now_sec = self._joy_time_sec(msg)
        if self._reset_fault.hold_start_sec is None:
            self._reset_fault.hold_start_sec = now_sec
            self._reset_fault.triggered_for_hold = False
            return None

        if now_sec < self._reset_fault.hold_start_sec:
            self._reset_fault.hold_start_sec = now_sec
            self._reset_fault.triggered_for_hold = False
            return None

        elapsed_sec = now_sec - self._reset_fault.hold_start_sec
        if (
            not self._reset_fault.triggered_for_hold
            and elapsed_sec >= self._RESET_FAULT_HOLD_DURATION_SEC
        ):
            self._reset_fault.triggered_for_hold = True
            return IntentSpec(intent_type="reset", command="reset_fault")
        return None

    def _map_exit_loop_combo(self, msg: Joy, buttons: list[int]) -> Optional[IntentSpec]:
        """解析 Back 单独长按退出循环动作组合键。

        Back 单独按住满 3 秒后只发布一次 model/finish_loop；
        松开 Back 后清空本次长按状态，下一次长按重新计时。
        """
        back_pressed = self._button(buttons, 8)
        start_pressed = self._button(buttons, 9)
        if not back_pressed or start_pressed:
            self._exit_loop.hold_start_sec = None
            self._exit_loop.triggered_for_hold = False
            return None

        now_sec = self._joy_time_sec(msg)
        if self._exit_loop.hold_start_sec is None:
            self._exit_loop.hold_start_sec = now_sec
            self._exit_loop.triggered_for_hold = False
            return None

        if now_sec < self._exit_loop.hold_start_sec:
            self._exit_loop.hold_start_sec = now_sec
            self._exit_loop.triggered_for_hold = False
            return None

        elapsed_sec = now_sec - self._exit_loop.hold_start_sec
        if (
            not self._exit_loop.triggered_for_hold
            and elapsed_sec >= self._EXIT_LOOP_HOLD_DURATION_SEC
        ):
            self._exit_loop.triggered_for_hold = True
            return IntentSpec(intent_type="model", command="finish_loop")
        return None

    def _map_double_button_combo(self, buttons: list[int], axes: list[float]) -> Optional[IntentSpec]:
        """解析 LB+RB+DPad 双键组合 model 意图，委托给策略，spec 模式切换特技映射。"""
        if not (self._button(buttons, 4) and self._button(buttons, 5)):
            return None
        return self._model_map.map_double_button_combo(
            self._axis(axes, 4), self._axis(axes, 5), self._config.dpad_threshold,
            mode=self._mode,
        )

    def _map_single_button_combo(self, buttons: list[int], axes: list[float]) -> Optional[IntentSpec]:
        """解析 LB/RB+DPad 单键组合 model 意图，委托给策略，spec 模式切换特技映射。"""
        return self._model_map.map_single_button_combo(
            self._button(buttons, 4), self._button(buttons, 5),
            self._axis(axes, 4), self._axis(axes, 5), self._config.dpad_threshold,
            mode=self._mode,
        )

    def _map_config_combo(self, buttons: list[int]) -> Optional[IntentSpec]:
        """解析 LT/RT 三连击配置菜单。

        LT 按住 + RT 三次上升沿切换一级菜单，并发布当前菜单提示 RobotIntent。
        RT 按住 + LT 三次上升沿切换二级选项，并发布 config 类 RobotIntent。
        """
        lt_pressed = self._button(buttons, 6)
        rt_pressed = self._button(buttons, 7)
        lt_rising = self._rising_edge(buttons, 6)
        rt_rising = self._rising_edge(buttons, 7)

        if lt_pressed and rt_rising:
            # 一级菜单切换：getup -> speed -> nav -> mode -> getup。
            self._menu.lt_rt_count += 1
            self._menu.rt_lt_count = 0
            if self._menu.lt_rt_count >= 3:
                self._menu.lt_rt_count = 0
                self._menu.primary_index = (self._menu.primary_index + 1) % len(self._PRIMARY_MENUS)
                menu = self._PRIMARY_MENUS[self._menu.primary_index]
                return IntentSpec(
                    intent_type="config",
                    command=menu,
                    action=menu,
                )
            return None

        if rt_pressed and lt_rising:
            # 二级选项切换：只有选项真正变化时才发布 config 意图。
            self._menu.rt_lt_count += 1
            self._menu.lt_rt_count = 0
            if self._menu.rt_lt_count >= 3:
                self._menu.rt_lt_count = 0
                menu = self._PRIMARY_MENUS[self._menu.primary_index]
                options = self._MENU_OPTIONS[menu]
                self._menu.option_indices[menu] = (self._menu.option_indices[menu] + 1) % len(options)
                action = options[self._menu.option_indices[menu]]
                if menu == "speed":
                    self._speed_mode = action
                elif menu == "mode":
                    self._mode = action
                return IntentSpec(
                    intent_type="config",
                    command=menu,
                    action=action,
                )
            return None

        if not lt_pressed and not rt_pressed:
            # 两个扳机都释放后清空三连击计数，下一次重新开始。
            self._menu.lt_rt_count = 0
            self._menu.rt_lt_count = 0
        if lt_pressed or rt_pressed:
            return None
        return None

    def _map_motion_axes(self, axes: list[float]) -> IntentSpec:
        """解析摇杆连续运动输入。"""
        if self._motion_center_required:
            if self._motion_axes_centered(axes):
                self._motion_center_required = False
            else:
                return self._zero_motion_spec()

        # F710 固件 auto-calibration 失败时，三个运动轴同时卡在 ±1.0 附近。
        # 检测到后锁存校准错误，输出零速度；只有当用户主动将三个拨杆都归零
        # （三个轴值同时进入死区）后才自动退出锁存，恢复正常控制。
        if (abs(self._axis(axes, 0)) > self._CALIB_FAIL_THRESHOLD
                and abs(self._axis(axes, 1)) > self._CALIB_FAIL_THRESHOLD
                and abs(self._axis(axes, 2)) > self._CALIB_FAIL_THRESHOLD):
            self._calib_error = True
        if self._calib_error:
            if self._motion_axes_centered(axes):
                self._calib_error = False
            else:
                return self._zero_motion_spec()
        if self._speed_mode == "high":
            max_x = self._config.max_linear_x_high_mps
            max_y = self._config.max_linear_y_high_mps
        else:
            max_x = self._config.max_linear_x_mps
            max_y = self._config.max_linear_y_mps
        linear_y = self._scaled_axis(axes, 0, max_y)
        linear_x = self._scaled_axis(axes, 1, max_x)
        angular_z = self._scaled_axis(axes, 2, self._config.max_angular_z_radps)

        return IntentSpec(
            intent_type="motion",
            command="walk",
            linear_x_mps=linear_x,
            linear_y_mps=linear_y,
            angular_z_radps=angular_z,
        )

    def _motion_axes_centered(self, axes: list[float]) -> bool:
        """三个运动轴均进入死区时视为回中。"""
        return (
            abs(self._axis(axes, 0)) < self._config.deadzone
            and abs(self._axis(axes, 1)) < self._config.deadzone
            and abs(self._axis(axes, 2)) < self._config.deadzone
        )

    @staticmethod
    def _zero_motion_spec() -> IntentSpec:
        """生成零速度 motion，用于覆盖上层连续 motion 缓存。"""
        return IntentSpec(
            intent_type="motion",
            command="walk",
            linear_x_mps=0.0,
            linear_y_mps=0.0,
            angular_z_radps=0.0,
        )

    def _apply_deadzone(self, value: float) -> float:
        """应用摇杆死区。"""
        return 0.0 if abs(value) < self._config.deadzone else float(value)

    def _scaled_axis(self, axes: list[float], index: int, scale: float) -> float:
        """读取轴值、应用死区并按最大速度缩放。"""
        return round(self._apply_deadzone(self._axis(axes, index)) * float(scale), 6)

    @staticmethod
    def _axis(axes: list[float], index: int) -> float:
        """安全读取 axes[index]，缺失时按 0 处理。"""
        return float(axes[index]) if index < len(axes) else 0.0

    @staticmethod
    def _button(buttons: list[int], index: int) -> bool:
        """安全读取 buttons[index]，1 表示按下。"""
        return index < len(buttons) and buttons[index] == 1

    def _rising_edge(self, buttons: list[int], index: int) -> bool:
        """检测某个按键从释放到按下的上升沿。"""
        current = 1 if self._button(buttons, index) else 0
        previous = self._previous_buttons[index] if index < len(self._previous_buttons) else 0
        return current == 1 and previous == 0

    @staticmethod
    def _joy_time_sec(msg: Joy) -> float:
        """读取 Joy 时间戳；没有有效时间戳时用单调时钟兜底。"""
        stamp = msg.header.stamp
        if stamp.sec != 0 or stamp.nanosec != 0:
            return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0
        return time.monotonic()

    @staticmethod
    def _event_key(spec: IntentSpec) -> str:
        """生成事件类组合键的锁存 key。"""
        return f"{spec.intent_type}:{spec.command}:{spec.action}"

    def _update_motion_authorization(self, spec: IntentSpec) -> None:
        """根据 status 意图更新 motion 输出授权。"""
        if spec.intent_type != "status":
            return
        if spec.command == "walk":
            self._motion_enabled = True
            self._motion_center_required = True
        elif spec.command in {"ready", "damp", "reinit"}:
            self._motion_enabled = False
            self._motion_center_required = False
