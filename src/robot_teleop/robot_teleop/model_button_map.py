"""机器人类型相关的模型切换按键映射策略。"""

from __future__ import annotations

from abc import ABC
from enum import Enum
from typing import Optional

from robot_teleop.intent_builder import IntentSpec

_MODEL_MAP_REGISTRY: dict[str, type["ModelButtonMap"]] = {}


def register_model_map(robot_type: str) -> callable:
    def decorator(cls: type[ModelButtonMap]) -> type[ModelButtonMap]:
        _MODEL_MAP_REGISTRY[robot_type] = cls
        return cls
    return decorator


def create_model_button_map(robot_type: str) -> "ModelButtonMap":
    cls = _MODEL_MAP_REGISTRY.get(robot_type)
    if cls is None:
        raise ValueError(f"unsupported robot_type: {robot_type}")
    return cls()


class DPad(Enum):
    """方向键四向枚举。"""
    LEFT = "left"
    UP = "up"
    RIGHT = "right"
    DOWN = "down"


def _dpad_direction(dpad_x: float, dpad_y: float, threshold: float) -> Optional[DPad]:
    """把方向键轴值解码为四向枚举；无有效方向输入返回 None。"""
    if dpad_x > threshold and dpad_y == 0:
        return DPad.LEFT
    if dpad_y > threshold and dpad_x == 0:
        return DPad.UP
    if dpad_x < -threshold and dpad_y == 0:
        return DPad.RIGHT
    if dpad_y < -threshold and dpad_x == 0:
        return DPad.DOWN
    return None


class ModelButtonMap(ABC):
    """遥控器模型切换按键映射策略基类。

    子类通过 _combo_actions 提供 {mode: {(lb, rb, DPad): action}} 动作表，
    基类统一负责方向解码、查表与 IntentSpec 构造；新增按键组合只需改表。
    """
    _combo_actions: dict[str, dict[tuple[bool, bool, DPad], str]] = {}

    def map_double_button_combo(
        self, dpad_x: float, dpad_y: float, threshold: float, mode: str = "rl",
    ) -> Optional[IntentSpec]:
        return self._map_combo(True, True, dpad_x, dpad_y, threshold, mode)

    def map_single_button_combo(
        self, lb_pressed: bool, rb_pressed: bool,
        dpad_x: float, dpad_y: float, threshold: float, mode: str = "rl",
    ) -> Optional[IntentSpec]:
        return self._map_combo(lb_pressed, rb_pressed, dpad_x, dpad_y, threshold, mode)

    def _map_combo(
        self, lb_pressed: bool, rb_pressed: bool,
        dpad_x: float, dpad_y: float, threshold: float, mode: str,
    ) -> Optional[IntentSpec]:
        direction = _dpad_direction(dpad_x, dpad_y, threshold)
        if direction is None:
            return None
        action = self._combo_actions.get(mode, {}).get(
            (lb_pressed, rb_pressed, direction)
        )
        if action is None:
            return None
        return IntentSpec(intent_type="model", command="switch_model", action=action)


@register_model_map("humanoid")
class HumanoidModelButtonMap(ModelButtonMap):
    """人形机器人（t4_ 前缀）模型切换映射。"""
    _combo_actions: dict[str, dict[tuple[bool, bool, DPad], str]] = {
        "rl": {
            # LB + 左/上/右/下
            (True, False, DPad.LEFT): "t4_rhello",
            (True, False, DPad.UP): "t4_clap",
            (True, False, DPad.RIGHT): "t4_handshake",
            (True, False, DPad.DOWN): "t4_standby",
            # RB + 左/上/右/下
            (False, True, DPad.LEFT): "t4_heart",
            (False, True, DPad.UP): "t4_salute",
            (False, True, DPad.RIGHT): "t4_hello",
            (False, True, DPad.DOWN): "t4_high_five",
            # LB + RB + 左/上/右/下
            (True, True, DPad.LEFT): "t4_taichi",
            (True, True, DPad.UP): "t4_taichi_w",
            (True, True, DPad.RIGHT): "t4_mj_dance",
            (True, True, DPad.DOWN): "t4_xqxa123_dance",
        },
        "spec": {
            # LB + 左/上/右/下
            (True, False, DPad.LEFT): "t4_none",
            (True, False, DPad.UP): "t4_none",
            (True, False, DPad.RIGHT): "t4_none",
            (True, False, DPad.DOWN): "t4_none",
            # RB + 左/上/右/下
            (False, True, DPad.LEFT): "t4_none",
            (False, True, DPad.UP): "t4_none",
            (False, True, DPad.RIGHT): "t4_none",
            (False, True, DPad.DOWN): "t4_none",
            # LB + RB + 左/上/右/下
            (True, True, DPad.LEFT): "z01_side_flip",
            (True, True, DPad.UP): "z01_front_flip",
            (True, True, DPad.RIGHT): "z01_jd_jump",
            (True, True, DPad.DOWN): "z01_pa_front_flip",
        },
    }


@register_model_map("dog")
class DogModelButtonMap(ModelButtonMap):
    """四足机器人（d4_ 前缀）模型切换映射。"""
    _combo_actions: dict[str, dict[tuple[bool, bool, DPad], str]] = {
        "rl": {
            # LB + 左/上/右/下
            (True, False, DPad.LEFT): "d4_none",
            (True, False, DPad.UP): "d4_none",
            (True, False, DPad.RIGHT): "d4_none",
            (True, False, DPad.DOWN): "d4_none",
            # RB + 左/上/右/下
            (False, True, DPad.LEFT): "d4_none",
            (False, True, DPad.UP): "d4_none",
            (False, True, DPad.RIGHT): "d4_none",
            (False, True, DPad.DOWN): "d4_none",
            # LB + RB + 左/上/右/下
            (True, True, DPad.LEFT): "d4_none",
            (True, True, DPad.UP): "d4_none",
            (True, True, DPad.RIGHT): "d4_none",
            (True, True, DPad.DOWN): "d4_none",
        },
        "spec": {
            # LB + 左/上/右/下
            (True, False, DPad.LEFT): "d4_none",
            (True, False, DPad.UP): "d4_none",
            (True, False, DPad.RIGHT): "d4_none",
            (True, False, DPad.DOWN): "d4_none",
            # RB + 左/上/右/下
            (False, True, DPad.LEFT): "d4_none",
            (False, True, DPad.UP): "d4_none",
            (False, True, DPad.RIGHT): "d4_none",
            (False, True, DPad.DOWN): "d4_none",
            # LB + RB + 左/上/右/下
            (True, True, DPad.LEFT): "d4_none",
            (True, True, DPad.UP): "d4_none",
            (True, True, DPad.RIGHT): "d4_none",
            (True, True, DPad.DOWN): "d4_none",
        },
    }
