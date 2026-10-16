"""纯几何工具函数（不依赖 ROS，便于单测）。"""

from __future__ import annotations

import math
from typing import Sequence, Tuple


def quaternion_to_yaw(quat: Sequence[float]) -> float:
    """从 (w, x, y, z) 四元数提取偏航角，单位 rad。

    只适用于标准 x-前 / y-左 / z-上右手系的姿态表示。
    """
    w, x, y, z = (float(value) for value in quat)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle: float) -> float:
    """把角度归一化到 (-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """由偏航角生成 (w, x, y, z) 四元数（x=y=0）。"""
    half = yaw / 2.0
    return (math.cos(half), 0.0, 0.0, math.sin(half))
