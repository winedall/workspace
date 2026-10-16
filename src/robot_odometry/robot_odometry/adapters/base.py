"""里程计 adapter 基类。

adapter 负责「机器人语义 → 模型输入」这一段：声明自己需要哪些输入、
维护帧间状态、拼出模型要的观测向量。节点只负责取数据、调 adapter、
跑模型、发消息，因此新增输入变体时只需要加一个 adapter 文件 + 一份配置。

与 robot_rl_manager 的 PolicyAdapter 同构，只是输入侧从 policy 上下文
换成了里程计上下文。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np


@dataclass
class OdometryContext:
    """每次推理时传给 adapter 的输入快照。

    与 PolicyContext 一样按需提供：adapter 通过 ``required_inputs()`` 声明
    自己用哪些字段，节点据此做缺输入判断。
    """

    #: {关节名: (位置 rad, 速度 rad/s)}，来自硬件反馈。
    joint_states: Optional[Dict[str, Tuple[float, float]]] = None

    #: body 系角速度 [wx, wy, wz]，单位 rad/s。
    base_ang_vel: Optional[Sequence[float]] = None

    #: 世界系姿态四元数 [w, x, y, z]。
    base_quat: Optional[Sequence[float]] = None

    #: body 系线加速度 [ax, ay, az]，单位 m/s^2。
    base_acc: Optional[Sequence[float]] = None

    #: 当前推理时刻（ROS 秒）。
    now_sec: float = 0.0

    def has_input(self, name: str) -> bool:
        """判断某个语义输入是否可用。

        与 robot_rl_manager 的 PolicyContext.has_input 同一形式：语义名显式
        映射到实际字段，未知名字再回退到同名字段。

        注意 ``imu`` 这个语义名并不直接对应任何字段，它要求姿态与角速度
        同时就绪——只按名字回退到同名字段会永远返回 False，导致每一帧都被
        判成缺输入。
        """
        if name == "joint_states":
            return self.joint_states is not None
        if name == "imu":
            return self.base_quat is not None and self.base_ang_vel is not None
        if name == "imu_acc":
            return self.base_acc is not None
        return getattr(self, name, None) is not None


class BaseOdometryAdapter(ABC):
    """所有里程计 adapter 的基类。"""

    #: 模型观测维度，由子类按配置设置。
    obs_dim: int = 0

    @abstractmethod
    def required_inputs(self) -> set[str]:
        """返回本 adapter 需要的上下文输入名集合。"""

    @abstractmethod
    def reset(self) -> None:
        """清空帧间状态（进入/退出行走、控制流中断时调用）。"""

    @abstractmethod
    def build_observation(self, context: OdometryContext) -> np.ndarray:
        """把上下文拼成一帧模型观测 ``[obs_dim]``。

        实现应当在返回前把本帧状态写回内部帧间缓存（例如上一帧关节位置）。
        """

    def missing_inputs(self, context: OdometryContext) -> set[str]:
        """返回声明需要但上下文缺失的输入名。"""
        return {
            name
            for name in self.required_inputs()
            if not context.has_input(name)
        }

    @staticmethod
    def _as_vector(value: Sequence[float], size: int, name: str) -> np.ndarray:
        """把序列转成定长 float32 向量，长度不符时报错。"""
        array = np.asarray(value, dtype=np.float32).reshape(-1)
        if array.shape != (size,):
            raise ValueError(f"{name} 维度应为 [{size}]，实际 {array.shape}")
        return array


__all__ = ["BaseOdometryAdapter", "OdometryContext"]
