"""当前 schema（v3）的里程计 adapter：joint_pos + joint_vel + 姿态 + [加速度] + 上一帧关节位置。

对应当前一批模型（``no_action*``）。``base_acc`` 由模型配置的
``use_base_acceleration`` 决定是否进入观测：带加速度的 91 维，不带的 88 维。

关键语义 —— ``joint_pos_previous``：
    第 t 帧的该字段是第 t-1 帧的 ``joint_pos``。首帧没有上一帧，按训练侧的
    约定用当前帧自身填充（见 ``Trajectory.build_full_observation``）。
    因此本 adapter 必须持有帧间状态，``reset()`` 时清空，下一次
    ``build_observation`` 会自动退回「首帧自引用」。

action 完全不参与本 schema：既不是观测的一部分，也不再有任何话题为它发布。
节点靠自己的 50Hz 定时器驱动推理。
"""

from __future__ import annotations

from typing import Optional, Set

import numpy as np

from ..joint_order import (
    build_storage_observation,
    joints_by_name,
    observation_spec,
    select_observation,
)
from .base import BaseOdometryAdapter, OdometryContext


class PrevJointPosOdometryAdapter(BaseOdometryAdapter):
    """v3 schema 的观测构造器。"""

    def __init__(self, use_base_acceleration: bool = False) -> None:
        self._use_base_acceleration = bool(use_base_acceleration)
        self.obs_dim, self.obs_layout, _ = observation_spec(
            self._use_base_acceleration
        )

        self._previous_joint_pos: Optional[np.ndarray] = None

    @property
    def use_base_acceleration(self) -> bool:
        """当前模型是否需要基座加速度。"""
        return self._use_base_acceleration

    def required_inputs(self) -> Set[str]:
        """v3 schema 需要关节反馈与 IMU；带加速度版本额外需要 IMU 加速度。"""
        inputs = {"joint_states", "imu"}
        if self._use_base_acceleration:
            inputs.add("imu_acc")
        return inputs

    def reset(self) -> None:
        """清空帧间状态，下一次构造观测时 ``joint_pos_previous`` 自引用。"""
        self._previous_joint_pos = None

    def build_observation(self, context: OdometryContext) -> np.ndarray:
        """拼出 88 或 91 维观测，并把本帧关节位置记为下一帧的 previous。"""
        if context.joint_states is None:
            raise ValueError("里程计上下文缺少关节状态")
        if context.base_quat is None or context.base_ang_vel is None:
            raise ValueError("里程计上下文缺少 IMU 姿态或角速度")

        joint_pos, joint_vel = joints_by_name(context.joint_states)

        # 首帧（或刚 reset）没有上一帧关节位置，按训练侧约定用当前帧自身填充。
        if self._previous_joint_pos is None:
            previous_joint_pos = joint_pos.copy()
        else:
            previous_joint_pos = self._previous_joint_pos

        if self._use_base_acceleration:
            if context.base_acc is None:
                raise ValueError("当前模型需要基座加速度，但上下文中没有该输入")
            base_acc = self._as_vector(context.base_acc, 3, "base_acc")
        else:
            # 不参与观测，但仍要凑出存储布局；用完即被列索引丢弃。
            base_acc = np.zeros(3, dtype=np.float32)

        storage = build_storage_observation(
            joint_pos=joint_pos,
            joint_vel=joint_vel,
            base_quat=self._as_vector(context.base_quat, 4, "base_quat"),
            base_ang_vel=self._as_vector(
                context.base_ang_vel, 3, "base_ang_vel"
            ),
            base_acc=base_acc,
            previous_joint_pos=previous_joint_pos,
        )

        observation = select_observation(storage, self._use_base_acceleration)

        # 本帧状态写回，供下一帧作为 previous 使用。
        self._previous_joint_pos = joint_pos.copy()
        return observation
