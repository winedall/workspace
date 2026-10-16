"""z01_std_odom_with_acc 的输入适配器。

91 维，含基座加速度（base_acc 插在 base_ang_vel 与 prev_joint_pos 之间）。
输入布局由基类实现，本文件只声明本模型是否需要加速度。
"""

from __future__ import annotations

from .prev_joint_pos import PrevJointPosOdometryAdapter


class Z01StdOdomWithAccOdometryAdapter(PrevJointPosOdometryAdapter):
    """schema v3，91 维，含 base_acc。"""

    USE_BASE_ACCELERATION = True

    def __init__(self) -> None:
        super().__init__(use_base_acceleration=self.USE_BASE_ACCELERATION)
