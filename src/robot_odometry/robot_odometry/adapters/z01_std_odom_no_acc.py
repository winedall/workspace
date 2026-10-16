"""z01_std_odom_no_acc 的输入适配器。

88 维，不含基座加速度。输入布局由基类实现，本文件只声明本模型是否需要
加速度——与 robot_rl_manager 的「一个模型一个 adapter」保持同一形式，
读 adapter 文件即可知道该模型的输入构成。
"""

from __future__ import annotations

from .prev_joint_pos import PrevJointPosOdometryAdapter


class Z01StdOdomNoAccOdometryAdapter(PrevJointPosOdometryAdapter):
    """schema v3，88 维，不含 base_acc。"""

    USE_BASE_ACCELERATION = False

    def __init__(self) -> None:
        super().__init__(use_base_acceleration=self.USE_BASE_ACCELERATION)
