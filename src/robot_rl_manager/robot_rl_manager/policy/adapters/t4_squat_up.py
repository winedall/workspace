"""SquatUp 策略 adapter — 硬编码四段线性插值，无 ONNX 模型。

参考旧 policy_t4_squat_up.py 的四阶段（q1→q2→q3→q4→q5）插值逻辑。
输出 31 关节（robot_joints.yaml 顺序），头、waist_pitch、waist_roll 保持 profile 默认位置。
完成后 motion_end=True，节点自动切回 walk。
"""

from __future__ import annotations

from typing import Dict, Iterable, List, Optional

from robot_rl_manager.policy.adapter import BasePolicyAdapter
from robot_rl_manager.policy.backend import ModelInput
from robot_rl_manager.policy.context import PolicyContext

# ── 四阶段轨迹关键帧（31 关节，robot_joints.yaml 顺序） ──────────────────────
# 顺序: head(2), L_arm(7), R_arm(7), waist(3), L_leg(6), R_leg(6)
#
# squat_up 从深蹲（q1）经过先下蹲更深蓄力再站起到放松站立（q5）。
# q5 与 squat_down 的 q1 位置一致，完成后 auto-switch 到 walk 顺畅过渡。

_QPOS1 = [
    0.0, 0.0,                                          # head_yaw, head_pitch
    -0.30, 0.20, 0.00, -0.80, 0.00, 0.20, 0.00,       # left_arm
    -0.30, -0.20, 0.00, -0.80, 0.00, 0.20, 0.00,      # right_arm
    0.0, 0.0, 0.0,                                      # waist_pitch, waist_roll, waist_yaw
    -1.60, 0.00, 0.00, 1.80, -0.2, 0.00,               # left_leg
    -1.60, 0.00, 0.00, 1.80, -0.2, 0.00,               # right_leg
]

_QPOS2 = [
    0.0, 0.0,
    -0.30, 0.20, 0.00, -0.80, 0.00, 0.20, 0.00,
    -0.30, -0.20, 0.00, -0.80, 0.00, 0.20, 0.00,
    0.0, 0.0, 0.0,
    -2.10, 0.00, 0.00, 2.0, -0.4, 0.00,
    -2.10, 0.00, 0.00, 2.0, -0.4, 0.00,
]

_QPOS3 = [
    0.0, 0.0,
    -0.30, 0.20, 0.00, -0.80, 0.00, 0.20, 0.00,
    -0.30, -0.20, 0.00, -0.80, 0.00, 0.20, 0.00,
    0.0, 0.0, 0.0,
    -2.10, 0.00, 0.00, 1.8, -0.5, 0.00,
    -2.10, 0.00, 0.00, 1.8, -0.5, 0.00,
]

_QPOS4 = [
    0.0, 0.0,
    -0.30, 0.20, 0.00, -0.80, 0.00, 0.20, 0.00,
    -0.30, -0.20, 0.00, -0.80, 0.00, 0.20, 0.00,
    0.0, 0.0, 0.0,
    -1.9, 0.00, 0.00, 1.7, -0.5, 0.00,
    -1.9, 0.00, 0.00, 1.7, -0.5, 0.00,
]

_QPOS5 = [
    0.0, 0.0,
    0.200, 0.150, 0.000, -0.500, 0.000, 0.000, 0.000,
    0.200, -0.150, 0.000, -0.500, 0.000, 0.000, 0.000,
    0.0, 0.0, 0.0,
    -0.2, 0.000, 0.000, 0.400, -0.240, 0.000,
    -0.2, 0.000, 0.000, 0.400, -0.240, 0.000,
]

_KEYFRAMES = [_QPOS1, _QPOS2, _QPOS3, _QPOS4, _QPOS5]
_NUM_JOINTS = 31


def _lerp(a: List[float], b: List[float], t: float) -> List[float]:
    return [a[i] + (b[i] - a[i]) * t for i in range(_NUM_JOINTS)]


class T4SquatUpPolicyAdapter(BasePolicyAdapter):

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._step = 0
        self._motion_end = False
        self._init_positions: Optional[Dict[str, float]] = None

    def is_motion_end(self) -> bool:
        return self._motion_end

    def build_model_input(self, context: PolicyContext) -> ModelInput:
        return {"obs": [0.0]}

    def decode_action(
        self, raw_action: Iterable[float], context: PolicyContext
    ) -> List[float]:
        if self._init_positions is None:
            joint_states = context.require_joint_states()
            self._init_positions = {
                s.joint_name: float(s.position) for s in joint_states.states
            }

        step = self._step
        self._step += 1

        if step >= 400:
            self._motion_end = True
            return list(_QPOS5)

        if step < 50:
            current = self._get_ordered_positions(context, self._init_positions)
            return _lerp(current, _QPOS1, step / 50.0)
        elif step < 100:
            return _lerp(_QPOS1, _QPOS2, (step - 50) / 50.0)
        elif step < 150:
            return _lerp(_QPOS2, _QPOS3, (step - 100) / 50.0)
        elif step < 200:
            return _lerp(_QPOS3, _QPOS4, (step - 150) / 50.0)
        else:
            return _lerp(_QPOS4, _QPOS5, (step - 200) / 200.0)

    @staticmethod
    def _get_ordered_positions(
        context: PolicyContext, source: Dict[str, float] | List[float]
    ) -> List[float]:
        if isinstance(source, list):
            return list(source)
        configs = context.require_joint_configs()
        return [float(source.get(j.name, 0.0)) for j in configs]
