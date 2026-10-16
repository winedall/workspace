"""RaiseDown 策略 adapter — 硬编码七段线性插值，无 ONNX 模型。

参考旧 policy_t4_raise_down.py 的七阶段插值逻辑：站立 → 深蹲 → 坐地 → 保持坐姿。
输出 31 关节（robot_joints.yaml 顺序），头、waist_pitch、waist_roll 保持 profile 默认位置。
"""

from __future__ import annotations

from typing import Dict, Iterable, List, Optional

from robot_rl_manager.policy.adapter import BasePolicyAdapter
from robot_rl_manager.policy.backend import ModelInput
from robot_rl_manager.policy.context import PolicyContext

# ── 七阶段轨迹关键帧（31 关节，robot_joints.yaml 顺序） ──────────────────────
# 顺序: head(2), L_arm(7), R_arm(7), waist(3), L_leg(6), R_leg(6)
# init(50) → q1→q2(150) → q2→q3(100) → q3→q4(100) → q4→q5(100) → q5→q6(50) → q6→q7(50) → hold q7

_QPOS1 = [
    0.0, 0.0,                                          # head_yaw, head_pitch
    0.20, 0.15, 0.0, -0.5, 0.0, 0.0, 0.0,              # left_arm
    0.20, -0.15, 0.0, -0.5, 0.0, 0.0, 0.0,             # right_arm
    # waist_pitch, waist_roll, waist_yaw
    0.0, 0.0, 0.0,
    -0.2, 0.0, 0.0, 0.4, -0.24, 0.0,                   # left_leg
    -0.2, 0.0, 0.0, 0.4, -0.24, 0.0,                   # right_leg
]

_QPOS2 = [
    0.0, 0.0,
    -1.2, 0.1, -1.30, 0.2, 0.00, 0.00, 0.5,
    -1.2, -0.1, 1.30, 0.2, 0.00, 0.00, -0.5,
    0.0, 0.0, 0.0,
    -2.0, 0.0, 0.0, 1.7, -0.5, 0.0,
    -2.0, 0.0, 0.0, 1.7, -0.5, 0.0,
]

_QPOS3 = [
    0.0, 0.0,
    -1.8, 0.1, -1.30, 0.2, 0.00, 0.00, 1.0,
    -1.8, -0.1, 1.30, 0.2, 0.00, 0.00, -1.0,
    0.0, 0.0, 0.0,
    -2.4, 0.0, 0.00, 2.0, -0.6, 0.0,
    -2.4, 0.0, 0.00, 2.0, -0.6, 0.0,
]

_QPOS4 = [
    0.0, 0.0,
    -1.8, 0.1, -1.30, 0.2, 0.00, 0.00, 0.5,
    -1.8, -0.1, 1.30, 0.2, 0.00, 0.00, -0.5,
    0.0, 0.0, 0.0,
    -2.8, 0.5, 0.0, 1.8, 0.0, 0.26,
    -2.8, -0.5, 0.0, 1.8, 0.0, -0.26,
]

_QPOS5 = [
    0.0, 0.0,
    -1.8, 0.1, -1.30, 0.2, 0.00, 0.00, 0.5,
    -1.8, -0.1, 1.30, 0.2, 0.00, 0.00, -0.5,
    0.0, 0.0, 0.0,
    -2.8, 0.2, 0.0, 2.0, 0.5, 0.0,
    -2.8, -0.2, 0.0, 2.0, 0.5, 0.0,
]

_QPOS6 = [
    0.0, 0.0,
    -1.8, 1.5, -1.30, 0.2, 0.00, 0.00, 0.5,
    -1.8, -1.5, 1.30, 0.2, 0.00, 0.00, -0.5,
    0.0, 0.0, 0.0,
    -2.8, 0.2, -0.2, 1.9, 0.5, 0.0,
    -2.8, -0.2, 0.2, 1.9, 0.5, 0.0,
]

_QPOS7 = [
    0.0, 0.0,
    -0.3, 0.8, -0.8, -0.8, 0.20, 0.00, 0.5,
    -0.3, -0.8, 0.8, -0.8, 0.20, 0.00, -0.5,
    0.0, 0.0, 0.0,
    -2.7, 0.2, -0.2, 1.9, 0.5, 0.0,
    -2.7, -0.2, 0.20, 1.9, 0.5, 0.0,
]

_KEYFRAMES = [_QPOS1, _QPOS2, _QPOS3, _QPOS4, _QPOS5, _QPOS6, _QPOS7]
_PHASE_STEPS = [150, 100, 100, 100, 50, 50]
_NUM_JOINTS = 31


def _lerp(a: List[float], b: List[float], t: float) -> List[float]:
    return [a[i] + (b[i] - a[i]) * t for i in range(_NUM_JOINTS)]


class T4RaiseDownPolicyAdapter(BasePolicyAdapter):

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._step = 0
        self._init_positions: Optional[Dict[str, float]] = None

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

        if step < 50:
            t = step / 50.0
            current = self._get_ordered_positions(
                context, self._init_positions)
            return _lerp(current, _QPOS1, t)

        rel = step - 50

        for phase, n_steps in enumerate(_PHASE_STEPS):
            if rel < n_steps:
                return _lerp(_KEYFRAMES[phase], _KEYFRAMES[phase + 1], rel / float(n_steps))
            rel -= n_steps

        return list(_QPOS7)

    @staticmethod
    def _get_ordered_positions(
        context: PolicyContext, source: Dict[str, float] | List[float]
    ) -> List[float]:
        if isinstance(source, list):
            return list(source)
        configs = context.require_joint_configs()
        return [float(source.get(j.name, 0.0)) for j in configs]
