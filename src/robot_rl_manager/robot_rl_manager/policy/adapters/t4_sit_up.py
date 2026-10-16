"""SitUp 策略 adapter — ONNX 参考运动起身模型。

参考旧 policy_t4_raise_up.py：从坐姿起身到站立。
模型输入 144 维 obs，输出 27 维 raw action，映射到 31 关节。
完成后 motion_end=True，节点自动切回 walk。
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Iterable, List

import numpy as np

from robot_rl_manager.policy.adapter import BasePolicyAdapter
from robot_rl_manager.policy.backend import ModelInput
from robot_rl_manager.policy.context import PolicyContext


def matrix_from_quat(q: List[float]) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
    ])


def yaw_quat(q: List[float]) -> List[float]:
    w, x, y, z = q
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]


SITUP_REAL_JOINT_NAMES = [
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
    "left_wrist_yaw", "left_wrist_pitch", "left_wrist_roll",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow",
    "right_wrist_yaw", "right_wrist_pitch", "right_wrist_roll",
    "waist_pitch", "waist_roll", "waist_yaw",
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee",
    "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee",
    "right_ankle_pitch", "right_ankle_roll",
]

POLICY_IDX = [
    0, 1, 2, 3, 4, 5, 6,
    7, 8, 9, 10, 11, 12, 13,
    16,
    17, 18, 19, 20, 21, 22,
    23, 24, 25, 26, 27, 28,
]

_INIT_BLEND_STEPS = 100


class T4SitUpPolicyAdapter(BasePolicyAdapter):

    _action_scale = 0.25

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._motion_step = 0
        self._step = 0
        self._motion_end = False
        self._init_to_world: np.ndarray = np.eye(3, dtype=np.float64)
        self._last_action = [0.0] * len(POLICY_IDX)
        self._motion_data: _MotionData | None = None
        self._init_positions: List[float] | None = None
        self._first_pos: np.ndarray | None = None

    def required_inputs(self) -> set[str]:
        return {"joint_states", "imu"}

    def build_model_input(self, context: PolicyContext) -> ModelInput:
        motion = self._load_motion(context)

        if self._step < _INIT_BLEND_STEPS:
            # 第 0-99 步：从当前关节位置线性插值到运动第一帧
            configs = context.require_joint_configs()
            if self._init_positions is None:
                joint_states = context.require_joint_states()
                by_name = {s.joint_name: s for s in joint_states.states}
                self._init_positions = [
                    float(by_name.get(j.name, 0.0).position) if by_name.get(j.name) else 0.0
                    for j in configs
                ]
            if self._first_pos is None:
                self._first_pos = motion.joint_pos[0]
            self._step += 1
            model_info = context.require_model_info()
            return {"obs": [0.0] * model_info.observation_dim}

        if self._motion_step >= motion.time_step_total:
            self._motion_step = motion.time_step_total - 1
            self._motion_end = True

        imu = context.require_policy_imu()
        q_robot = imu.orientation_quat
        q_ref = motion.body_quat_w[self._motion_step, 0].tolist()
        anchor_quat = q_robot
        ref_anchor_ori_w = q_ref

        if self._motion_step == 0:
            init_to_anchor = matrix_from_quat(yaw_quat(ref_anchor_ori_w))
            world_to_anchor = matrix_from_quat(yaw_quat(anchor_quat))
            self._init_to_world = world_to_anchor @ init_to_anchor.T

        motion_anchor_ori_b = (
            matrix_from_quat(anchor_quat).T
            @ self._init_to_world
            @ matrix_from_quat(ref_anchor_ori_w)
        )

        command_pos = motion.joint_pos[self._motion_step].tolist()
        command_vel = motion.joint_vel[self._motion_step].tolist()

        joint_states = context.require_joint_states()
        by_name = {s.joint_name: s for s in joint_states.states}
        default_pos = self._run_profile_positions(context)
        qpos_29 = []
        qvel_29 = []
        for name in SITUP_REAL_JOINT_NAMES:
            state = by_name.get(name)
            qpos_29.append(float(state.position) if state is not None else 0.0)
            qvel_29.append(float(state.velocity) if state is not None else 0.0)

        obs: List[float] = []
        obs.extend(command_pos)  # 27
        obs.extend(command_vel)  # 27
        obs.extend(motion_anchor_ori_b[:, :2].reshape(-1).tolist())  # 6
        obs.extend([float(v) for v in imu.angular_velocity])  # 3
        obs.extend([
            qpos_29[idx] - default_pos[SITUP_REAL_JOINT_NAMES[idx]]
            for idx in POLICY_IDX
        ])  # 27
        obs.extend([qvel_29[idx] for idx in POLICY_IDX])  # 27
        obs.extend(list(self._last_action))  # 27

        if len(obs) != 144:
            raise ValueError(f"sit_up obs dim mismatch: {len(obs)} != 144")

        motion_step_next = self._motion_step + 1
        if motion_step_next >= motion.time_step_total:
            self._motion_step = motion.time_step_total - 1
            self._motion_end = True
        else:
            self._motion_step = motion_step_next

        return {"obs": obs}

    def decode_action(
        self, raw_action: Iterable[float], context: PolicyContext
    ) -> List[float]:
        if self._step < _INIT_BLEND_STEPS:
            t = self._step / float(_INIT_BLEND_STEPS)
            configs = context.require_joint_configs()
            assert self._init_positions is not None and self._first_pos is not None
            result: List[float] = []
            for i, config in enumerate(configs):
                try:
                    policy_idx = SITUP_REAL_JOINT_NAMES.index(config.name)
                    old_idx = _real_idx_to_policy_idx(policy_idx)
                    if old_idx is not None:
                        init_val = self._init_positions[i]
                        first_val = float(self._first_pos[old_idx])
                        result.append(init_val + (first_val - init_val) * t)
                    else:
                        result.append(self._init_positions[i])
                except ValueError:
                    result.append(0.0)
            self._step += 1
            return result

        action = [float(v) for v in raw_action]
        if len(action) != len(POLICY_IDX):
            raise ValueError(
                f"sit_up raw action dim mismatch: {len(action)} != {len(POLICY_IDX)}"
            )
        self._last_action = list(action)

        profile_positions = self._run_profile_positions(context)
        amp_targets = [float(profile_positions[name]) for name in SITUP_REAL_JOINT_NAMES]
        for action_idx, real_idx in enumerate(POLICY_IDX):
            amp_targets[real_idx] += self._action_scale * action[action_idx]

        target_by_name = dict(profile_positions)
        for joint_name, target in zip(SITUP_REAL_JOINT_NAMES, amp_targets):
            target_by_name[joint_name] = target

        return [
            float(target_by_name.get(joint.name, 0.0))
            for joint in context.require_joint_configs()
        ]

    def select_joint_profile(self, context: PolicyContext) -> str:
        del context
        return "run"

    def is_motion_end(self) -> bool:
        return self._motion_end

    def _load_motion(self, context: PolicyContext) -> _MotionData:
        if self._motion_data is not None:
            return self._motion_data
        model_info = context.require_model_info()
        model_path = Path(model_info.file)
        motion_path = model_path.with_suffix(".npz")
        if not motion_path.exists():
            raise FileNotFoundError(
                f"sit_up motion file not found: {motion_path}"
            )
        data = np.load(str(motion_path))
        self._motion_data = _MotionData(
            joint_pos=np.asarray(data["joint_pos"], dtype=np.float64),
            joint_vel=np.asarray(data["joint_vel"], dtype=np.float64),
            body_quat_w=np.asarray(data["body_quat_w"], dtype=np.float64)[:, [0]],
            time_step_total=int(data["joint_pos"].shape[0]),
        )
        return self._motion_data

    @staticmethod
    def _run_profile_positions(context: PolicyContext) -> Dict[str, float]:
        model_info = context.require_model_info()
        profile = model_info.joint_profiles.get("run")
        if profile is None:
            raise ValueError(f"model {model_info.name} missing run joint profile")
        positions: Dict[str, float] = {}
        for joint in context.require_joint_configs():
            profile_joint = profile.joints.get(joint.name)
            if profile_joint is None:
                raise ValueError(f"run profile missing joint: {joint.name}")
            positions[joint.name] = float(profile_joint.position)
        for joint_name in SITUP_REAL_JOINT_NAMES:
            if joint_name not in positions:
                raise ValueError(f"run profile missing sit_up joint: {joint_name}")
        return positions


def _real_idx_to_policy_idx(real_idx: int) -> int | None:
    try:
        return POLICY_IDX.index(real_idx)
    except ValueError:
        return None


class _MotionData:

    def __init__(
        self,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
        body_quat_w: np.ndarray,
        time_step_total: int,
    ) -> None:
        self.joint_pos = joint_pos
        self.joint_vel = joint_vel
        self.body_quat_w = body_quat_w
        self.time_step_total = time_step_total
