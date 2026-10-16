"""踢球策略 adapter — 足球踢球模型的语义适配器。

与 heart 等动作模型相比的区别：
  - 需要额外感知输入 target_point（球，基座系）与 target_destination（球门，基座系）。
  - 单帧 150 维 obs：command(54) + anchor_ori(6) + ang(3) + qpos(27) + qvel(27) + action(27) + ball(3) + goal(3)
  - 根据球的横向位置动态选择 5 个参考动作 npz（forward/left45/left90/right45/right90）。
  - 模型原始输出 27 维 raw action，经 POLICY_IDX 映射到全身关节目标。
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


KICK_REAL_JOINT_NAMES = [
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

OBS_GROUP_KEYS = [
    "command",
    "motion_anchor_ori_b",
    "base_ang_vel",
    "joint_pos",
    "joint_vel",
    "actions",
    "ball_pos",
    "goal_pos",
]

OBS_GROUP_DIMS = [54, 6, 3, 27, 27, 27, 3, 3]


class T4KickPolicyAdapter(BasePolicyAdapter):
    _action_scale = 0.25

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._motion_step = 0
        self._motion_end = False
        self._init_to_world: np.ndarray = np.eye(3, dtype=np.float64)
        self._last_action = [0.0] * len(POLICY_IDX)
        self._obs: Dict[str, List[float]] = {}
        for key, dim in zip(OBS_GROUP_KEYS, OBS_GROUP_DIMS):
            self._obs[key] = [0.0] * dim
        self._motion_data: _MotionData | None = None

    def required_inputs(self) -> set[str]:
        return {"joint_states", "imu", "target_point", "target_destination"}

    def build_model_input(self, context: PolicyContext) -> ModelInput:
        motion = self._load_motion(context)

        command_pos = motion.joint_pos[self._motion_step].tolist()
        command_vel = motion.joint_vel[self._motion_step].tolist()
        self._obs["command"] = command_pos + command_vel

        imu = context.require_policy_imu()
        q_robot = imu.orientation_quat
        q_ref = motion.body_quat_w[self._motion_step, 0].tolist()

        if self._motion_step == 0:
            init_to_anchor = matrix_from_quat(yaw_quat(q_ref))
            world_to_anchor = matrix_from_quat(yaw_quat(q_robot))
            self._init_to_world = world_to_anchor @ init_to_anchor.T

        ori_b_matrix = matrix_from_quat(q_robot).T @ self._init_to_world @ matrix_from_quat(q_ref)
        self._obs["motion_anchor_ori_b"] = ori_b_matrix[:, :2].reshape(-1).tolist()

        self._obs["base_ang_vel"] = [float(v) for v in imu.angular_velocity]

        joint_states = context.require_joint_states()
        by_name = {s.joint_name: s for s in joint_states.states}
        default_pos = self._run_profile_positions(context)
        qpos_29 = []
        qvel_29 = []
        for name in KICK_REAL_JOINT_NAMES:
            state = by_name.get(name)
            qpos_29.append(float(state.position) if state is not None else 0.0)
            qvel_29.append(float(state.velocity) if state is not None else 0.0)
        self._obs["joint_pos"] = [
            qpos_29[idx] - default_pos[KICK_REAL_JOINT_NAMES[idx]]
            for idx in POLICY_IDX
        ]
        self._obs["joint_vel"] = [qvel_29[idx] for idx in POLICY_IDX]

        self._obs["actions"] = list(self._last_action)

        # 目标点信息（基座标系下，由 C++ 层变换后发布）
        tp = context.require_target_point()
        td = context.require_target_destination()
        self._obs["ball_pos"] = [float(tp.x), float(tp.y), float(tp.z)]
        self._obs["goal_pos"] = [float(td.x), float(td.y), float(td.z)]

        obs: List[float] = []
        for key in OBS_GROUP_KEYS:
            obs.extend(self._obs[key])
        self._validate_dims(obs)

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
        action = [float(v) for v in raw_action]
        if len(action) != len(POLICY_IDX):
            raise ValueError(
                f"kick raw action dim mismatch: {len(action)} != {len(POLICY_IDX)}"
            )
        self._last_action = list(action)

        profile_positions = self._run_profile_positions(context)
        amp_targets = [float(profile_positions[name]) for name in KICK_REAL_JOINT_NAMES]
        for action_idx, real_idx in enumerate(POLICY_IDX):
            amp_targets[real_idx] += self._action_scale * action[action_idx]

        target_by_name = dict(profile_positions)
        for joint_name, target in zip(KICK_REAL_JOINT_NAMES, amp_targets):
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

        # 根据球的相对位置动态选择对应的踢球参考动作文件
        suffix = "_forward"
        try:
            tp = context.require_target_point()
            y = tp.y
            if y > 0.25:
                suffix = "_right45" if y > 0.5 else "_right90"
            elif y < -0.25:
                suffix = "_left45" if y < -0.5 else "_left90"
        except Exception:
            pass

        motion_path = model_path.parent / f"t4_kick{suffix}.npz"
        if not motion_path.exists():
            motion_path = model_path.with_suffix(".npz")

        if not motion_path.exists():
            raise FileNotFoundError(
                f"kick motion file not found: {motion_path} "
                f"(expected alongside {model_path})"
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
        for joint_name in KICK_REAL_JOINT_NAMES:
            if joint_name not in positions:
                raise ValueError(f"run profile missing kick joint: {joint_name}")
        return positions

    @staticmethod
    def _validate_dims(obs: List[float]) -> None:
        if len(obs) != 150:
            raise ValueError(
                f"kick obs dim mismatch: {len(obs)} != 150"
            )


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
