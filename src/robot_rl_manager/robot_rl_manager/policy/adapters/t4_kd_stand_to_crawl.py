"""Shared standalone adapter template for T4 stand-to-crawl tracking models.

Each deployment package receives an identical copy of this file beside its
ONNX and reference NPZ.  The policy uses the exact 150-D actor observation from
Mjlab and advances the finite reference at 50 Hz.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Iterable, List

import numpy as np

from robot_rl_manager.policy.adapter import BasePolicyAdapter
from robot_rl_manager.policy.backend import ModelInput
from robot_rl_manager.policy.context import PolicyContext


REAL_JOINT_NAMES = [
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

# Training has 27 policy DoF; waist_pitch and waist_roll are not policy actions.
POLICY_IDX = [
    0, 1, 2, 3, 4, 5, 6,
    7, 8, 9, 10, 11, 12, 13,
    16,
    17, 18, 19, 20, 21, 22,
    23, 24, 25, 26, 27, 28,
]

# This is the T4 asset default used by the joint_pos_rel observation.  It is
# intentionally different from the stand-to-crawl action offset in the YAML.
DEFAULT_JOINT_POS = np.asarray([
    0.0, 0.0, 0.0, 0.6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.6, 0.0, 0.0, 0.0,
    0.0,
    -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,
    -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,
], dtype=np.float64)


def matrix_from_quat(q: List[float]) -> np.ndarray:
    """Return body-to-world rotation for a scalar-first quaternion."""
    w, x, y, z = q
    return np.asarray([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def yaw_rotation(q: List[float]) -> np.ndarray:
    w, x, y, z = q
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class T4KdStandToCrawlPolicyAdapter(BasePolicyAdapter):
    """Finite 150-D BeyondMimic stand-to-crawl tracking adapter."""

    _action_scale = 0.25
    _dt = 0.02

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._motion_step = 0
        self._motion_end = False
        self._last_action = np.zeros(27, dtype=np.float64)
        self._motion_data: _MotionData | None = None
        self._init_to_world = np.eye(3, dtype=np.float64)
        self._robot_anchor_pos_w = np.zeros(3, dtype=np.float64)
        self._reference_origin_w = np.zeros(3, dtype=np.float64)
        self._alignment_initialized = False

    def required_inputs(self) -> set[str]:
        return {"joint_states", "imu"}

    @staticmethod
    def _imu_linear_velocity(imu) -> np.ndarray:
        value = getattr(imu, "linear_velocity", None)
        if value is None:
            value = getattr(imu, "linear_velocity_body", (0.0, 0.0, 0.0))
        return np.asarray(value, dtype=np.float64)

    def build_model_input(self, context: PolicyContext) -> ModelInput:
        motion = self._load_motion(context)
        step = min(self._motion_step, motion.time_step_total - 1)

        imu = context.require_policy_imu()
        q_robot = [float(value) for value in imu.orientation_quat]
        rotation_wb = matrix_from_quat(q_robot)
        lin_vel_b = self._imu_linear_velocity(imu)
        ang_vel_b = np.asarray(imu.angular_velocity, dtype=np.float64)
        lin_vel_w = rotation_wb @ lin_vel_b

        q_ref = motion.body_quat_w[step, 0].tolist()
        if not self._alignment_initialized:
            # Match only reference yaw to the robot's reset yaw, exactly as the
            # tracking command does.  Roll/pitch remain part of the target.
            self._init_to_world = yaw_rotation(q_robot) @ yaw_rotation(q_ref).T
            self._reference_origin_w = (
                self._init_to_world @ motion.body_pos_w[0, 0]
            )
            self._alignment_initialized = True
        else:
            self._robot_anchor_pos_w += lin_vel_w * self._dt

        desired_anchor_w = (
            self._init_to_world @ motion.body_pos_w[step, 0]
            - self._reference_origin_w
        )
        anchor_pos_b = rotation_wb.T @ (
            desired_anchor_w - self._robot_anchor_pos_w
        )
        anchor_ori_b = (
            rotation_wb.T
            @ self._init_to_world
            @ matrix_from_quat(q_ref)
        )

        joint_states = context.require_joint_states()
        by_name = {state.joint_name: state for state in joint_states.states}
        q_real = np.asarray([
            float(by_name[name].position) if name in by_name else 0.0
            for name in REAL_JOINT_NAMES
        ])
        qd_real = np.asarray([
            float(by_name[name].velocity) if name in by_name else 0.0
            for name in REAL_JOINT_NAMES
        ])
        q = q_real[POLICY_IDX]
        qd = qd_real[POLICY_IDX]

        observation = np.concatenate((
            motion.joint_pos[step],
            motion.joint_vel[step],
            anchor_pos_b,
            anchor_ori_b[:, :2].reshape(-1),
            lin_vel_b,
            ang_vel_b,
            q - DEFAULT_JOINT_POS,
            qd,
            self._last_action,
        )).astype(np.float32)
        if observation.shape != (150,):
            raise ValueError(
                f"stand-to-crawl obs dim mismatch: {observation.shape} != (150,)"
            )
        if not np.isfinite(observation).all():
            raise ValueError("stand-to-crawl observation contains non-finite values")

        next_step = step + 1
        if next_step >= motion.time_step_total:
            self._motion_step = motion.time_step_total - 1
            self._motion_end = True
        else:
            self._motion_step = next_step
        return {"obs": observation.tolist()}

    def decode_action(
        self, raw_action: Iterable[float], context: PolicyContext
    ) -> List[float]:
        action = np.asarray(list(raw_action), dtype=np.float64)
        if action.shape != (27,):
            raise ValueError(f"stand-to-crawl action dim mismatch: {action.shape}")
        if not np.isfinite(action).all():
            raise ValueError("stand-to-crawl action contains non-finite values")
        action = np.clip(action, -8.0, 8.0)
        self._last_action = action.copy()

        profile_positions = self._run_profile_positions(context)
        targets = [profile_positions[name] for name in REAL_JOINT_NAMES]
        for action_idx, real_idx in enumerate(POLICY_IDX):
            targets[real_idx] += self._action_scale * action[action_idx]
        target_by_name = dict(zip(REAL_JOINT_NAMES, targets))
        return [
            float(target_by_name.get(joint.name, 0.0))
            for joint in context.require_joint_configs()
        ]

    def select_joint_profile(self, context: PolicyContext) -> str:
        del context
        return "run"

    def is_motion_end(self) -> bool:
        return self._motion_end

    def _load_motion(self, context: PolicyContext) -> "_MotionData":
        if self._motion_data is not None:
            return self._motion_data
        model_path = Path(context.require_model_info().file)
        motion_path = model_path.with_suffix(".npz")
        if not motion_path.exists():
            raise FileNotFoundError(
                f"stand-to-crawl reference motion not found: {motion_path}"
            )
        with np.load(str(motion_path)) as data:
            self._motion_data = _MotionData(
                joint_pos=np.asarray(data["joint_pos"], dtype=np.float64),
                joint_vel=np.asarray(data["joint_vel"], dtype=np.float64),
                body_pos_w=np.asarray(data["body_pos_w"], dtype=np.float64)[:, [0]],
                body_quat_w=np.asarray(data["body_quat_w"], dtype=np.float64)[:, [0]],
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
        for name in REAL_JOINT_NAMES:
            if name not in positions:
                raise ValueError(f"run profile missing policy joint: {name}")
        return positions


class _MotionData:
    def __init__(
        self,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
        body_pos_w: np.ndarray,
        body_quat_w: np.ndarray,
    ) -> None:
        if joint_pos.ndim != 2 or joint_pos.shape[1] != 27:
            raise ValueError(f"reference joint_pos must be [T,27], got {joint_pos.shape}")
        if joint_vel.shape != joint_pos.shape:
            raise ValueError(
                f"reference joint_vel shape {joint_vel.shape} != {joint_pos.shape}"
            )
        if body_pos_w.shape[:2] != (joint_pos.shape[0], 1):
            raise ValueError("reference body_pos_w length does not match joint_pos")
        if body_quat_w.shape[:2] != (joint_pos.shape[0], 1):
            raise ValueError("reference body_quat_w length does not match joint_pos")
        self.joint_pos = joint_pos
        self.joint_vel = joint_vel
        self.body_pos_w = body_pos_w
        self.body_quat_w = body_quat_w
        self.time_step_total = int(joint_pos.shape[0])
