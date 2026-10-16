"""Adapter for the mjlab T4 broadjump motion-tracking policy.

The policy was trained with the No-State-Estimation actor observation:
  command(54) + anchor_orientation(6) + angular_velocity(3)
  + joint_position(27) + joint_velocity(27) + previous_action(27) = 144.

The exported ONNX also has a ``time_step`` input. Its first output is the
27-dimensional policy action; the remaining outputs contain bundled reference
motion data and are intentionally ignored by robot_rl_manager's ONNX backend.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Iterable, List

import numpy as np
from robot_rl_manager.policy.adapter import BasePolicyAdapter
from robot_rl_manager.policy.backend import ModelInput
from robot_rl_manager.policy.context import PolicyContext

# Exact order used by the mjlab T4 entity and converted 27-DoF motions.
POLICY_JOINT_NAMES = [
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_yaw",
    "left_wrist_pitch",
    "left_wrist_roll",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
    "right_wrist_yaw",
    "right_wrist_pitch",
    "right_wrist_roll",
    "waist_yaw",
    "left_hip_pitch",
    "left_hip_roll",
    "left_hip_yaw",
    "left_knee",
    "left_ankle_pitch",
    "left_ankle_roll",
    "right_hip_pitch",
    "right_hip_roll",
    "right_hip_yaw",
    "right_knee",
    "right_ankle_pitch",
    "right_ankle_roll",
]

# Exact T4_ACTION_SCALE values from constants_t4.py, in POLICY_JOINT_NAMES order.
ACTION_SCALES = [
    0.3125,
    0.3125,
    0.3125,
    0.3125,
    0.15,
    0.3,
    0.3,
    0.3125,
    0.3125,
    0.3125,
    0.3125,
    0.15,
    0.3,
    0.3,
    0.6,
    0.325,
    0.6,
    0.6,
    0.325,
    1.8,
    1.8,
    0.325,
    0.6,
    0.6,
    0.325,
    1.8,
    1.8,
]


def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> List[float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def _matrix_from_quaternion(quaternion: Iterable[float]) -> np.ndarray:
    w, x, y, z = [float(value) for value in quaternion]
    norm_sq = w * w + x * x + y * y + z * z
    if norm_sq <= 1.0e-12:
        raise ValueError("tracking quaternion has zero norm")
    scale = 2.0 / norm_sq
    return np.asarray(
        [
            [
                1.0 - scale * (y * y + z * z),
                scale * (x * y - z * w),
                scale * (x * z + y * w),
            ],
            [
                scale * (x * y + z * w),
                1.0 - scale * (x * x + z * z),
                scale * (y * z - x * w),
            ],
            [
                scale * (x * z - y * w),
                scale * (y * z + x * w),
                1.0 - scale * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def _yaw_quaternion(quaternion: Iterable[float]) -> List[float]:
    w, x, y, z = [float(value) for value in quaternion]
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return [math.cos(yaw * 0.5), 0.0, 0.0, math.sin(yaw * 0.5)]


class T4KdBroadjump14000PolicyAdapter(BasePolicyAdapter):
    """Run one pass of the broadjump reference motion."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._motion_step = 0
        self._pending_step: int | None = None
        self._motion_end = False
        self._last_frame_decoded = False
        self._initial_reference_to_world = np.eye(3, dtype=np.float64)
        self._last_action = [0.0] * len(POLICY_JOINT_NAMES)
        self._motion_data: _MotionData | None = None

    def required_inputs(self) -> set[str]:
        return {"joint_states", "imu"}

    def build_model_input(self, context: PolicyContext) -> ModelInput:
        motion = self._load_motion(context)
        step = self._motion_step
        profile_positions = self._run_profile_positions(context)

        command = np.concatenate((motion.joint_pos[step], motion.joint_vel[step]))

        imu = context.require_policy_imu()
        robot_quaternion = self._fixed_len(
            imu.orientation_quat, 4, "orientation_quat"
        )
        reference_quaternion = motion.anchor_quat_w[step]
        if step == 0:
            initial_to_anchor = _matrix_from_quaternion(
                _yaw_quaternion(reference_quaternion)
            )
            world_to_anchor = _matrix_from_quaternion(_yaw_quaternion(robot_quaternion))
            self._initial_reference_to_world = world_to_anchor @ initial_to_anchor.T

        orientation_b = (
            _matrix_from_quaternion(robot_quaternion).T
            @ self._initial_reference_to_world
            @ _matrix_from_quaternion(reference_quaternion)
        )
        anchor_orientation = orientation_b[:, :2].reshape(-1)

        states_by_name = {
            state.joint_name: state for state in context.require_joint_states().states
        }
        missing_states = [
            name for name in POLICY_JOINT_NAMES if name not in states_by_name
        ]
        if missing_states:
            raise ValueError(f"tracking joint states missing: {missing_states}")

        joint_position = [
            float(states_by_name[name].position) - profile_positions[name]
            for name in POLICY_JOINT_NAMES
        ]
        joint_velocity = [
            float(states_by_name[name].velocity) for name in POLICY_JOINT_NAMES
        ]

        obs = np.concatenate(
            (
                command,
                anchor_orientation,
                np.asarray(
                    self._fixed_len(imu.angular_velocity, 3, "angular_velocity"),
                    dtype=np.float64,
                ),
                np.asarray(joint_position, dtype=np.float64),
                np.asarray(joint_velocity, dtype=np.float64),
                np.asarray(self._last_action, dtype=np.float64),
            )
        )
        if obs.shape != (144,):
            raise ValueError(f"tracking obs shape mismatch: {obs.shape} != (144,)")
        if not np.all(np.isfinite(obs)):
            raise ValueError("tracking observation contains NaN or infinity")

        self._pending_step = step
        return {"obs": obs.tolist(), "time_step": [float(step)]}

    def decode_action(
        self, raw_action: Iterable[float], context: PolicyContext
    ) -> List[float]:
        action = np.asarray(list(raw_action), dtype=np.float64)
        if action.shape != (len(POLICY_JOINT_NAMES),):
            raise ValueError(
                f"tracking action shape mismatch: {action.shape} "
                f"!= ({len(POLICY_JOINT_NAMES)},)"
            )
        if not np.all(np.isfinite(action)):
            raise ValueError("tracking action contains NaN or infinity")

        profile_positions = self._run_profile_positions(context)
        target_by_name = dict(profile_positions)
        for name, scale, value in zip(
            POLICY_JOINT_NAMES, ACTION_SCALES, action, strict=True
        ):
            target_by_name[name] = profile_positions[name] + scale * float(value)

        self._last_action = action.tolist()
        self._advance_motion()
        return [
            float(target_by_name[joint.name])
            for joint in context.require_joint_configs()
        ]

    def select_joint_profile(self, context: PolicyContext) -> str:
        del context
        return "run"

    def is_motion_end(self) -> bool:
        return self._motion_end

    def _advance_motion(self) -> None:
        if self._motion_data is None or self._pending_step is None:
            raise RuntimeError("tracking action decoded without a pending motion frame")
        if self._pending_step != self._motion_step:
            raise RuntimeError("tracking motion step changed before action decoding")
        self._pending_step = None
        if self._motion_step + 1 >= self._motion_data.frame_count:
            # Hold the last target for one full inference interval so the command
            # publish timer can emit it before the manager switches back to walk.
            if self._last_frame_decoded:
                self._motion_end = True
            else:
                self._last_frame_decoded = True
            return
        self._motion_step += 1

    def _load_motion(self, context: PolicyContext) -> "_MotionData":
        if self._motion_data is not None:
            return self._motion_data

        model_path = Path(context.require_model_info().file)
        motion_path = model_path.with_suffix(".npz")
        if not motion_path.exists():
            raise FileNotFoundError(
                f"tracking motion file not found: {motion_path}; "
                "place the matching .npz beside the ONNX file"
            )
        data = np.load(str(motion_path))
        joint_pos = np.asarray(data["joint_pos"], dtype=np.float64)
        joint_vel = np.asarray(data["joint_vel"], dtype=np.float64)
        body_quat_w = np.asarray(data["body_quat_w"], dtype=np.float64)
        if joint_pos.ndim != 2 or joint_pos.shape[1] != len(POLICY_JOINT_NAMES):
            raise ValueError(
                f"tracking motion joint_pos shape invalid: {joint_pos.shape}"
            )
        if joint_vel.shape != joint_pos.shape:
            raise ValueError(
                f"tracking motion joint_vel shape {joint_vel.shape} "
                f"does not match joint_pos {joint_pos.shape}"
            )
        if body_quat_w.ndim != 3 or body_quat_w.shape[0] != joint_pos.shape[0]:
            raise ValueError(
                f"tracking motion body_quat_w shape invalid: {body_quat_w.shape}"
            )
        if not all(
            np.all(np.isfinite(array)) for array in (joint_pos, joint_vel, body_quat_w)
        ):
            raise ValueError("tracking motion contains NaN or infinity")

        self._motion_data = _MotionData(
            joint_pos=joint_pos,
            joint_vel=joint_vel,
            anchor_quat_w=body_quat_w[:, 0],
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
        for name in POLICY_JOINT_NAMES:
            if name not in positions:
                raise ValueError(f"run profile missing policy joint: {name}")
        return positions

    @staticmethod
    def _fixed_len(values: Iterable[float], expected_len: int, name: str) -> List[float]:
        result = [float(value) for value in values]
        if len(result) != expected_len:
            raise ValueError(f"{name} dim mismatch: {len(result)} != {expected_len}")
        return result


class _MotionData:
    def __init__(
        self,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
        anchor_quat_w: np.ndarray,
    ) -> None:
        self.joint_pos = joint_pos
        self.joint_vel = joint_vel
        self.anchor_quat_w = anchor_quat_w
        self.frame_count = int(joint_pos.shape[0])
