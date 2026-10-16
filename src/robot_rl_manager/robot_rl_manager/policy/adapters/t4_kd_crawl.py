"""T4 27-DoF proprioceptive crawl student adapter.

The ONNX policy consumes the exact 627-D actor observation used by SMP:
current proprioception (90), a 9-D straight-line waypoint command, and sixteen
33-D history frames.  No camera or lidar input is used.  The adapter estimates
planar progress by integrating the IMU linear velocity; replacing that estimate
with simulator/robot odometry is recommended when the manager exposes it.
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

# Policy order omits the unactuated waist_pitch and waist_roll entries.
POLICY_IDX = [
    0, 1, 2, 3, 4, 5, 6,
    7, 8, 9, 10, 11, 12, 13,
    16,
    17, 18, 19, 20, 21, 22,
    23, 24, 25, 26, 27, 28,
]
POLICY_JOINT_NAMES = [REAL_JOINT_NAMES[index] for index in POLICY_IDX]

DEFAULT_JOINT_POS = np.asarray([
    0.0, 0.0, 0.0, 0.6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.6, 0.0, 0.0, 0.0,
    0.0,
    -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,
    -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,
], dtype=np.float64)


def matrix_from_quat(q: List[float]) -> np.ndarray:
    w, x, y, z = q
    return np.asarray([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def yaw_from_quat(q: List[float]) -> float:
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def rotate_world_to_heading(vector_w: np.ndarray, heading: float) -> np.ndarray:
    c, s = math.cos(heading), math.sin(heading)
    x, y = float(vector_w[0]), float(vector_w[1])
    if vector_w.shape[0] == 2:
        return np.asarray([c * x + s * y, -s * x + c * y])
    return np.asarray([c * x + s * y, -s * x + c * y, float(vector_w[2])])


class T4KdCrawlPolicyAdapter(BasePolicyAdapter):
    """627-D crawl adapter with a reset-relative virtual waypoint ray."""

    _action_scale = 0.25
    _dt = 0.02
    _target_speed = 0.35
    _waypoint_spacing = 1.0
    _num_waypoints = 8
    _history_length = 16

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._last_action = np.zeros(27, dtype=np.float64)
        self._initial_state: _InitialCrawlState | None = None
        self._settle_start_sec: float | None = None
        self._settle_frames = 0
        self._initial_pose_settled = False
        self._history: list[np.ndarray] = []
        self._position_w = np.zeros(2, dtype=np.float64)
        self._path_yaw_w = 0.0
        self._path_initialized = False
        self._waypoint_index = 0
        self._motion_end = False

    def required_inputs(self) -> set[str]:
        return {"joint_states", "imu"}

    @staticmethod
    def _imu_linear_velocity(imu) -> np.ndarray:
        # Current robot_rl_manager builds may expose either spelling.  The
        # fallback keeps the adapter loadable, but locomotion deployment should
        # supply a real base-linear-velocity/odometry estimate.
        value = getattr(imu, "linear_velocity", None)
        if value is None:
            value = getattr(imu, "linear_velocity_body", (0.0, 0.0, 0.0))
        return np.asarray(value, dtype=np.float64)

    def build_model_input(self, context: PolicyContext) -> ModelInput:
        initial_state = self._load_initial_state(context)
        self._ensure_initial_settle_started(context, initial_state)
        if self._initial_settle_active(context, initial_state):
            return {
                "raw_observation": self._build_initial_settle_observation(
                    context,
                    initial_state,
                ).tolist()
            }

        imu = context.require_policy_imu()
        quat = [float(value) for value in imu.orientation_quat]
        rotation_wb = matrix_from_quat(quat)
        heading_w = yaw_from_quat(quat)
        lin_vel_b = self._imu_linear_velocity(imu)
        ang_vel_b = np.asarray(imu.angular_velocity, dtype=np.float64)
        lin_vel_w = rotation_wb @ lin_vel_b
        ang_vel_w = rotation_wb @ ang_vel_b

        if not self._path_initialized:
            head_axis_w = rotation_wb @ np.asarray([0.0, 0.0, 1.0])
            if np.linalg.norm(head_axis_w[:2]) > 0.1:
                self._path_yaw_w = math.atan2(head_axis_w[1], head_axis_w[0])
            else:
                self._path_yaw_w = heading_w
            self._path_initialized = True
        else:
            self._position_w += lin_vel_w[:2] * self._dt
        path_forward = np.asarray([
            math.cos(self._path_yaw_w), math.sin(self._path_yaw_w)
        ])
        path_left = np.asarray([-path_forward[1], path_forward[0]])
        along = max(float(self._position_w @ path_forward), 0.0)
        lateral = float(self._position_w @ path_left)
        waypoint_along = (self._waypoint_index + 1) * self._waypoint_spacing
        active_target = path_forward * waypoint_along
        target_distance = float(np.linalg.norm(active_target - self._position_w))
        if target_distance <= 0.05 or (
            along >= waypoint_along and abs(lateral) <= 0.25
        ):
            self._waypoint_index += 1
        if self._waypoint_index >= self._num_waypoints:
            self._waypoint_index = self._num_waypoints
            self._motion_end = True

        active_index = min(self._waypoint_index + 1, self._num_waypoints)
        target_w = path_forward * (active_index * self._waypoint_spacing)
        target_offset_w = target_w - self._position_w
        target_b = rotate_world_to_heading(target_offset_w, heading_w)
        tangent_b = rotate_world_to_heading(path_forward, heading_w)
        target_distance = float(np.linalg.norm(target_offset_w))
        total_distance = self._num_waypoints * self._waypoint_spacing
        command = np.asarray([
            target_b[0], target_b[1], 0.0 if self._motion_end else self._target_speed,
            tangent_b[0], tangent_b[1], lateral,
            np.clip(along / total_distance, 0.0, 1.0),
            self._waypoint_index / self._num_waypoints,
            np.clip(target_distance / self._waypoint_spacing, 0.0, 2.0),
        ], dtype=np.float64)

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
        lin_vel_h = rotate_world_to_heading(lin_vel_w, heading_w)
        ang_vel_h = rotate_world_to_heading(ang_vel_w, heading_w)
        frame = np.concatenate((q, lin_vel_h, ang_vel_h))
        if not self._history:
            self._history = [frame.copy() for _ in range(self._history_length)]
        else:
            self._history = self._history[1:] + [frame]

        projected_gravity = rotation_wb.T @ np.asarray([0.0, 0.0, -1.0])
        observation = np.concatenate((
            lin_vel_b,
            ang_vel_b,
            projected_gravity,
            q - DEFAULT_JOINT_POS,
            qd,
            self._last_action,
            command,
            np.concatenate(self._history),
        )).astype(np.float32)
        if observation.shape != (627,):
            raise ValueError(f"t4_kd_crawl obs dim mismatch: {observation.shape} != (627,)")
        if not np.isfinite(observation).all():
            raise ValueError("t4_kd_crawl observation contains non-finite values")
        return {"raw_observation": observation.tolist()}

    def decode_action(
        self, raw_action: Iterable[float], context: PolicyContext
    ) -> List[float]:
        initial_state = self._load_initial_state(context)
        self._ensure_initial_settle_started(context, initial_state)
        if self._initial_settle_active(context, initial_state):
            self._settle_frames += 1
            self._last_action = initial_state.last_action.copy()
            return self._initial_pose_targets(context, initial_state)

        action = np.asarray(list(raw_action), dtype=np.float64)
        if action.shape != (27,):
            raise ValueError(f"t4_kd_crawl action dim mismatch: {action.shape}")
        if not np.isfinite(action).all():
            raise ValueError("t4_kd_crawl action contains non-finite values")
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

    def _load_initial_state(self, context: PolicyContext) -> "_InitialCrawlState":
        """Load the fixed prone state used before crawl inference starts."""
        if self._initial_state is not None:
            return self._initial_state
        model_path = Path(context.require_model_info().file)
        state_path = model_path.with_name("t4_kd_crawl_initial_state.npz")
        if not state_path.is_file():
            raise FileNotFoundError(
                f"t4_kd_crawl fixed initial state not found: {state_path}"
            )
        with np.load(state_path, allow_pickle=False) as data:
            joint_pos = np.asarray(data["joint_pos"], dtype=np.float64)
            joint_vel = np.asarray(data["joint_vel"], dtype=np.float64)
            action = np.asarray(data["last_action_crawl"], dtype=np.float64)
            settling_s = float(data["recommended_settling_s"]) if "recommended_settling_s" in data else 1.0
        for key, value in (
            ("joint_pos", joint_pos),
            ("joint_vel", joint_vel),
            ("last_action_crawl", action),
        ):
            if value.shape != (27,) or not np.isfinite(value).all():
                raise ValueError(
                    f"invalid {key} in {state_path}: {value.shape}"
                )
        if not math.isfinite(settling_s) or settling_s < 0.0:
            raise ValueError(
                f"invalid recommended_settling_s in {state_path}: {settling_s}"
            )
        self._initial_state = _InitialCrawlState(
            joint_pos=joint_pos,
            joint_vel=joint_vel,
            last_action=action,
            settling_s=settling_s,
        )
        return self._initial_state

    def _ensure_initial_settle_started(
        self,
        context: PolicyContext,
        initial_state: "_InitialCrawlState",
    ) -> None:
        if self._settle_start_sec is not None:
            return
        self._settle_start_sec = float(context.now_sec)
        self._settle_frames = 0
        self._last_action = initial_state.last_action.copy()

    def _initial_settle_active(
        self,
        context: PolicyContext,
        initial_state: "_InitialCrawlState",
    ) -> bool:
        if self._initial_pose_settled:
            return False
        if initial_state.settling_s <= 0.0:
            self._initial_pose_settled = True
            return False
        start_sec = self._settle_start_sec
        now_sec = float(context.now_sec)
        if start_sec is not None and now_sec > start_sec:
            if now_sec - start_sec >= initial_state.settling_s:
                self._initial_pose_settled = True
                return False
        elif self._settle_frames * self._dt >= initial_state.settling_s:
            self._initial_pose_settled = True
            return False
        return True

    def _build_initial_settle_observation(
        self,
        context: PolicyContext,
        initial_state: "_InitialCrawlState",
    ) -> np.ndarray:
        imu = context.require_policy_imu()
        quat = [float(value) for value in imu.orientation_quat]
        rotation_wb = matrix_from_quat(quat)
        projected_gravity = rotation_wb.T @ np.asarray([0.0, 0.0, -1.0])
        zeros3 = np.zeros(3, dtype=np.float64)
        command = np.zeros(9, dtype=np.float64)
        frame = np.concatenate((initial_state.joint_pos, zeros3, zeros3))
        observation = np.concatenate((
            zeros3,
            zeros3,
            projected_gravity,
            initial_state.joint_pos - DEFAULT_JOINT_POS,
            initial_state.joint_vel,
            initial_state.last_action,
            command,
            np.tile(frame, self._history_length),
        )).astype(np.float32)
        if observation.shape != (627,):
            raise ValueError(f"t4_kd_crawl initial obs dim mismatch: {observation.shape} != (627,)")
        if not np.isfinite(observation).all():
            raise ValueError("t4_kd_crawl initial observation contains non-finite values")
        return observation

    def _initial_pose_targets(
        self,
        context: PolicyContext,
        initial_state: "_InitialCrawlState",
    ) -> List[float]:
        target_by_name = self._run_profile_positions(context)
        for joint_name, position in zip(POLICY_JOINT_NAMES, initial_state.joint_pos):
            target_by_name[joint_name] = float(position)
        return [
            float(target_by_name.get(joint.name, 0.0))
            for joint in context.require_joint_configs()
        ]

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


class _InitialCrawlState:
    def __init__(
        self,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
        last_action: np.ndarray,
        settling_s: float,
    ) -> None:
        self.joint_pos = joint_pos
        self.joint_vel = joint_vel
        self.last_action = last_action
        self.settling_s = settling_s
