"""PHP T4 box-climb tracking policy adapter."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Iterable, List

import numpy as np

from robot_rl_manager.policy.adapter import BasePolicyAdapter
from robot_rl_manager.policy.backend import ModelInput
from robot_rl_manager.policy.context import PolicyContext


POLICY_JOINT_NAMES = [
    "left_shoulder_pitch",
    "right_shoulder_pitch",
    "waist_yaw",
    "left_shoulder_roll",
    "right_shoulder_roll",
    "left_hip_pitch",
    "right_hip_pitch",
    "left_shoulder_yaw",
    "right_shoulder_yaw",
    "left_hip_roll",
    "right_hip_roll",
    "left_elbow",
    "right_elbow",
    "left_hip_yaw",
    "right_hip_yaw",
    "left_wrist_yaw",
    "right_wrist_yaw",
    "left_knee",
    "right_knee",
    "left_wrist_pitch",
    "right_wrist_pitch",
    "left_ankle_pitch",
    "right_ankle_pitch",
    "left_wrist_roll",
    "right_wrist_roll",
    "left_ankle_roll",
    "right_ankle_roll",
]

DEFAULT_POLICY_JOINT_POS = dict.fromkeys(POLICY_JOINT_NAMES, 0.0)
DEFAULT_POLICY_JOINT_POS["left_shoulder_roll"] = 0.25
DEFAULT_POLICY_JOINT_POS["right_shoulder_roll"] = -0.25

MOTION_JOINT_NAMES = [
    "J_arm_l_01",
    "J_arm_r_01",
    "J_waist_yaw",
    "J_arm_l_02",
    "J_arm_r_02",
    "J_hip_l_pitch",
    "J_hip_r_pitch",
    "J_arm_l_03",
    "J_arm_r_03",
    "J_hip_l_roll",
    "J_hip_r_roll",
    "J_arm_l_04",
    "J_arm_r_04",
    "J_hip_l_yaw",
    "J_hip_r_yaw",
    "J_arm_l_05",
    "J_arm_r_05",
    "J_knee_l_pitch",
    "J_knee_r_pitch",
    "J_arm_l_06",
    "J_arm_r_06",
    "J_ankle_l_pitch",
    "J_ankle_r_pitch",
    "J_arm_l_07",
    "J_arm_r_07",
    "J_ankle_l_roll",
    "J_ankle_r_roll",
]

OBS_DIMS = {
    "command": 54,
    "motion_anchor_pos_b": 3,
    "motion_anchor_ori_b": 6,
    "base_lin_vel": 3,
    "base_ang_vel": 3,
    "joint_pos": 27,
    "joint_vel": 27,
    "actions": 27,
}


def _matrix_from_quat(quat: Iterable[float]) -> np.ndarray:
    w, x, y, z = [float(v) for v in quat]
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isfinite(norm) or norm < 1.0e-8:
        raise ValueError("invalid orientation quaternion")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _yaw_matrix(quat: Iterable[float]) -> np.ndarray:
    w, x, y, z = [float(v) for v in quat]
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return np.asarray(
        [[cos_yaw, -sin_yaw, 0.0], [sin_yaw, cos_yaw, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


class T4KdPhpBoxclimbPolicyAdapter(BasePolicyAdapter):
    """把 robot_rl_manager 上下文转换为 PHP tracking 的 150 维观测。"""

    _action_scale = 0.5
    _expected_frequency_hz = 50.0

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._motion_step = 0
        self._motion_end = False
        self._last_action = np.zeros(27, dtype=np.float64)
        self._motion_data: _MotionData | None = None
        self._alignment_initialized = False
        self._init_to_world = np.eye(3, dtype=np.float64)
        self._reference_origin = np.zeros(3, dtype=np.float64)
        self._robot_origin = np.zeros(3, dtype=np.float64)
        self._estimated_robot_pos = np.zeros(3, dtype=np.float64)

    def required_inputs(self) -> set[str]:
        return {"joint_states", "imu"}

    def build_model_input(self, context: PolicyContext) -> ModelInput:
        motion = self._load_motion(context)
        step = self._motion_step
        imu = context.require_policy_imu()
        robot_quat = self._require_vector(imu.orientation_quat, 4, "orientation_quat")
        robot_rotation = _matrix_from_quat(robot_quat)
        world_linear_velocity = np.asarray(
            self._require_vector(imu.linear_velocity, 3, "linear_velocity"), dtype=np.float64
        )
        body_angular_velocity = self._require_vector(
            imu.angular_velocity, 3, "angular_velocity"
        )

        reference_pos = motion.body_pos_w[step, motion.anchor_body_index]
        reference_quat = motion.body_quat_w[step, motion.anchor_body_index]
        if not self._alignment_initialized:
            self._init_to_world = _yaw_matrix(robot_quat) @ _yaw_matrix(reference_quat).T
            self._reference_origin = reference_pos.copy()
            self._robot_origin = np.asarray([0.0, 0.0, reference_pos[2]], dtype=np.float64)
            self._estimated_robot_pos = self._robot_origin.copy()
            self._alignment_initialized = True
        else:
            dt = min(max(float(context.dt_sec), 0.0), 0.1)
            self._estimated_robot_pos += world_linear_velocity * dt

        aligned_reference_pos = self._robot_origin + self._init_to_world @ (
            reference_pos - self._reference_origin
        )
        anchor_pos_b = robot_rotation.T @ (
            aligned_reference_pos - self._estimated_robot_pos
        )
        anchor_ori_b = (
            robot_rotation.T @ self._init_to_world @ _matrix_from_quat(reference_quat)
        )[:, :2].reshape(-1)
        base_lin_vel = robot_rotation.T @ world_linear_velocity

        joint_states = context.require_joint_states()
        by_name = {state.joint_name: state for state in joint_states.states}
        missing = [name for name in POLICY_JOINT_NAMES if name not in by_name]
        if missing:
            raise ValueError(f"PHP climb joint states missing: {missing}")
        joint_pos = [
            float(by_name[name].position) - DEFAULT_POLICY_JOINT_POS[name]
            for name in POLICY_JOINT_NAMES
        ]
        joint_vel = [float(by_name[name].velocity) for name in POLICY_JOINT_NAMES]

        observation_groups = {
            "command": np.concatenate((motion.joint_pos[step], motion.joint_vel[step])),
            "motion_anchor_pos_b": anchor_pos_b,
            "motion_anchor_ori_b": anchor_ori_b,
            "base_lin_vel": base_lin_vel,
            "base_ang_vel": np.asarray(body_angular_velocity, dtype=np.float64),
            "joint_pos": np.asarray(joint_pos, dtype=np.float64),
            "joint_vel": np.asarray(joint_vel, dtype=np.float64),
            "actions": self._last_action,
        }
        for name, expected_dim in OBS_DIMS.items():
            actual_dim = int(observation_groups[name].size)
            if actual_dim != expected_dim:
                raise ValueError(
                    f"PHP climb observation group {name} dim mismatch: "
                    f"{actual_dim} != {expected_dim}"
                )
        obs = np.concatenate([observation_groups[name] for name in OBS_DIMS])
        if obs.size != 150 or not np.isfinite(obs).all():
            raise ValueError("PHP climb observation must be 150 finite values")

        if step + 1 >= motion.time_step_total:
            self._motion_step = motion.time_step_total - 1
            self._motion_end = True
        else:
            self._motion_step = step + 1
        return {"obs": obs.tolist(), "time_step": [float(step)]}

    def decode_action(
        self, raw_action: Iterable[float], context: PolicyContext
    ) -> List[float]:
        action = np.asarray([float(value) for value in raw_action], dtype=np.float64)
        if action.shape != (27,) or not np.isfinite(action).all():
            raise ValueError("PHP climb raw action must be 27 finite values")
        self._last_action = action.copy()

        target_by_name = self._run_profile_positions(context)
        for index, joint_name in enumerate(POLICY_JOINT_NAMES):
            target_by_name[joint_name] = (
                DEFAULT_POLICY_JOINT_POS[joint_name]
                + self._action_scale * float(action[index])
            )
        return [target_by_name[joint.name] for joint in context.require_joint_configs()]

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
                f"PHP climb motion file not found: {motion_path} "
                f"(expected alongside {model_path})"
            )
        with np.load(str(motion_path), allow_pickle=False) as data:
            joint_names = [str(name) for name in data["joint_names"].tolist()]
            body_names = [str(name) for name in data["body_names"].tolist()]
            fps = float(np.asarray(data["fps"]).reshape(-1)[0])
            joint_pos = np.asarray(data["joint_pos"], dtype=np.float64)
            joint_vel = np.asarray(data["joint_vel"], dtype=np.float64)
            body_pos_w = np.asarray(data["body_pos_w"], dtype=np.float64)
            body_quat_w = np.asarray(data["body_quat_w"], dtype=np.float64)
        if joint_names != MOTION_JOINT_NAMES:
            raise ValueError("PHP climb motion joint order does not match exported policy")
        if "Trunk" not in body_names:
            raise ValueError("PHP climb motion is missing Trunk anchor body")
        if not math.isclose(fps, self._expected_frequency_hz, rel_tol=0.0, abs_tol=1.0e-6):
            raise ValueError(f"PHP climb motion fps mismatch: {fps} != 50")
        if joint_pos.ndim != 2 or joint_pos.shape[1] != 27 or joint_pos.shape != joint_vel.shape:
            raise ValueError("PHP climb motion joint arrays must have shape [frames, 27]")
        if body_pos_w.shape[:2] != body_quat_w.shape[:2]:
            raise ValueError("PHP climb motion body arrays have incompatible shapes")
        if body_pos_w.shape[0] != joint_pos.shape[0] or joint_pos.shape[0] == 0:
            raise ValueError("PHP climb motion frame count is invalid")
        run_positions = self._run_profile_positions(context)
        run_policy_positions = np.asarray(
            [run_positions[name] for name in POLICY_JOINT_NAMES], dtype=np.float64
        )
        if not np.allclose(run_policy_positions, joint_pos[0], rtol=0.0, atol=1.0e-6):
            raise ValueError("PHP climb run profile must equal motion frame-0 joint pose")
        self._motion_data = _MotionData(
            joint_pos=joint_pos,
            joint_vel=joint_vel,
            body_pos_w=body_pos_w,
            body_quat_w=body_quat_w,
            anchor_body_index=body_names.index("Trunk"),
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
        missing = [name for name in POLICY_JOINT_NAMES if name not in positions]
        if missing:
            raise ValueError(f"run profile missing PHP climb joints: {missing}")
        return positions

    @staticmethod
    def _require_vector(values: Iterable[float], size: int, name: str) -> List[float]:
        result = [float(value) for value in values]
        if len(result) != size or not all(math.isfinite(value) for value in result):
            raise ValueError(f"{name} must contain {size} finite values")
        return result


class _MotionData:
    def __init__(
        self,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
        body_pos_w: np.ndarray,
        body_quat_w: np.ndarray,
        anchor_body_index: int,
    ) -> None:
        self.joint_pos = joint_pos
        self.joint_vel = joint_vel
        self.body_pos_w = body_pos_w
        self.body_quat_w = body_quat_w
        self.anchor_body_index = anchor_body_index
        self.time_step_total = int(joint_pos.shape[0])
