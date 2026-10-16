"""D4 getup policy adapter."""

from __future__ import annotations

from typing import Dict, Iterable, List

import numpy as np

from robot_rl_manager.policy.adapter import BasePolicyAdapter
from robot_rl_manager.policy.backend import ModelInput
from robot_rl_manager.policy.context import PolicyContext


D4_JOINT_NAMES = [
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
]

JOINT_MAPPING = np.array([3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8], dtype=np.int32)
INVERSE_JOINT_MAPPING = np.argsort(JOINT_MAPPING).tolist()

_ANG_VEL_SCALE = 0.25
_DOF_POS_SCALE = 1.0
_DOF_VEL_SCALE = 0.05
_COMMANDS_SCALE = np.asarray([1.0, 1.0, 1.0], dtype=np.float32)

_ACTION_DIM = 12
_OBSERVATION_DIM = 45
_ACTION_SCALE = np.asarray([0.125, 0.25, 0.25,
                             0.125, 0.25, 0.25,
                             0.125, 0.25, 0.25,
                             0.125, 0.25, 0.25], dtype=np.float32)
_ACTION_LOWER = np.full(_ACTION_DIM, -100.0, dtype=np.float32)
_ACTION_UPPER = np.full(_ACTION_DIM, 100.0, dtype=np.float32)
_CLIP_OBS = 100.0


class D4GetupPolicyAdapter(BasePolicyAdapter):
    """Build D4 getup policy's 45-D observation and decode 12-D action."""

    _observation_dim = 45
    _action_dim = _ACTION_DIM

    def reset(self) -> None:
        self._last_action = np.zeros(_ACTION_DIM, dtype=np.float32)

    def required_inputs(self) -> set[str]:
        return {"joint_states", "imu"}

    def build_model_input(self, context: PolicyContext) -> ModelInput:
        joint_states = context.require_joint_states()
        imu = context.require_policy_imu()
        by_name = {state.joint_name: state for state in joint_states.states}
        missing = [name for name in D4_JOINT_NAMES if name not in by_name]
        if missing:
            raise ValueError(f"getup joint states missing: {missing}")

        default_q = self._default_q(context)

        q_physical = np.asarray(
            [float(by_name[name].position) for name in D4_JOINT_NAMES],
            dtype=np.float32,
        )
        dq_physical = np.asarray(
            [float(by_name[name].velocity) for name in D4_JOINT_NAMES],
            dtype=np.float32,
        )

        q_policy = self._apply_joint_mapping(q_physical)
        dq_policy = self._apply_joint_mapping(dq_physical)
        default_q_policy = self._apply_joint_mapping(default_q)

        angular_velocity = np.asarray(imu.angular_velocity, dtype=np.float32)
        if angular_velocity.shape != (3,):
            raise ValueError("getup IMU angular velocity must be 3-D")

        commands = np.asarray(
            [
                float(context.cmd_vel.linear.x),
                float(context.cmd_vel.linear.y),
                float(context.cmd_vel.angular.z),
            ],
            dtype=np.float32,
        )

        gravity = self._projected_gravity(imu.orientation_quat)
        q_rel_policy = q_policy - default_q_policy

        observation = np.concatenate(
            [
                angular_velocity * _ANG_VEL_SCALE,
                gravity,
                commands * _COMMANDS_SCALE,
                q_rel_policy * _DOF_POS_SCALE,
                dq_policy * _DOF_VEL_SCALE,
                self._last_action,
            ]
        )
        if observation.shape != (_OBSERVATION_DIM,):
            raise ValueError(
                f"getup observation dim mismatch: "
                f"{observation.size} != {_OBSERVATION_DIM}"
            )
        observation = np.clip(
            observation, -_CLIP_OBS, _CLIP_OBS
        ).astype(np.float32)

        return {"obs": observation.tolist()}

    def decode_action(
        self,
        raw_action: Iterable[float],
        context: PolicyContext,
    ) -> List[float]:
        action = np.asarray(list(raw_action), dtype=np.float32).reshape(-1)
        if action.shape != (_ACTION_DIM,):
            raise ValueError(
                f"getup raw action dim mismatch: "
                f"{action.size} != {_ACTION_DIM}"
            )
        action = np.clip(action, _ACTION_LOWER, _ACTION_UPPER)
        self._last_action = action.copy()

        action_physical = self._apply_inverse_joint_mapping(action)

        default_q = self._default_q(context)
        target_q = default_q + action_physical * _ACTION_SCALE

        target_by_name = dict(zip(D4_JOINT_NAMES, target_q.tolist()))
        joint_configs = context.require_joint_configs()
        configured_names = [joint.name for joint in joint_configs]
        if set(configured_names) != set(D4_JOINT_NAMES):
            raise ValueError(
                "getup requires exactly the 12 D4 joints; configured: "
                f"{configured_names}"
            )
        return [float(target_by_name[joint.name]) for joint in joint_configs]

    def select_joint_profile(self, context: PolicyContext) -> str:
        del context
        return "run"

    @staticmethod
    def _apply_joint_mapping(arr: np.ndarray) -> np.ndarray:
        return arr[JOINT_MAPPING]

    @staticmethod
    def _apply_inverse_joint_mapping(arr: np.ndarray) -> np.ndarray:
        return arr[INVERSE_JOINT_MAPPING]

    @staticmethod
    def _projected_gravity(quaternion_wxyz: Iterable[float]) -> np.ndarray:
        quaternion = np.asarray(
            list(quaternion_wxyz), dtype=np.float32
        ).reshape(-1)
        if quaternion.shape != (4,):
            raise ValueError("getup IMU orientation quaternion must be 4-D")
        norm = float(np.linalg.norm(quaternion))
        if norm < 1.0e-9:
            quaternion = np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        else:
            quaternion = quaternion / norm

        w, x, y, z = quaternion
        return np.asarray(
            [
                2.0 * (w * y - x * z),
                -2.0 * (y * z + w * x),
                2.0 * (x * x + y * y) - 1.0,
            ],
            dtype=np.float32,
        )

    def _default_q(self, context: PolicyContext) -> np.ndarray:
        positions = self._run_profile_positions(context)
        return np.asarray(
            [positions[name] for name in D4_JOINT_NAMES],
            dtype=np.float32,
        )

    @staticmethod
    def _run_profile_positions(context: PolicyContext) -> Dict[str, float]:
        model_info = context.require_model_info()
        profile = model_info.joint_profiles.get("run")
        if profile is None:
            raise ValueError(
                f"model {model_info.name} missing run joint profile"
            )
        positions = {
            name: float(joint.position) for name, joint in profile.joints.items()
        }
        missing = [name for name in D4_JOINT_NAMES if name not in positions]
        if missing:
            raise ValueError(f"getup run profile missing joints: {missing}")
        return positions
