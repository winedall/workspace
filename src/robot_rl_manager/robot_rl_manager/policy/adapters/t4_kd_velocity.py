"""T4_27 velocity adapter.

ONNX contract:
obs(92) = base_ang_vel(3) + projected_gravity(3) + command(3) + phase(2)
          + joint_pos_relative(27) + joint_vel(27) + last_action(27)
action(27) -> target_joint_pos = run_profile + 0.25 * raw_action
"""

from __future__ import annotations

import math
from typing import Dict, Iterable, List

from robot_rl_manager.policy.adapter import BasePolicyAdapter
from robot_rl_manager.policy.backend import ModelInput
from robot_rl_manager.policy.context import PolicyContext


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

OBS_DIM = 92
ACTION_DIM = 27
ACTION_SCALE = [0.25] * ACTION_DIM
PHASE_PERIOD_SEC = 0.6
POLICY_DT_SEC = 1.0 / 50.0
STAND_PHASE_COMMAND_NORM = 0.1


class T4KdVelocityPolicyAdapter(BasePolicyAdapter):
    """Velocity-command T4_27 policy with gait phase input."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._last_action = [0.0] * ACTION_DIM
        self._phase_time_sec = 0.0

    def required_inputs(self) -> set[str]:
        return {"joint_states", "imu"}

    def build_model_input(self, context: PolicyContext) -> ModelInput:
        imu = context.require_policy_imu()
        joint_states = context.require_joint_states()

        base_ang_vel = self._fixed_len(imu.angular_velocity, 3, "angular_velocity")
        projected_gravity = self._projected_gravity(imu.orientation_quat)
        command = [
            float(context.cmd_vel.linear.x),
            float(context.cmd_vel.linear.y),
            float(context.cmd_vel.angular.z),
        ]
        phase = self._next_phase(command)

        default_pos = self._run_profile_positions(context)
        states_by_name = {state.joint_name: state for state in joint_states.states}
        joint_pos: List[float] = []
        joint_vel: List[float] = []
        for name in POLICY_JOINT_NAMES:
            state = states_by_name.get(name)
            if state is None:
                joint_pos.append(0.0)
                joint_vel.append(0.0)
                continue
            joint_pos.append(float(state.position) - default_pos[name])
            joint_vel.append(float(state.velocity))

        obs: List[float] = []
        obs.extend(base_ang_vel)
        obs.extend(projected_gravity)
        obs.extend(command)
        obs.extend(phase)
        obs.extend(joint_pos)
        obs.extend(joint_vel)
        obs.extend(self._last_action)
        if len(obs) != OBS_DIM:
            raise ValueError(f"T4_27 velocity obs dim mismatch: {len(obs)} != {OBS_DIM}")
        return {"obs": obs}

    def decode_action(self, raw_action: Iterable[float], context: PolicyContext) -> List[float]:
        action = [float(value) for value in raw_action]
        if len(action) != ACTION_DIM:
            raise ValueError(f"T4_27 velocity raw action dim mismatch: {len(action)} != {ACTION_DIM}")

        default_pos = self._run_profile_positions(context)
        target_by_name = dict(default_pos)
        for index, name in enumerate(POLICY_JOINT_NAMES):
            target_by_name[name] = default_pos[name] + ACTION_SCALE[index] * action[index]

        self._last_action = list(action)
        return [
            float(target_by_name.get(joint.name, 0.0))
            for joint in context.require_joint_configs()
        ]

    def select_joint_profile(self, context: PolicyContext) -> str:
        del context
        return "run"

    def is_motion_end(self) -> bool:
        return False

    def _next_phase(self, command: List[float]) -> List[float]:
        self._phase_time_sec += POLICY_DT_SEC
        command_norm = math.sqrt(sum(value * value for value in command))
        if command_norm <= STAND_PHASE_COMMAND_NORM:
            return [0.0, 0.0]
        phase = (self._phase_time_sec / PHASE_PERIOD_SEC) % 1.0
        return [
            math.sin(2.0 * math.pi * phase),
            math.cos(2.0 * math.pi * phase),
        ]

    def _projected_gravity(self, quaternion_wxyz: Iterable[float]) -> List[float]:
        quaternion = self._fixed_len(quaternion_wxyz, 4, "orientation_quat")
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm <= 1.0e-9:
            raise ValueError("orientation_quat norm is too small")
        w, x, y, z = [value / norm for value in quaternion]
        return [
            2.0 * (w * y - x * z),
            -2.0 * (y * z + w * x),
            2.0 * (x * x + y * y) - 1.0,
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
