"""Z01_std 量产版人形行走策略 adapter。"""

from __future__ import annotations

from collections import deque
import math
from typing import Deque, Dict, Iterable, List
import numpy as np
import cv2

from robot_interfaces.msg import JointStates

from robot_rl_manager.policy.adapter import BasePolicyAdapter
from robot_rl_manager.policy.backend import ModelInput
from robot_rl_manager.policy.context import PolicyContext


AMP_REAL_JOINT_NAMES = [
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
    "waist_pitch",
    "waist_roll",
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

AMP_POLICY_IDX = [
    0,
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    10,
    11,
    12,
    13,
    16,
    17,
    18,
    19,
    20,
    21,
    22,
    23,
    24,
    25,
    26,
    27,
    28,
]

class Z01StdWalkPolicyAdapter(BasePolicyAdapter):
    """z01_std_walk 行走模型的语义适配器。

    该模型输入为 ``obs`` 和 ``hist`` 两个张量：``obs`` 是当前 92 维观测，
    ``hist`` 是按旧 policy 分组方式堆叠的 5 帧历史观测。模型输出 27 维 raw
    action，再映射回 31 维全身关节位置目标。

    27 维 action 的关节顺序取自 ONNX metadata 的 ``joint_names``：
    双臂 14 + ``waist_yaw`` + 双腿 12，即 ``AMP_POLICY_IDX`` 跳过的
    ``waist_pitch``/``waist_roll`` 由 run profile 保持不动。Z01_std 的
    29 个执行器顺序与 ``AMP_REAL_JOINT_NAMES`` 一致，因此两者可直接对齐。
    """

    _frame_stack = 5
    _action_scale = 0.25
    _policy_interval_sec = 0.02
    _period_sec = 0.6
    ONNX_RUNTIME_OPTIONS = {
        "intra_op_num_threads": 2,
        "inter_op_num_threads": 1,
        "execution_mode": "sequential",
    }

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        """重置历史观测、相位、上一帧 action 和速度平滑状态。"""
        self._phase = 0.0
        self._last_action = [0.0] * len(AMP_POLICY_IDX)
        self._smoothed_command = [0.0, 0.0, 0.0]
        self._hist_obs: Dict[str, Deque[List[float]]] = {}
        self._cam_update_counter = 0
        for key, dim in self._obs_group_dims().items():
            queue: Deque[List[float]] = deque(maxlen=self._frame_stack)
            for _ in range(self._frame_stack):
                queue.append([0.0] * dim)
            self._hist_obs[key] = queue

    def required_inputs(self) -> set[str]:
        """walk AMP 模型需要关节状态和 IMU 语义输入。"""
        return {"joint_states", "imu"}

    def build_model_input(self, context: PolicyContext) -> ModelInput:
        """构造 z01_std_walk.onnx 需要的 obs[92] 和 hist[460]。"""
        model_info = context.require_model_info()
        joint_states = context.require_joint_states()
        imu = context.require_policy_imu()
        # if self._cam_update_counter % 5 == 0 and context.depth_image is not None:
            # depth_image = context.depth_image
            # depth_image = depth_image.T
            # depth_image = depth_image / 1000
            # print(depth_image.shape)
            # print(depth_image)
            # depth_image = cv2.resize(depth_image, (64, 36), interpolation=cv2.INTER_NEAREST)
            # depth_image = cv2.flip(depth_image, 0)
            # depth_image = self._process_depth_image(depth_image)
            # cv2.namedWindow('depth image', cv2.WINDOW_NORMAL)
            # cv2.imshow("depth image", depth_image)
            # cv2.waitKey(1)
        self._cam_update_counter += 1
        ang = self._fixed_len(imu.angular_velocity, 3, "angular_velocity")
        grav = self._fixed_len(
            self._compute_projected_gravity(*imu.euler_rad), 3, "projected_gravity"
        )
        cmd = self._update_smoothed_command(
            [
                float(context.cmd_vel.linear.x),
                float(context.cmd_vel.linear.y),
                float(context.cmd_vel.angular.z),
            ]
        )
        qpos_full, qvel_full = self._amp_joint_state_vectors(joint_states)
        reference_qpos = self._amp_reference_qpos(context)
        qpos = [
            qpos_full[index] - reference_qpos[index]
            for index in AMP_POLICY_IDX
        ]
        qvel = [qvel_full[index] for index in AMP_POLICY_IDX]
        action = list(self._last_action)
        phase = self._next_phase(cmd)

        groups = {
            "ang": ang,
            "grav": grav,
            "cmd": cmd,
            "qpos": qpos,
            "qvel": qvel,
            "action": action,
            "phase": phase,
        }
        for key, values in groups.items():
            self._hist_obs[key].append(list(values))

        obs = self._concat_groups(groups)
        hist = self._concat_history()
        self._validate_model_dims(model_info.name, obs, hist)
        return {"obs": obs, "hist": hist}

    def decode_action(self, raw_action: Iterable[float], context: PolicyContext) -> List[float]:
        """把 27 维 raw action 映射成按 robot_joints.yaml 顺序排列的全身关节目标。"""
        action = [float(value) for value in raw_action]
        if len(action) != len(AMP_POLICY_IDX):
            raise ValueError(
                f"walk raw action dim mismatch: {len(action)} != {len(AMP_POLICY_IDX)}"
            )
        self._last_action = list(action)

        profile_positions = self._run_profile_positions(context)
        amp_targets = [profile_positions[joint_name] for joint_name in AMP_REAL_JOINT_NAMES]
        for action_index, real_index in enumerate(AMP_POLICY_IDX):
            amp_targets[real_index] = (
                amp_targets[real_index] + self._action_scale * action[action_index]
            )

        target_by_name = dict(profile_positions)
        for joint_name, target in zip(AMP_REAL_JOINT_NAMES, amp_targets):
            target_by_name[joint_name] = target

        return [
            float(target_by_name.get(joint.name, 0.0))
            for joint in context.require_joint_configs()
        ]

    def select_joint_profile(self, context: PolicyContext) -> str:
        """模型推理命令统一使用 run profile 的增益。"""
        del context
        return "run"

    def is_motion_end(self) -> bool:
        """walk 是连续行走模型，没有参考运动结束的概念。"""
        return False

    @staticmethod
    def _amp_reference_qpos(context: PolicyContext) -> List[float]:
        """从当前模型 run profile 提取 AMP 29 关节参考姿态。"""
        profile_positions = Z01StdWalkPolicyAdapter._run_profile_positions(context)
        return [profile_positions[joint_name] for joint_name in AMP_REAL_JOINT_NAMES]

    @staticmethod
    def _run_profile_positions(context: PolicyContext) -> Dict[str, float]:
        """按 joint_name 返回当前模型 run profile 的全身参考位置。"""
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
        for joint_name in AMP_REAL_JOINT_NAMES:
            if joint_name not in positions:
                raise ValueError(f"run profile missing AMP joint: {joint_name}")
        return positions

    @staticmethod
    def _obs_group_dims() -> Dict[str, int]:
        """返回旧 AMP policy 的历史观测分组维度。"""
        action_dim = len(AMP_POLICY_IDX)
        return {
            "ang": 3,
            "grav": 3,
            "cmd": 3,
            "phase": 2,
            "qpos": action_dim,
            "qvel": action_dim,
            "action": action_dim,
        }

    @staticmethod
    def _fixed_len(values: Iterable[float], expected_len: int, name: str) -> List[float]:
        """把输入转换成固定长度 float list。"""
        result = [float(value) for value in values]
        if len(result) != expected_len:
            raise ValueError(f"{name} dim mismatch: {len(result)} != {expected_len}")
        return result

    def _update_smoothed_command(self, command: List[float]) -> List[float]:
        """按旧 policy 的每周期最大 0.05 步长平滑速度命令。"""
        for index, value in enumerate(command):
            delta = value - self._smoothed_command[index]
            delta = min(0.05, max(-0.05, delta))
            self._smoothed_command[index] += delta
        return list(self._smoothed_command)

    def _next_phase(self, command: List[float]) -> List[float]:
        """更新步态相位；速度命令很小时按旧逻辑输出零相位。"""
        if math.sqrt(command[0] ** 2 + command[1] ** 2) <= 1.5:
            self._period_sec = 0.8
        else:
            self._period_sec = 0.6
        self._phase = (self._phase + self._policy_interval_sec / self._period_sec) % 1.0
        if math.sqrt(sum(value * value for value in command)) <= 0.1:
            return [0.0, 0.0]
        return [
            math.sin(2.0 * math.pi * self._phase),
            math.cos(2.0 * math.pi * self._phase),
        ]

    @staticmethod
    def _amp_joint_state_vectors(joint_states: JointStates) -> tuple[List[float], List[float]]:
        """按旧 AMP policy 的 29 关节顺序提取位置和速度。"""
        by_name = {state.joint_name: state for state in joint_states.states}
        qpos = []
        qvel = []
        for joint_name in AMP_REAL_JOINT_NAMES:
            state = by_name.get(joint_name)
            qpos.append(float(state.position) if state is not None else 0.0)
            qvel.append(float(state.velocity) if state is not None else 0.0)
        return qpos, qvel

    @staticmethod
    def _concat_groups(groups: Dict[str, List[float]]) -> List[float]:
        """按旧 ``cur_input`` 顺序拼接当前观测。"""
        obs: List[float] = []
        for key in ["ang", "grav", "cmd", "qpos", "qvel", "action", "phase"]:
            obs.extend(groups[key])
        return obs

    def _concat_history(self) -> List[float]:
        """按旧 ``policy_input`` 分组顺序拼接历史观测。"""
        hist: List[float] = []
        for key in ["ang", "grav", "cmd", "qpos", "qvel", "action", "phase"]:
            for frame in self._hist_obs[key]:
                hist.extend(frame)
        return hist

    @staticmethod
    def _process_depth_image(depth_image: np.ndarray) -> List[float]:
        # depth_image = depth_image[18:36, 16:48]
        # noise = np.random.normal(0, 0.02, depth_image.shape).astype(np.float32)
        # depth_image = depth_image.astype(np.float32) + noise
        invalid_mask = (depth_image < 0.2).astype(np.uint8)
        if invalid_mask.any():
            depth_image = cv2.inpaint(depth_image, invalid_mask, inpaintRadius=3, flags=cv2.INPAINT_NS)
        depth_image = cv2.GaussianBlur(depth_image, (3, 3), 1)
        depth_image = np.clip(depth_image, 0.0, 2.5)
        depth_image = np.clip(depth_image / 2.5, 0.0, 1.0)
        return depth_image

    @staticmethod
    def _compute_projected_gravity(
        roll_rad: float, pitch_rad: float, yaw_rad: float,
    ) -> list[float]:
        """按 Rz*Ry*Rx 逆旋转方式投影重力向量 [0,0,-1]。"""
        cr = math.cos(roll_rad)
        sr = math.sin(roll_rad)
        cp = math.cos(pitch_rad)
        sp = math.sin(pitch_rad)
        cy = math.cos(yaw_rad)
        sy = math.sin(yaw_rad)

        rx = ((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr))
        ry = ((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp))
        rz = ((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0))

        def _matmul3(lhs, rhs):
            return tuple(
                tuple(
                    lhs[row][0] * rhs[0][col]
                    + lhs[row][1] * rhs[1][col]
                    + lhs[row][2] * rhs[2][col]
                    for col in range(3)
                )
                for row in range(3)
            )

        rotation = _matmul3(_matmul3(rz, ry), rx)
        vector = [0.0, 0.0, -1.0]
        return [
            rotation[0][i] * vector[0]
            + rotation[1][i] * vector[1]
            + rotation[2][i] * vector[2]
            for i in range(3)
        ]

    @staticmethod
    def _validate_model_dims(model_name: str, obs: List[float], hist: List[float]) -> None:
        """校验 AMP walk 模型输入维度。"""
        if len(obs) != 92:
            raise ValueError(f"walk obs dim mismatch for model {model_name}: {len(obs)} != 92")
        if len(hist) != 460:
            raise ValueError(f"walk hist dim mismatch for model {model_name}: {len(hist)} != 460")
