"""lay_up 策略 adapter — 躺起模型的语义适配器。

参考旧 policy_t4_lay_up.py 的观测构造逻辑：
  - 单帧 144 维 obs：command(54) + anchor_ori(6) + ang(3) + qpos(27) + qvel(27) + action(27)
  - 依赖参考运动 .npz 文件
  - 模型原始输出 27 维 raw action，经 POLICY_IDX 映射到 29 全身关节目标
  - 旧框架使用 euler_to_quaternion 转换欧拉角为四元数；新框架直接使用 IMU 四元数
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


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """欧拉角 (roll, pitch, yaw) → 四元数 [x, y, z, w]。

    与旧框架 policy_t4_lay_up.py 保持一致的转换公式。
    新框架中 IMU 直接提供四元数，此函数保留用于参考和调试。
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([x, y, z, w])


# ==========================================
# 关节常量
# ==========================================

LAY_UP_REAL_JOINT_NAMES = [
    # left_arm (7)
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
    "left_wrist_yaw", "left_wrist_pitch", "left_wrist_roll",
    # right_arm (7)
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow",
    "right_wrist_yaw", "right_wrist_pitch", "right_wrist_roll",
    # waist (3)
    "waist_pitch", "waist_roll", "waist_yaw",
    # left_leg (6)
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee",
    "left_ankle_pitch", "left_ankle_roll",
    # right_leg (6)
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
]

OBS_GROUP_DIMS = [54, 6, 3, 27, 27, 27]

# ==========================================
# 插值帧常量（与 policy_t4_lay_up.py.back 中的三阶段插值一致）
# ==========================================

# 250 帧插值，分 3 个阶段逐步过渡到参考运动首帧
_INTERP_TOTAL = 250

# 先在参考首帧预热 action history，再平滑接管策略输出。
# 整个阶段固定 motion_step=0，不会提前消耗 NPZ 参考帧。
_POLICY_WARMUP_FRAMES = 8
_POLICY_BLEND_FRAMES = 20

# 三个阶段的关键姿态（27 关节，与 NPZ 参考运动顺序一致，对应 POLICY_IDX）
# 姿态 1：卧倒起始姿态
_INTERP_POSE_1: List[float] = [
    0.2, 0.15, 0.0, -0.5, 0.0, 0.0, 0.0,
    0.2, -0.15, 0.0, -0.5, 0.0, 0.0, 0.0,
    0.0,
    -0.2, 0.0, 0.0, 0.4, -0.24, 0.0,
    -0.2, 0.0, 0.0, 0.4, -0.24, 0.0,
]
# 姿态 2：中间过渡姿态
_INTERP_POSE_2: List[float] = [
    1.7, 0.2, -0.0, -1.2, -0.0, 0.0, 1.5,
    1.7, -0.2, 0.0, -1.2, 0.0, 0.0, -1.5,
    0.0,
    -1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
]
# 姿态 3：靠近参考运动首帧的过渡姿态
_INTERP_POSE_3: List[float] = [
    1.2, 0.2, -0.0, -0.8, -0.0, 0.0, 0.0,
    1.2, -0.2, 0.0, -0.8, 0.0, 0.0, 0.0,
    0.0,
    -1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
]


# ==========================================
# Adapter
# ==========================================

class T4LayUpPolicyAdapter(BasePolicyAdapter):
    """t4_lay_up.onnx 躺起模型的语义适配器。

    模型输入为单张 144 维 obs 张量，输出 27 维 raw action。
    观测构造依赖参考运动 .npz 文件，需要 IMU 四元数参与 anchor 姿态计算。

    插值帧：在策略推理前，先执行 250 帧三阶段插值，
    从卧倒姿态逐步过渡到参考运动首帧，避免动作突变。
    """

    _action_scale = 0.25
    _action_filter_alpha = 0.8
    _clip_observations = 100.0
    _clip_actions = 100.0

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._motion_step = 0
        self._motion_end = False
        self._init_to_world: np.ndarray = np.eye(3, dtype=np.float64)
        self._last_action = [0.0] * len(POLICY_IDX)
        self._filtered_action: List[float] | None = None
        self._policy_warmup_step = 0
        self._policy_blend_start: List[float] | None = None
        self._policy_blend_goal: List[float] | None = None
        self._policy_blend_step = 0
        self._last_target: List[float] | None = None
        self._obs: Dict[str, List[float]] = {}
        for key, dim in zip(OBS_GROUP_KEYS, OBS_GROUP_DIMS):
            self._obs[key] = [0.0] * dim
        self._motion_data: _MotionData | None = None
        self._interp_step = 0

    def required_inputs(self) -> set[str]:
        return {"joint_states", "imu"}

    def build_model_input(self, context: PolicyContext) -> ModelInput:
        motion = self._load_motion(context)

        command_pos = motion.joint_pos[self._motion_step].tolist()
        command_vel = motion.joint_vel[self._motion_step].tolist()
        self._obs["command"] = command_pos + command_vel  # 54

        imu = context.require_policy_imu()
        q_robot = imu.orientation_quat
        q_ref = motion.body_quat_w[self._motion_step, 0].tolist()

        if self._motion_step == 0:
            init_to_anchor = matrix_from_quat(yaw_quat(q_ref))
            world_to_anchor = matrix_from_quat(yaw_quat(q_robot))
            self._init_to_world = world_to_anchor @ init_to_anchor.T

        ori_b_matrix = matrix_from_quat(
            q_robot).T @ self._init_to_world @ matrix_from_quat(q_ref)
        self._obs["motion_anchor_ori_b"] = ori_b_matrix[:,
                                                        # 6
                                                        :2].reshape(-1).tolist()

        self._obs["base_ang_vel"] = [float(v)
                                     for v in imu.angular_velocity]  # 3

        joint_states = context.require_joint_states()
        by_name = {s.joint_name: s for s in joint_states.states}
        default_pos = self._run_profile_positions(context)
        qpos_29 = []
        qvel_29 = []
        for name in LAY_UP_REAL_JOINT_NAMES:
            state = by_name.get(name)
            qpos_29.append(float(state.position) if state is not None else 0.0)
            qvel_29.append(float(state.velocity) if state is not None else 0.0)
        self._obs["joint_pos"] = [
            qpos_29[idx] - default_pos[LAY_UP_REAL_JOINT_NAMES[idx]]
            for idx in POLICY_IDX
        ]  # 27
        self._obs["joint_vel"] = [qvel_29[idx] for idx in POLICY_IDX]  # 27

        self._obs["actions"] = list(self._last_action)  # 27

        obs: List[float] = []
        for key in OBS_GROUP_KEYS:
            obs.extend(self._obs[key])
        self._validate_dims(obs)

        obs = [
            float(max(-self._clip_observations, min(self._clip_observations, v)))
            for v in obs
        ]

        if self._interp_step < _INTERP_TOTAL:
            self._interp_step += 1
        else:
            # 第 250 帧已经由 decode_action 下发参考首帧；之后固定标记为策略阶段。
            self._interp_step = _INTERP_TOTAL + 1

        return {"obs": obs}

    def decode_action(
        self, raw_action: Iterable[float], context: PolicyContext
    ) -> List[float]:
        if self._interp_step <= _INTERP_TOTAL:
            target = self._interp_target(context)
            self._policy_warmup_step = 0
            self._policy_blend_start = list(target)
            self._policy_blend_goal = None
            self._policy_blend_step = 0
            self._last_target = list(target)
            return target

        action = [float(v) for v in raw_action]
        if len(action) != len(POLICY_IDX):
            raise ValueError(
                f"LAY_UP raw action dim mismatch: {len(action)} != {len(POLICY_IDX)}"
            )

        if not all(math.isfinite(v) for v in action):
            if self._last_target is not None:
                return list(self._last_target)
            profile_positions = self._run_profile_positions(context)
            return [float(profile_positions[joint.name])
                    for joint in context.require_joint_configs()]

        profile_positions = self._run_profile_positions(context)
        if self._policy_blend_goal is not None:
            # 接管目标一旦建立便保持不变，避免最后一帧追赶不断漂移的策略输出。
            raw_target = list(self._policy_blend_goal)
        else:
            action = [
                float(max(-self._clip_actions, min(self._clip_actions, v)))
                for v in action
            ]
            action = self._filter_action(action)

            if self._policy_warmup_step < _POLICY_WARMUP_FRAMES:
                # 只预热模型的上一帧 action 观测，关节目标和参考相位均保持不动。
                self._last_action = list(action)
                self._policy_warmup_step += 1
                if self._last_target is not None:
                    return list(self._last_target)
                return self._interp_target(context)

            amp_targets = [float(profile_positions[name])
                           for name in LAY_UP_REAL_JOINT_NAMES]
            for action_idx, real_idx in enumerate(POLICY_IDX):
                amp_targets[real_idx] += self._action_scale * action[action_idx]

            target_by_name = dict(profile_positions)
            for joint_name, position in zip(LAY_UP_REAL_JOINT_NAMES, amp_targets):
                target_by_name[joint_name] = position

            raw_target = [
                float(target_by_name.get(joint.name, 0.0))
                for joint in context.require_joint_configs()
            ]
            self._last_action = list(action)
        target = self._blend_policy_target(raw_target)
        self._last_target = list(target)
        # 接管期间保持参考第 0 帧；完全接管后才开始消耗 NPZ 运动帧。
        if self._policy_blend_start is None:
            self._advance_motion(context)
        return target

    def select_joint_profile(self, context: PolicyContext) -> str:
        del context
        return "run"

    def is_motion_end(self) -> bool:
        return self._motion_end

    # ---- 内部辅助 -----------------------------------------------------------

    def _interp_target(self, context: PolicyContext) -> List[float]:
        """三阶段插值：qw1→qw2(0-99) → qw2→qw3(100-149) → qw3→qw5(150-249)。

        qw5 = 参考运动首帧 joint_pos[0]，qw1/qw2/qw3 为硬编码关键姿态。
        """
        motion = self._load_motion(context)
        qw5 = motion.joint_pos[0].tolist()

        # 复刻原策略的 1-based 计数；第 250 帧必须精确落到 qw5。
        step = self._interp_step
        if step < 100:
            t = step / 100.0
            target = [
                (1.0 - t) * p1 + t * p2
                for p1, p2 in zip(_INTERP_POSE_1, _INTERP_POSE_2)
            ]
        elif step < 150:
            t = (step - 100) / 50.0
            target = [
                (1.0 - t) * p2 + t * p3
                for p2, p3 in zip(_INTERP_POSE_2, _INTERP_POSE_3)
            ]
        elif step < _INTERP_TOTAL:
            t = (step - 150) / 100.0
            target = [
                (1.0 - t) * p3 + t * p5
                for p3, p5 in zip(_INTERP_POSE_3, qw5)
            ]
        else:
            target = qw5

        assert len(target) == len(qw5), f"target({len(target)}) != qw5({len(qw5)})"
        amp_targets = [0.0] * len(LAY_UP_REAL_JOINT_NAMES)
        for action_idx, real_idx in enumerate(POLICY_IDX):
            amp_targets[real_idx] = float(target[action_idx])
        target_by_name = dict(zip(LAY_UP_REAL_JOINT_NAMES, amp_targets))
        return [
            float(target_by_name.get(joint.name, 0.0))
            for joint in context.require_joint_configs()
        ]

    def _filter_action(self, action: List[float]) -> List[float]:
        """对模型 action 做一阶低通滤波，抑制相邻帧高频跳变。"""
        alpha = float(max(0.0, min(1.0, self._action_filter_alpha)))
        if self._filtered_action is None:
            self._filtered_action = list(action)
        else:
            self._filtered_action = [
                alpha * current + (1.0 - alpha) * previous
                for current, previous in zip(action, self._filtered_action)
            ]
        return list(self._filtered_action)

    def _blend_policy_target(self, raw_target: List[float]) -> List[float]:
        """从插值末帧平滑接管到策略在参考首帧给出的固定目标。"""
        if self._policy_blend_start is None or _POLICY_BLEND_FRAMES <= 0:
            return raw_target
        if self._policy_blend_step >= _POLICY_BLEND_FRAMES:
            self._policy_blend_start = None
            self._policy_blend_goal = None
            return raw_target

        if self._policy_blend_goal is None:
            self._policy_blend_goal = list(raw_target)
        goal = self._policy_blend_goal

        t = (self._policy_blend_step + 1) / float(_POLICY_BLEND_FRAMES)
        alpha = t * t * (3.0 - 2.0 * t)
        target = [
            (1.0 - alpha) * start + alpha * destination
            for start, destination in zip(self._policy_blend_start, goal)
        ]
        self._policy_blend_step += 1
        if self._policy_blend_step >= _POLICY_BLEND_FRAMES:
            self._policy_blend_start = None
            self._policy_blend_goal = None
        return target

    def _advance_motion(self, context: PolicyContext) -> None:
        """策略完整接管后，将参考运动推进一帧。"""
        motion = self._load_motion(context)
        motion_step_next = self._motion_step + 1
        if motion_step_next >= motion.time_step_total:
            self._motion_step = motion.time_step_total - 1
            self._motion_end = True
        else:
            self._motion_step = motion_step_next

    def _load_motion(self, context: PolicyContext) -> _MotionData:
        if self._motion_data is not None:
            return self._motion_data
        model_info = context.require_model_info()
        model_path = Path(model_info.file)
        motion_path = model_path.with_suffix(".npz")
        if not motion_path.exists():
            raise FileNotFoundError(
                f"LAY_UP motion file not found: {motion_path} "
                f"(expected alongside {model_path})"
            )
        data = np.load(str(motion_path))
        self._motion_data = _MotionData(
            joint_pos=np.asarray(data["joint_pos"], dtype=np.float64),
            joint_vel=np.asarray(data["joint_vel"], dtype=np.float64),
            body_quat_w=np.asarray(
                data["body_quat_w"], dtype=np.float64)[:, [0]],
            time_step_total=int(data["joint_pos"].shape[0]),
        )
        return self._motion_data

    @staticmethod
    def _run_profile_positions(context: PolicyContext) -> Dict[str, float]:
        model_info = context.require_model_info()
        profile = model_info.joint_profiles.get("run")
        if profile is None:
            raise ValueError(
                f"model {model_info.name} missing run joint profile")
        positions: Dict[str, float] = {}
        for joint in context.require_joint_configs():
            profile_joint = profile.joints.get(joint.name)
            if profile_joint is None:
                raise ValueError(f"run profile missing joint: {joint.name}")
            positions[joint.name] = float(profile_joint.position)
        for joint_name in LAY_UP_REAL_JOINT_NAMES:
            if joint_name not in positions:
                raise ValueError(
                    f"run profile missing LAY_UP joint: {joint_name}")
        return positions

    @staticmethod
    def _validate_dims(obs: List[float]) -> None:
        if len(obs) != 144:
            raise ValueError(
                f"LAY_UP obs dim mismatch: {len(obs)} != 144"
            )


# ==========================================
# 内部数据类型
# ==========================================

class _MotionData:
    """参考运动数据容器。"""

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
