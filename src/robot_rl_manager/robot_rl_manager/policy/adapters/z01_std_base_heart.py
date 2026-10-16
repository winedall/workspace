"""z01_std_base_heart 策略 adapter —— Z01「比心」动作模仿模型。

  - 单帧 144 维 obs：
      command(54) + motion_anchor_ori_b(6) + base_ang_vel(3)
      + joint_pos(27) + joint_vel(27) + actions(27)
  - 模型输出 27 维 raw action；head_pitch/head_yaw 已被训练侧 free，
    部署时保持 run profile 参考位置（0.0），不参与策略
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


# Z01_std_add_head 的关节顺序（= 模型 action 顺序 = npz joint_pos 列顺序）
Z01_JOINT_NAMES = [
    # left_arm (7)
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
    "left_wrist_yaw", "left_wrist_pitch", "left_wrist_roll",
    # right_arm (7)
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow",
    "right_wrist_yaw", "right_wrist_pitch", "right_wrist_roll",
    # waist_yaw (1)
    "waist_yaw",
    # left_leg (6)
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee",
    "left_ankle_pitch", "left_ankle_roll",
    # right_leg (6)
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee",
    "right_ankle_pitch", "right_ankle_roll",

]

OBS_GROUP_KEYS = [
    "command",
    "motion_anchor_ori_b",
    "base_ang_vel",
    "joint_pos",
    "joint_vel",
    "actions",
]
OBS_GROUP_DIMS = [54, 6, 3, 27, 27, 27]   # command=27+27（npz 已是 27 列）
OBS_DIM = sum(OBS_GROUP_DIMS)             # 144
ACTION_DIM = len(Z01_JOINT_NAMES)         # 27



class Z01StdBaseHeartPolicyAdapter(BasePolicyAdapter):
    """z01_std_base_heart.onnx 的语义适配器（obs[154] -> action[29]）。"""

    _action_scale = 0.25

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._motion_step = 0
        self._motion_end = False
        self._init_to_world: np.ndarray = np.eye(3, dtype=np.float64)
        self._last_action = [0.0] * ACTION_DIM
        self._obs: Dict[str, List[float]] = {
            key: [0.0] * dim for key, dim in zip(OBS_GROUP_KEYS, OBS_GROUP_DIMS)
        }
        self._motion_data: "_MotionData | None" = None

    def required_inputs(self) -> set[str]:
        return {"joint_states", "imu"}

    def is_motion_end(self) -> bool:
        """参考运动播放完毕时返回 True（RL manager 据此回 home）。"""
        return self._motion_end

    def select_joint_profile(self, context: PolicyContext) -> str:
        del context
        return "run"

    def build_model_input(self, context: PolicyContext) -> ModelInput:
        motion = self._load_motion(context)

        # 1) command = 参考关节位置(29) + 参考关节速度(29)
        ref_pos = motion.joint_pos[self._motion_step].tolist()
        ref_vel = motion.joint_vel[self._motion_step].tolist()
        self._obs["command"] = ref_pos + ref_vel  # 58

        # 2) motion_anchor_ori_b —— 机器人 anchor 系下的参考 anchor 姿态
        imu = context.require_policy_imu()
        q_robot = imu.orientation_quat
        q_ref = motion.body_quat_w[self._motion_step, 0].tolist()
        if self._motion_step == 0:
            init_to_anchor = matrix_from_quat(yaw_quat(q_ref))
            world_to_anchor = matrix_from_quat(yaw_quat(q_robot))
            self._init_to_world = world_to_anchor @ init_to_anchor.T
        ori_b = matrix_from_quat(q_robot).T @ self._init_to_world @ matrix_from_quat(q_ref)
        self._obs["motion_anchor_ori_b"] = ori_b[:, :2].reshape(-1).tolist()  # 6

        # 3) base_ang_vel —— IMU 角速度
        self._obs["base_ang_vel"] = [float(v) for v in imu.angular_velocity]  # 3

        # 4/5) joint_pos_rel / joint_vel
        joint_states = context.require_joint_states()
        by_name = {state.joint_name: state for state in joint_states.states}
        default_pos = self._run_profile_positions(context)
        missing = [n for n in Z01_JOINT_NAMES if n not in by_name]
        if missing:
            raise ValueError(f"z01_std_base_heart joint states missing: {missing}")
        self._obs["joint_pos"] = [
            float(by_name[n].position) - default_pos[n] for n in Z01_JOINT_NAMES
        ]  # 29
        self._obs["joint_vel"] = [float(by_name[n].velocity) for n in Z01_JOINT_NAMES]  # 29

        # 6) actions —— 上一帧模型输出
        self._obs["actions"] = list(self._last_action)  # 29

        obs: List[float] = []
        for key in OBS_GROUP_KEYS:
            obs.extend(self._obs[key])
        if len(obs) != OBS_DIM:
            raise ValueError(f"z01_std_base_heart obs dim mismatch: {len(obs)} != {OBS_DIM}")

        # 推进参考运动步；到末尾后保持并标记结束
        next_step = self._motion_step + 1
        if next_step >= motion.time_step_total:
            self._motion_step = motion.time_step_total - 1
            self._motion_end = True
        else:
            self._motion_step = next_step

        return {"obs": obs}

    def decode_action(
        self, raw_action: Iterable[float], context: PolicyContext
    ) -> List[float]:
        action = [float(v) for v in raw_action]
        if len(action) != ACTION_DIM:   # ACTION_DIM 现在是 27
            raise ValueError(
                f"z01_std_base_heart action dim mismatch: {len(action)} != {ACTION_DIM}"
            )
        self._last_action = list(action)

        default_pos = self._run_profile_positions(context)
        target_by_name: Dict[str, float] = dict(default_pos)
        for idx, name in enumerate(Z01_JOINT_NAMES):   # 只遍历 27 个策略关节
            target_by_name[name] = default_pos[name] + self._action_scale * action[idx]

        # head_pitch/head_yaw 不在 Z01_JOINT_NAMES 里，保持 run profile 参考位置（0.0）
        # 部署侧多出的 waist_pitch/waist_roll 同理
        return [
            float(target_by_name.get(joint.name, default_pos.get(joint.name, 0.0)))
            for joint in context.require_joint_configs()
        ]


    # ---- 内部辅助 -----------------------------------------------------------

    def _load_motion(self, context: PolicyContext) -> "_MotionData":
        """从 .onnx 同目录加载同名 .npz 参考运动（首次调用惰性加载）。"""
        if self._motion_data is not None:
            return self._motion_data
        model_info = context.require_model_info()
        model_path = Path(model_info.file)
        motion_path = model_path.with_suffix(".npz")
        if not motion_path.exists():
            raise FileNotFoundError(
                f"z01_std_base_heart motion file not found: {motion_path} "
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
        for name in Z01_JOINT_NAMES:
            if name not in positions:
                raise ValueError(f"run profile missing model joint: {name}")
        return positions


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
