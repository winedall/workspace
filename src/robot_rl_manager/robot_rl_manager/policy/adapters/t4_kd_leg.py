"""LEG策略 adapter。"""

from __future__ import annotations

from collections import deque
import math
from typing import Deque, Dict, Iterable, List

from robot_interfaces.msg import JointStates

from robot_rl_manager.policy.adapter import BasePolicyAdapter
from robot_rl_manager.policy.backend import ModelInput
from robot_rl_manager.policy.context import PolicyContext


LEG_ACTION_JOINT_NAMES = [
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
LEG_POLICY_JOINT_NAMES = [
        # left arm
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_yaw",
    "left_wrist_pitch",
    "left_wrist_roll",

    # right arm
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
    "right_wrist_yaw",
    "right_wrist_pitch",
    "right_wrist_roll",

    "waist_yaw",


    # legs: action 直接控制的 12 个关节
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

LEG_DEFAULT_DOF_POS = [
        # left arm
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    # right arm
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    # waist_yaw
    0.0,
    # left leg
    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
    # right leg
    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
]
class T4KdLegPolicyAdapter(BasePolicyAdapter):
    _frame_stack = 6  # LEG 模型需要 6 帧历史观测。
    _frame_dim = 76  # 每帧观测的维度
    _input_dim = 456   # 模型输入的维度
    _action_dim = 12   # 模型输出的维度
    _action_scale = 0.25  # LEG 模型的 action 缩放因子
    _min_height = 0.40
    _max_height = 0.80
    _default_height = 0.80  # 默认高度
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        """重置 6 帧历史观测和上一帧 12 维腿部 action。"""
        self._last_action = [0.0] * len(LEG_ACTION_JOINT_NAMES)
        self._obs_history: Deque[List[float]] = deque(maxlen=self._frame_stack)
        for _ in range(self._frame_stack):
            self._obs_history.append([0.0] * self._frame_dim)

    def required_inputs(self) -> set[str]:
        """模型需要关节状态和 IMU 语义输入。"""
        return {"joint_states", "imu"}

    def build_model_input(self, context: PolicyContext) -> ModelInput:
        """构造 leg.onnx 需要的 input[456]。"""
        model_info = context.require_model_info()
        joint_states = context.require_joint_states()
        imu = context.require_policy_imu()
        # 模型读取速度指令,并加入限制
        x = max(-0.8, min(1.2, float(context.cmd_vel.linear.x)))
        y = max(-0.5, min(0.5, float(context.cmd_vel.linear.y)))
        yaw = max(-0.8, min(0.8, float(context.cmd_vel.angular.z)))

        height_raw = float(context.cmd_vel.linear.z)
        height = (
            self._default_height
            if abs(height_raw) < 1e-6
            else max(self._min_height, min(self._max_height, height_raw))
        )

        cmd = [
            x * 2.0,
            y * 2.0,
            yaw * 0.25,
            height,
        ]
        ang = [
            value * 0.5
            for value in self._fixed_len(imu.angular_velocity, 3, "base_ang_vel_radps")
        ]
        grav = self._fixed_len(self._compute_projected_gravity(*imu.euler_rad), 3, "projected_gravity")

        qpos_full, qvel_full = self._policy_joint_state_vectors(joint_states)
        reference_qpos = self._policy_reference_qpos()
        qpos = [
            position - reference
            for position, reference in zip(qpos_full, reference_qpos)
        ]
        qvel = [velocity * 0.05 for velocity in qvel_full]
        action = list(self._last_action)

        obs: List[float] = []
        obs.extend(cmd) # 速度指令
        obs.extend(ang) # 角速度
        obs.extend(grav) # 重力方向
        obs.extend(qpos) # 关节位置
        obs.extend(qvel) # 关节速度
        obs.extend(action) # 上一个关节腿部动作

        self._validate_frame_dim(model_info.name, obs)
        self._obs_history.append(obs)
        obs_history = self._concat_history()
        self._validate_input_dim(model_info.name, obs_history)
        return {"input": obs_history}

    def decode_action(self, raw_action: Iterable[float], context: PolicyContext) -> List[float]:
        """把 12 维腿部 raw action 映射成按 robot_joints.yaml 顺序排列的全身关节目标。"""
        action = [float(value) for value in raw_action]
        if len(action) != len(LEG_ACTION_JOINT_NAMES):
            raise ValueError(
                f"leg raw action dim mismatch: {len(action)} != {len(LEG_ACTION_JOINT_NAMES)}"
            )
        self._last_action = list(action)

        profile_positions = self._run_profile_positions(context)
        target_by_name = dict(profile_positions)

        for joint_name, value in zip(LEG_ACTION_JOINT_NAMES, action):
            target_by_name[joint_name] = (
                profile_positions[joint_name] + self._action_scale * value #默认角度加上scale乘模型输出
            )

        return [
            float(target_by_name.get(joint.name, 0.0))
            for joint in context.require_joint_configs()
        ]

    def select_joint_profile(self, context: PolicyContext) -> str:
        """模型推理命令统一使用 run profile 的增益。"""
        del context
        return "run"

    def is_motion_end(self) -> bool:
        """LEG是连续行走模型,没有参考运动结束的概念。"""
        return False



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
        for joint_name in LEG_ACTION_JOINT_NAMES:
            if joint_name not in positions:
                raise ValueError(f"run profile missing LEG joint: {joint_name}")
        return positions



    @staticmethod
    def _fixed_len(values: Iterable[float], expected_len: int, name: str) -> List[float]:
        """把输入转换成固定长度 float list。"""
        result = [float(value) for value in values]
        if len(result) != expected_len:
            raise ValueError(f"{name} dim mismatch: {len(result)} != {expected_len}")
        return result


    @staticmethod
    def _policy_reference_qpos() -> List[float]:
        """返回 LEG 训练/部署使用的 27 维默认关节位姿。"""
        return list(LEG_DEFAULT_DOF_POS)

    @staticmethod
    def _policy_joint_state_vectors(joint_states: JointStates) -> tuple[List[float], List[float]]:
        """按 LEG policy 的 27 维关节顺序提取位置和速度。"""
        by_name = {state.joint_name: state for state in joint_states.states}
        qpos = []
        qvel = []
        for joint_name in LEG_POLICY_JOINT_NAMES:
            state = by_name.get(joint_name)
            qpos.append(float(state.position) if state is not None else 0.0)
            qvel.append(float(state.velocity) if state is not None else 0.0)
        return qpos, qvel

    def _concat_history(self) -> List[float]:
        """按 HistoryWrapper 方式拼接 6 帧完整观测。"""
        history: List[float] = []
        for frame in self._obs_history:
            history.extend(frame)
        return history

    @staticmethod
    def _validate_frame_dim(model_name: str, obs: List[float]) -> None:
        if len(obs) != 76:
            raise ValueError(f"leg frame obs dim mismatch for model {model_name}: {len(obs)} != 76")

    @staticmethod
    def _validate_input_dim(model_name: str, obs_history: List[float]) -> None:
        if len(obs_history) != 456:
            raise ValueError(
                f"leg input dim mismatch for model {model_name}: {len(obs_history)} != 456"
            )
            
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
