"""D4 flat himloco lab policy adapter.

关键差异:
  1. 历史帧输入: 6 帧 × 45 维 → ONNX 输入 270 维
  2. 观测顺序: commands → ang_vel → gravity → dof_pos → dof_vel → actions
  3. 关节映射: joint_mapping=[0,3,6,9,1,4,7,10,2,5,8,11]
  4. commands_scale: [1.0, 1.0, 1.0]
  5. default_dof_pos: [hip=0, thigh=0.8, calf=-1.5]

调试模式: D4_HIM_DEBUG=1
"""

import os
from collections import deque
from typing import Dict, Iterable, List

import numpy as np

from robot_rl_manager.policy.adapter import BasePolicyAdapter
from robot_rl_manager.policy.backend import ModelInput
from robot_rl_manager.policy.context import PolicyContext

# 调试开关
_DEBUG = os.environ.get("D4_HIM_DEBUG", "0") == "1"
_call_count = 0  # 用于追踪调用次数


# 关节顺序: [FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf,
#             RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf]
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

# 关节映射: 物理顺序 → 策略网络期望顺序
# 物理: [FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf, RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf]
# 策略: [FL_hip, FR_hip, RL_hip, RR_hip, FL_thigh, FR_thigh, RL_thigh, RR_thigh, FL_calf, FR_calf, RL_calf, RR_calf]
JOINT_MAPPING = np.array([0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11], dtype=np.int32)

# 逆关节映射: 策略网络输出顺序 → 物理顺序
INVERSE_JOINT_MAPPING = np.argsort(JOINT_MAPPING).tolist()

# 历史帧配置
_HISTORY_FRAMES = 6           # 当前帧 + 5 步历史
_SINGLE_FRAME_DIM = 45        # 单帧观测维度

# 缩放系数 (来自 himloco_lab config.yaml)
_ANG_VEL_SCALE = 0.25
_DOF_POS_SCALE = 1.0
_DOF_VEL_SCALE = 0.05
_COMMANDS_SCALE = np.asarray([1.5, 1.0, 0.5], dtype=np.float32)

# 动作缩放和裁剪
_ACTION_DIM = 12
_ACTION_SCALE = np.full(_ACTION_DIM, 0.25, dtype=np.float32)
_ACTION_LOWER = np.full(_ACTION_DIM, -100.0, dtype=np.float32)
_ACTION_UPPER = np.full(_ACTION_DIM, 100.0, dtype=np.float32)
_CLIP_OBS = 100.0



class D4FlatPolicyAdapter(BasePolicyAdapter):
    """Build D4 himloco lab policy's 270-D input (6 frames × 45) and decode 12-D action."""

    _observation_dim = _SINGLE_FRAME_DIM        # 单帧 45 维
    _action_dim = _ACTION_DIM                     # 12 维

    def __init__(self) -> None:
        global _call_count
        _call_count = 0
        if _DEBUG:
            print("\n" + "=" * 70)
            print("[D4_HIM_DEBUG] D4FlatPolicyAdapter 初始化")
            print(f"[D4_HIM_DEBUG] 历史帧数: {_HISTORY_FRAMES}")
            print(f"[D4_HIM_DEBUG] 单帧维度: {_SINGLE_FRAME_DIM}")
            print(f"[D4_HIM_DEBUG] 总输入维度: {_HISTORY_FRAMES * _SINGLE_FRAME_DIM}")
            print(f"[D4_HIM_DEBUG] 动作维度: {_ACTION_DIM}")
            print(f"[D4_HIM_DEBUG] 缩放系数:")
            print(f"[D4_HIM_DEBUG]   - ang_vel_scale: {_ANG_VEL_SCALE}")
            print(f"[D4_HIM_DEBUG]   - dof_pos_scale: {_DOF_POS_SCALE}")
            print(f"[D4_HIM_DEBUG]   - dof_vel_scale: {_DOF_VEL_SCALE}")
            print(f"[D4_HIM_DEBUG]   - commands_scale: {_COMMANDS_SCALE}")
            print(f"[D4_HIM_DEBUG]   - action_scale: {_ACTION_SCALE}")
            print(f"[D4_HIM_DEBUG]   - clip_obs: {_CLIP_OBS}")
            print(f"[D4_HIM_DEBUG]   - clip_actions: [{-100.0}, {100.0}]")
            print(f"[D4_HIM_DEBUG] 关节映射 (物理→策略): {JOINT_MAPPING.tolist()}")
            print(f"[D4_HIM_DEBUG] 逆关节映射 (策略→物理): {INVERSE_JOINT_MAPPING}")
            print("=" * 70 + "\n")
        self.reset()

    def reset(self) -> None:
        if _DEBUG:
            print("\n" + "-" * 50)
            print("[D4_HIM_DEBUG] === RESET CALLED ===")
            print("-" * 50)
        # 历史帧缓冲区: deque of 45 维 ndarray
        # 最新帧 appendleft，拼接时按 [obs_t, obs_{t-1}, ..., obs_{t-5}] 顺序
        self._history: deque[np.ndarray] = deque(
            maxlen=_HISTORY_FRAMES
        )
        self._last_action = np.zeros(_ACTION_DIM, dtype=np.float32)
        if _DEBUG:
            print("[D4_HIM_DEBUG] 历史缓冲区已重置")
            print(f"[D4_HIM_DEBUG] _last_action 初始值: {self._last_action.tolist()}")
            print("-" * 50)

    def required_inputs(self) -> set[str]:
        return {"joint_states", "imu"}

    def build_model_input(self, context: PolicyContext) -> ModelInput:
        global _call_count
        _call_count += 1
        
        # ---- 1. 构造当前帧 45 维观测 ----
        # 注意: 这里读取 self._last_action，是上一帧的输出
        current_frame = self._build_single_frame(context)  # (45,)

        # ---- 2. 更新历史缓冲区 ----
        # observations_history_priority="time" → 最新帧在 concat 向量最左侧
        # 队首是最新的，所以用 appendleft
        self._history.appendleft(current_frame)

        # ---- 3. 拼接 6 帧历史为 270 维 ----
        # list(deque) 返回从队首到队尾的迭代
        # deque 内容: [frame_t, frame_{t-1}, ..., frame_{t-5}]
        # list() 迭代: frame_t, frame_{t-1}, ..., frame_{t-5}
        # concat 结果: [frame_t, frame_{t-1}, ..., frame_{t-5}] (270D)
        # 正确性验证: 最新帧(frame_t)在最左侧 ✓
        while len(self._history) < _HISTORY_FRAMES:
            self._history.append(np.zeros(_SINGLE_FRAME_DIM, dtype=np.float32))

        history_array = np.concatenate(list(self._history))  # (270,)
        assert history_array.shape == (
            _HISTORY_FRAMES * _SINGLE_FRAME_DIM,
        ), f"history dim mismatch: {history_array.shape}"

        # ---- 4. 裁剪并返回 ----
        history_array = np.clip(
            history_array, -_CLIP_OBS, _CLIP_OBS
        ).astype(np.float32)
        
        if _DEBUG:
            print(f"\n[D4_HIM_DEBUG] === build_model_input #{_call_count} ===")
            frames = [history_array[i*_SINGLE_FRAME_DIM:(i+1)*_SINGLE_FRAME_DIM]
                      for i in range(_HISTORY_FRAMES)]
            print(f"  历史帧: {len(self._history)}/{_HISTORY_FRAMES}, "
                  f"维度: {len(history_array)}, "
                  f"范围: [{history_array.min():.2f}, {history_array.max():.2f}]")
            # 当前帧
            frame = frames[0]
            ptr = 0
            cmd = frame[ptr:ptr+3]; ptr += 3
            av = frame[ptr:ptr+3]; ptr += 3
            grav = frame[ptr:ptr+3]
            print(f"  cmd_vel: [{context.cmd_vel.linear.x:.3f}, {context.cmd_vel.linear.y:.3f}, "
                  f"{context.cmd_vel.angular.z:.3f}] × scale {_COMMANDS_SCALE.tolist()}")
            print(f"  cmd_scaled: [{cmd[0]:.4f}, {cmd[1]:.4f}, {cmd[2]:.4f}]")
            print(f"  ang_vel_scaled: [{av[0]:.4f}, {av[1]:.4f}, {av[2]:.4f}] (scale={_ANG_VEL_SCALE})")
            print(f"  gravity: [{grav[0]:.4f}, {grav[1]:.4f}, {grav[2]:.4f}]")
        
        return {"obs": history_array.tolist()}

    def decode_action(
        self,
        raw_action: Iterable[float],
        context: PolicyContext,
    ) -> List[float]:
        global _call_count
        
        action = np.asarray(list(raw_action), dtype=np.float32).reshape(-1)
        if action.shape != (_ACTION_DIM,):
            raise ValueError(
                f"himloco raw action dim mismatch: "
                f"{action.size} != {_ACTION_DIM}"
            )
        action = np.clip(action, _ACTION_LOWER, _ACTION_UPPER)
        self._last_action = action.copy()

        # ---- 5. 逆关节映射: 策略输出 → 物理顺序 ----
        # 当前 zl_robot 物理顺序恰好与策略期望一致，逆映射为恒等
        # 但保留完整映射逻辑以备将来关节顺序变更
        action_physical = self._apply_inverse_joint_mapping(action)

        # ---- 6. 计算目标关节位置 ----
        default_q = self._default_q(context)
        target_q = default_q + action_physical * _ACTION_SCALE

        # ---- 7. 按 joint_configs 的物理顺序返回目标角度 ----
        target_by_name = dict(zip(D4_JOINT_NAMES, target_q.tolist()))
        joint_configs = context.require_joint_configs()
        configured_names = [joint.name for joint in joint_configs]
        if set(configured_names) != set(D4_JOINT_NAMES):
            raise ValueError(
                "himloco requires exactly the 12 D4 joints; configured: "
                f"{configured_names}"
            )
        result = [float(target_by_name[joint.name]) for joint in joint_configs]
        
        # 调试打印
        if _DEBUG:
            print(f"\n[D4_HIM_DEBUG] === decode_action #{_call_count} ===")
            print(f"  action(策略): {action.tolist()}")
            print(f"  action(物理): {action_physical.tolist()}")
            print(f"  逆映射: {INVERSE_JOINT_MAPPING}")
            print(f"  default_q: {default_q.tolist()}")
            print(f"  target_q:  {target_q.tolist()}")
        
        return result

    def select_joint_profile(self, context: PolicyContext) -> str:
        del context
        return "run"

    # -------------------------------------------------------------------------
    # 私有方法
    # -------------------------------------------------------------------------

    def _build_single_frame(self, context: PolicyContext) -> np.ndarray:
        """构造单帧 45 维观测。

        观测顺序 (与 himloco_lab 训练/导出严格一致):
          commands (3) → ang_vel (3) → gravity (3) → dof_pos (12)
          → dof_vel (12) → actions (12) = 45
        
        注意: dof_pos 是相对位置 (q - default_q)，与 d4_flat 一致
        """
        global _call_count
        
        joint_states = context.require_joint_states()
        imu = context.require_policy_imu()
        by_name = {state.joint_name: state for state in joint_states.states}
        missing = [name for name in D4_JOINT_NAMES if name not in by_name]
        if missing:
            raise ValueError(f"himloco joint states missing: {missing}")

        # 获取 default_q 用于计算相对位置
        default_q = self._default_q(context)

        # 关节位置和速度 (按物理顺序)
        q_physical = np.asarray(
            [float(by_name[name].position) for name in D4_JOINT_NAMES],
            dtype=np.float32,
        )
        dq_physical = np.asarray(
            [float(by_name[name].velocity) for name in D4_JOINT_NAMES],
            dtype=np.float32,
        )

        # ---- 关节映射: 物理 → 策略期望顺序 ----
        q_policy = self._apply_joint_mapping(q_physical)
        dq_policy = self._apply_joint_mapping(dq_physical)
        default_q_policy = self._apply_joint_mapping(default_q)

        # angular velocity (3) - 直接用原始数据
        angular_velocity = np.asarray(imu.angular_velocity, dtype=np.float32)
        if angular_velocity.shape != (3,):
            raise ValueError("himloco IMU angular velocity must be 3-D")

        # commands (3)
        commands = np.asarray(
            [
                float(context.cmd_vel.linear.x),
                float(context.cmd_vel.linear.y),
                float(context.cmd_vel.angular.z),
            ],
            dtype=np.float32,
        )

        # projected gravity (3) — 与 d4_flat 完全相同的方式计算
        gravity = self._projected_gravity(imu.orientation_quat)

        # ---- 按策略顺序拼接 45 维 ----
        # 注意: dof_pos 是相对位置 (q - default_q)，与 d4_flat 一致
        q_rel_policy = q_policy - default_q_policy
        frame = np.concatenate(
            [
                commands * _COMMANDS_SCALE,               # 3
                angular_velocity * _ANG_VEL_SCALE,        # 3
                gravity,                                  # 3
                q_rel_policy * _DOF_POS_SCALE,          # 12 - 相对位置！
                dq_policy * _DOF_VEL_SCALE,              # 12
                self._last_action,                       # 12
            ]
        )
        if frame.shape != (_SINGLE_FRAME_DIM,):
            raise ValueError(
                f"himloco frame dim mismatch: "
                f"{frame.size} != {_SINGLE_FRAME_DIM}"
            )
        
        if _DEBUG:
            print(f"\n[D4_HIM_DEBUG] --- frame #{_call_count} ---")
            print(f"  q_physical: {q_physical.tolist()}")
            print(f"  q_policy:   {q_policy.tolist()}")
            print(f"  映射后相同: {np.array_equal(q_physical, q_policy)}")
            print(f"  帧维度: {frame.shape}, 期望: ({_SINGLE_FRAME_DIM},)")
        
        return frame

    @staticmethod
    def _apply_joint_mapping(arr: np.ndarray) -> np.ndarray:
        """物理关节顺序 → 策略网络期望顺序 (恒等, 保留以备扩展)."""
        return arr[JOINT_MAPPING]

    @staticmethod
    def _apply_inverse_joint_mapping(arr: np.ndarray) -> np.ndarray:
        """策略网络输出顺序 → 物理关节顺序 (恒等, 保留以备扩展)."""
        return arr[INVERSE_JOINT_MAPPING]

    @staticmethod
    def _projected_gravity(quaternion_wxyz: Iterable[float]) -> np.ndarray:
        """从 IMU 四元数反推投影重力向量 (base 系下).
        
        计算: g_proj = q_inv * [0,0,-1] * q
        即世界坐标系下的重力向量在 body 坐标系下的表示。
        
        期望的站立四元数: [1, 0, 0, 0] 或其等价形式
        站立时期望重力投影: [0, 0, -1]
        """
        quaternion = np.asarray(
            list(quaternion_wxyz), dtype=np.float32
        ).reshape(-1)
        if quaternion.shape != (4,):
            raise ValueError("himloco IMU orientation quaternion must be 4-D")
        norm = float(np.linalg.norm(quaternion))
        if norm < 1.0e-9:
            quaternion = np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
            print("[D4_HIM_DEBUG] 警告: 四元数归一化失败，使用单位四元数")
        else:
            quaternion = quaternion / norm
        
        w, x, y, z = quaternion
        
        # QuatRotateInverse: q_inv * v * q
        # 对于单位四元数: q_inv = [w, -x, -y, -z]
        # g = [0, 0, -1]
        # 推导结果:
        #   g_proj[0] = 2*(w*y - x*z)       → x方向重力分量
        #   g_proj[1] = -2*(y*z + w*x)      → y方向重力分量  
        #   g_proj[2] = 2*(x*x + y*y) - 1   → z方向重力分量
        
        g_proj = np.asarray(
            [
                2.0 * (w * y - x * z),
                -2.0 * (y * z + w * x),
                2.0 * (x * x + y * y) - 1.0,
            ],
            dtype=np.float32,
        )
        
        if _DEBUG:
            print(f"\n  [重力投影] quat: [{w:.3f}, {x:.3f}, {y:.3f}, {z:.3f}]")
            print(f"  [重力投影] g_proj: [{g_proj[0]:.4f}, {g_proj[1]:.4f}, {g_proj[2]:.4f}]")
            if abs(g_proj[2] + 1.0) < 0.1:
                print(f"  [状态] 站立姿态 ✓")
            elif abs(g_proj[2] - 1.0) < 0.1:
                print(f"  [警告] 机器人上下颠倒！")
        
        return g_proj

    def _default_q(self, context: PolicyContext) -> np.ndarray:
        """返回默认关节位置 (按物理顺序 D4_JOINT_NAMES)."""
        positions = self._run_profile_positions(context)
        default_q = np.asarray(
            [positions[name] for name in D4_JOINT_NAMES],
            dtype=np.float32,
        )
        
        if _DEBUG:
            print(f"  [default_q] {dict(zip(D4_JOINT_NAMES, default_q.tolist()))}")
        
        return default_q

    @staticmethod
    def _run_profile_positions(context: PolicyContext) -> Dict[str, float]:
        """从 joint_profiles.run 获取 12 个关节的默认位置."""
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
            raise ValueError(f"himloco run profile missing joints: {missing}")
        
        return positions
