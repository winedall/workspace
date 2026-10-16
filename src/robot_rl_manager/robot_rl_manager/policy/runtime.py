"""PolicyRuntime：连接 backend、adapter 和 JointCommandBuilder。"""

from __future__ import annotations

from collections.abc import Callable
from typing import Iterable, List, Optional

import numpy as np
from geometry_msgs.msg import Point, Twist
from robot_interfaces.msg import JointCommands, JointStates

from robot_rl_manager.joint_command_builder import JointCommandBuilder, JointConfig
from robot_rl_manager.model_registry import JointProfile, RegistryRlModelInfo
from robot_rl_manager.policy.adapter import PolicyAdapter
from robot_rl_manager.policy.backend import ModelInput, PolicyBackend
from robot_rl_manager.policy.context import PolicyContext
from robot_rl_manager.policy.imu import PolicyImu


class PolicyRuntime:
    """单个活动策略模型的运行时编排器。

    backend 负责模型加载和推理；adapter 负责观测/动作语义；builder 负责生成 ROS
    JointCommands。节点层只调用 runtime，因此后续替换 ONNX/Torch 后端不影响节点流程。
    """

    def __init__(
        self,
        joint_configs: Iterable[JointConfig],
        backend: PolicyBackend,
        adapter: PolicyAdapter,
    ) -> None:
        self._joint_configs = list(joint_configs)
        if not self._joint_configs:
            raise ValueError("joint_configs cannot be empty")
        self._backend = backend
        self._adapter = adapter
        self._builder = JointCommandBuilder(self._joint_configs)
        self._model_info: RegistryRlModelInfo | None = None
        self._last_raw_action: Optional[List[float]] = None
        self._last_decoded_positions: Optional[List[float]] = None
        self._last_context_time_sec: Optional[float] = None

    @property
    def backend(self) -> PolicyBackend:
        """返回当前 backend，供节点关闭资源或查询 ready 状态。"""
        return self._backend

    @property
    def adapter(self) -> PolicyAdapter:
        """返回当前 adapter，主要便于测试和状态诊断。"""
        return self._adapter

    def load(self, model_info: RegistryRlModelInfo) -> None:
        """加载模型并重置 backend/adapter 状态。"""
        self._validate_model_metadata(model_info)
        self._model_info = model_info
        self._backend.load(model_info)
        self.reset()

    def reset(self) -> None:
        """重置 backend 和 adapter 的内部状态。"""
        self._backend.reset()
        self._adapter.reset()
        self._last_raw_action = None
        self._last_decoded_positions = None
        self._last_context_time_sec = None

    def is_ready(self) -> bool:
        """返回 runtime 是否具备推理条件。"""
        return self._model_info is not None and self._backend.is_ready()

    def is_motion_end(self) -> bool:
        """返回当前 adapter 的参考运动是否已播放完毕。"""
        return self._adapter.is_motion_end()

    def close(self) -> None:
        """释放 backend 资源。"""
        self._backend.close()

    def infer_commands(
        self,
        cmd_vel: Twist,
        joint_states: JointStates,
        policy_imu: PolicyImu | None = None,
        depth_image: np.ndarray | None = None,
        target_point: Point | None = None,
        target_destination: Point | None = None,
        profile_name: str = "run",
        source: str = "rl_manager",
        now_sec: float = 0.0,
        model_input_callback: Callable[[RegistryRlModelInfo, ModelInput, float], None] | None = None,
    ) -> JointCommands:
        """执行一次推理并构造 JointCommands。"""
        if self._model_info is None:
            raise RuntimeError("policy runtime has no active model")
        if not self._backend.is_ready():
            raise RuntimeError("policy backend is not ready")

        context = self._make_context(
            now_sec,
            cmd_vel,
            joint_states,
            policy_imu,
            depth_image,
            target_point,
            target_destination,
        )
        self._validate_required_inputs(context)

        model_input = self._adapter.build_model_input(context)
        self._validate_model_input(model_input)
        if model_input_callback is not None:
            model_input_callback(self._model_info, model_input, now_sec)

        raw_action = list(self._backend.infer(model_input))

        context.last_raw_action = raw_action
        positions = self._adapter.decode_action(raw_action, context)
        positions = list(positions)
        self._validate_decoded_action_dim(positions)

        context.last_decoded_positions = positions
        self._last_raw_action = raw_action
        self._last_decoded_positions = positions
        self._last_context_time_sec = now_sec
        
        profile = self._model_info.joint_profiles.get(profile_name)
        if profile is None:
            raise ValueError(f"unknown joint profile: {profile_name}")
        return self._builder.build(profile, positions, source=source)

    def build_model_profile_commands(
        self,
        profile_name: str,
        source: str = "rl_manager",
    ) -> JointCommands:
        """不执行推理，直接从当前模型 joint profile 生成命令。"""
        if self._model_info is None:
            raise RuntimeError("policy runtime has no active model")
        profile = self._model_info.joint_profiles.get(profile_name)
        if profile is None:
            raise ValueError(f"unknown joint profile: {profile_name}")
        return self._builder.build_from_profile(profile, source=source, command_type=JointCommands.TYPE_RUN)

    def build_prepare_commands(
        self,
        profile_name: str,
        source: str = "rl_manager",
        command_type: int | None = None,
    ) -> JointCommands:
        """从当前模型携带的 prepare profile 生成准备命令。"""
        if self._model_info is None:
            raise RuntimeError("policy runtime has no active model")
        profile = self._resolve_prepare_profile(profile_name)
        cmd_type = command_type if command_type is not None else JointCommands.TYPE_PREPARE
        return self._builder.build_from_profile(profile, source=source, command_type=cmd_type)

    def _resolve_prepare_profile(self, profile_name: str) -> JointProfile:
        """解析 prepare profile 的 alias 和踝空间选择，返回实际 joints profile。"""
        assert self._model_info is not None
        current_name = profile_name
        visited: set[str] = set()
        while True:
            if current_name in visited:
                chain = " -> ".join([*visited, current_name])
                raise ValueError(f"prepare profile alias cycle: {chain}")
            visited.add(current_name)

            profile = self._model_info.prepare_profiles.get(current_name)
            if profile is None:
                raise ValueError(f"unknown prepare profile: {current_name}")
            if profile.alias_of:
                current_name = profile.alias_of
                continue
            if profile.ankle_space_profiles:
                key = "series" if self._model_info.ankle_in_series else "parallel"
                current_name = profile.ankle_space_profiles.get(key, "")
                if not current_name:
                    raise ValueError(f"prepare profile {profile.name} missing {key} ankle target")
                continue
            return profile

    def build_damping_commands(self, source: str = "rl_manager") -> JointCommands:
        """生成统一阻尼命令，不依赖具体模型 profile。"""
        return self._builder.build_damping(source=source)

    def build_zero_moment_commands(self, source: str = "rl_manager") -> JointCommands:
        """生成统一零力矩命令，不依赖具体模型 profile。"""
        return self._builder.build_zero_moment(source=source)

    def _make_context(
        self,
        now_sec: float,
        cmd_vel: Twist,
        joint_states: JointStates,
        policy_imu: PolicyImu | None,
        depth_image: np.ndarray | None = None,
        target_point: Point | None = None,
        target_destination: Point | None = None,
    ) -> PolicyContext:
        """构造传递给 adapter 的统一上下文。"""
        dt_sec = 0.0
        if self._last_context_time_sec is not None and now_sec > self._last_context_time_sec:
            dt_sec = now_sec - self._last_context_time_sec

        return PolicyContext(
            cmd_vel=cmd_vel,
            joint_states=joint_states,
            model_info=self._model_info,
            joint_configs=list(self._joint_configs),
            policy_imu=policy_imu,
            target_point=target_point,
            target_destination=target_destination,
            depth_image=depth_image,
            last_raw_action=self._last_raw_action,
            last_decoded_positions=self._last_decoded_positions,
            now_sec=float(now_sec),
            dt_sec=float(dt_sec),
        )

    def _validate_required_inputs(self, context: PolicyContext) -> None:
        """根据 adapter 声明检查上下文输入是否齐全。"""
        missing_inputs = self.missing_required_inputs(context)
        if missing_inputs:
            missing_text = ", ".join(sorted(missing_inputs))
            raise RuntimeError(f"policy context missing required input: {missing_text}")

    def missing_required_inputs(self, context: PolicyContext) -> set[str]:
        """返回当前 adapter 声明但上下文未提供的输入。"""
        return {
            input_name
            for input_name in self._adapter.required_inputs()
            if not context.has_input(input_name)
        }

    def _validate_model_metadata(self, model_info: RegistryRlModelInfo) -> None:
        """校验模型元数据自身有效。

        action_dim 表示 backend 原始输出 raw_action 的维度，不要求等于机器人关节数量。
        raw_action 到全身关节 positions 的转换由 adapter.decode_action 负责。
        """
        if model_info.action_dim <= 0:
            raise ValueError(f"action_dim must be > 0 for model {model_info.name}")

    def _validate_model_input(self, model_input: ModelInput) -> None:
        """校验 adapter 构造的模型输入。"""
        if not model_input:
            raise ValueError("model_input cannot be empty")
        for input_name, values in model_input.items():
            if not input_name:
                raise ValueError("model_input input name cannot be empty")
        #     [float(value) for value in values]
        # self._validate_obs_dim(model_input)

    def _validate_obs_dim(self, model_input: ModelInput) -> None:
        """校验 obs 输入维度。"""
        assert self._model_info is not None
        expected_dim = int(self._model_info.observation_dim)
        if expected_dim <= 0 or "obs" not in model_input:
            return
        if len(model_input["obs"]) != expected_dim:
            raise ValueError(
                f"observation_dim mismatch for model {self._model_info.name}: "
                f"{len(model_input['obs'])} != {expected_dim}"
            )

    def _validate_decoded_action_dim(self, positions: Iterable[float]) -> None:
        """校验 adapter 解码后的关节位置目标维度。"""
        position_list = list(positions)
        if len(position_list) != len(self._joint_configs):
            raise ValueError(
                f"decoded action dim mismatch: {len(position_list)} != joint_count "
                f"{len(self._joint_configs)}"
            )
