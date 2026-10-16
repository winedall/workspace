"""PolicyRuntime 与 PolicyAdapter 之间传递的上下文对象。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, List, Optional

import numpy as np
from geometry_msgs.msg import Point, Twist
from robot_interfaces.msg import JointStates

from robot_rl_manager.joint_command_builder import JointConfig
from robot_rl_manager.model_registry import RegistryRlModelInfo
from robot_rl_manager.policy.imu import PolicyImu


@dataclass
class PolicyContext:
    """单帧 policy 推理上下文。

    当前字段覆盖 joint_states、intent、模型配置和关节配置；NavAll 等传感器先在
    节点层转换成 policy 直接需要的语义输入，避免 adapter 依赖具体厂商消息。
    """

    cmd_vel: Twist = field(default_factory=Twist)
    joint_states: Optional[JointStates] = None
    model_info: Optional[RegistryRlModelInfo] = None
    joint_configs: List[JointConfig] = field(default_factory=list)
    policy_imu: Optional[PolicyImu] = None
    target_point: Optional[Point] = None
    target_destination: Optional[Point] = None
    depth_image: Optional[np.ndarray] = None
    odom: Any = None
    contacts: Any = None
    last_raw_action: Optional[List[float]] = None
    last_decoded_positions: Optional[List[float]] = None
    now_sec: float = 0.0
    dt_sec: float = 0.0

    def require_joint_states(self) -> JointStates:
        """返回关节状态；缺失时抛出明确错误。"""
        if self.joint_states is None:
            raise RuntimeError("policy context missing joint_states")
        return self.joint_states

    def require_model_info(self) -> RegistryRlModelInfo:
        """返回模型配置；缺失时抛出明确错误。"""
        if self.model_info is None:
            raise RuntimeError("policy context missing model_info")
        return self.model_info

    def require_joint_configs(self) -> List[JointConfig]:
        """返回关节配置；缺失时抛出明确错误。"""
        if not self.joint_configs:
            raise RuntimeError("policy context missing joint_configs")
        return self.joint_configs

    def require_policy_imu(self) -> PolicyImu:
        """返回 policy IMU 语义输入。"""
        if self.policy_imu is None:
            raise RuntimeError("policy context missing policy_imu")
        return self.policy_imu

    def require_target_point(self) -> Point:
        """返回球在基座系下的位置（米）；缺失时抛出明确错误。"""
        if self.target_point is None:
            raise RuntimeError("policy context missing target_point")
        return self.target_point

    def require_target_destination(self) -> Point:
        """返回球门在基座系下的位置（米）；缺失时抛出明确错误。"""
        if self.target_destination is None:
            raise RuntimeError("policy context missing target_destination")
        return self.target_destination

    def has_input(self, input_name: str) -> bool:
        """检查某个命名输入是否已提供。"""
        if input_name == "cmd_vel":
            return self.cmd_vel is not None
        if input_name == "joint_states":
            return self.joint_states is not None
        if input_name == "model_info":
            return self.model_info is not None
        if input_name == "joint_configs":
            return bool(self.joint_configs)
        if input_name == "imu":
            return self.policy_imu is not None
        if input_name == "target_point":
            return self.target_point is not None
        if input_name == "target_destination":
            return self.target_destination is not None
        return getattr(self, input_name, None) is not None
