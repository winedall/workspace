"""根据 RL policy 输出构造 JointCommands。

该模块只处理 robot_joints.yaml 中的逻辑关节顺序和统一消息构造，不读取
hw_encos.yaml，也不包含任何真实硬件访问逻辑。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List

import yaml
from robot_interfaces.msg import JointCommand, JointCommands
from robot_rl_manager.model_registry import JointProfile, RegistryRlModelInfo


@dataclass(frozen=True)
class JointConfig:
    """从 robot_joints.yaml 加载的单个逻辑关节配置。"""

    # 关节唯一名称，必须与 JointCommand.joint_name / JointState.joint_name 一致。
    name: str
    # 逻辑分组，例如 head、left_leg、right_arm。
    group: str
    # 单位，旋转关节通常为 rad，直线关节后续可用 m。
    unit: str
    # 默认是否允许使能；mock policy 构造命令时直接沿用该策略。
    enable_default: bool
    # 以下边界初版仅随配置保存，最终限幅由 robot_control 负责。
    min_pos: float
    max_pos: float
    max_vel: float
    max_effort: float


def _find_default_robot_joints_config() -> str:
    """从当前目录向上查找 config/robot_joints.yaml。"""
    current = Path.cwd()
    for candidate_root in [current, *current.parents]:
        candidate = candidate_root / "config" / "robot_joints.yaml"
        if candidate.exists():
            return str(candidate)
    raise FileNotFoundError("config/robot_joints.yaml not found")


def load_joint_configs(config_path: str = "") -> List[JointConfig]:
    """加载 robot_joints.yaml，并保留 YAML 中的关节顺序。

    这个顺序是 RL action 向量与 JointCommands 之间的稳定映射关系，不能随意排序。
    """
    path = Path(config_path) if config_path else Path(_find_default_robot_joints_config())
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}

    # 兼容两种 YAML 结构：根节点直接是 joints，或包在 robot_joints 下。
    root = data.get("robot_joints", data)
    defaults = root.get("defaults", {})
    joints = root.get("joints", [])
    if not joints:
        raise ValueError("robot_joints.yaml contains no joints")

    configs: List[JointConfig] = []
    seen = set()
    for item in joints:
        # 关节名是全系统连接接口、控制和驱动的稳定 key，必须非空且唯一。
        name = str(item.get("name", ""))
        if not name:
            raise ValueError("joint name cannot be empty")
        if name in seen:
            raise ValueError(f"duplicate joint name: {name}")
        seen.add(name)

        configs.append(
            JointConfig(
                name=name,
                # 单关节字段缺失时使用 defaults，便于配置文件保持简洁。
                group=str(item.get("group", defaults.get("group", ""))),
                unit=str(item.get("unit", defaults.get("unit", "rad"))),
                enable_default=bool(item.get("enable_default", defaults.get("enable_default", False))),
                min_pos=float(item.get("min_pos", defaults.get("min_pos", 0.0))),
                max_pos=float(item.get("max_pos", defaults.get("max_pos", 0.0))),
                max_vel=float(item.get("max_vel", defaults.get("max_vel", 0.0))),
                max_effort=float(item.get("max_effort", defaults.get("max_effort", 0.0))),
            )
        )

    return configs


class JointCommandBuilder:
    """把 policy action 向量转换成 robot_interfaces/JointCommands。"""

    def __init__(self, joint_configs: Iterable[JointConfig]) -> None:
        # 复制成 list，保证后续多次 build 时使用同一份稳定顺序。
        self._joint_configs = list(joint_configs)
        if not self._joint_configs:
            raise ValueError("joint_configs cannot be empty")

    @property
    def joint_count(self) -> int:
        """返回已配置关节数量。"""
        return len(self._joint_configs)

    def build(
        self,
        profile: JointProfile,
        positions: Iterable[float],
        source: str = "rl_manager",
        sequence: int = 0,
    ) -> JointCommands:
        """按 robot_joints.yaml 顺序构造一帧 JointCommands。"""
        position_list = list(positions)
        # adapter 解码后的 positions 必须与关节数量一致，否则无法可靠映射到关节名。
        if len(position_list) != len(self._joint_configs):
            raise ValueError(f"decoded positions dim mismatch: positions has {len(position_list)} values, but {len(self._joint_configs)} joints are configured")

        frame = self._new_frame(source, sequence, JointCommands.TYPE_RUN)
        for joint_config, position in zip(self._joint_configs, position_list):
            profile_joint = profile.joints.get(joint_config.name)
            if profile_joint is None:
                raise ValueError(f"missing joint profile joint for {joint_config.name} in profile {profile.name}")
            command = self._new_joint_command(joint_config)
            command.position = float(position)
            command.kp = float(profile_joint.kp)
            command.kd = float(profile_joint.kd)
            frame.commands.append(command)
        return frame

    def build_from_profile(
        self,
        profile: JointProfile,
        source: str = "rl_manager",
        sequence: int = 0,
        command_type: int = 0,
    ) -> JointCommands:
        """直接使用 profile 中的 position/kp/kd 构造一帧全身命令。"""
        frame = self._new_frame(source, sequence, command_type)
        for joint_config in self._joint_configs:
            profile_joint = profile.joints.get(joint_config.name)
            if profile_joint is None:
                raise ValueError(f"missing joint profile joint for {joint_config.name} in profile {profile.name}")
            command = self._new_joint_command(joint_config)
            command.position = float(profile_joint.position)
            command.kp = float(profile_joint.kp)
            command.kd = float(profile_joint.kd)
            frame.commands.append(command)
        return frame

    def build_damping(
        self,
        source: str = "rl_manager",
        sequence: int = 0,
        kd: float = 2.0,
    ) -> JointCommands:
        """生成统一阻尼命令：position=0、kp=0、kd=2。"""
        frame = self._new_frame(source, sequence, JointCommands.TYPE_DAMPING)
        for joint_config in self._joint_configs:
            command = self._new_joint_command(joint_config)
            command.position = 0.0
            command.kp = 0.0
            command.kd = float(kd)
            frame.commands.append(command)
        return frame

    def build_zero_moment(
        self,
        source: str = "rl_manager",
        sequence: int = 0,
    ) -> JointCommands:
        """生成统一零力矩命令：position=0、kp=0、kd=5。"""
        frame = self._new_frame(source, sequence, JointCommands.TYPE_ZERO_MOMENT)
        for joint_config in self._joint_configs:
            command = self._new_joint_command(joint_config)
            command.position = 0.0
            command.kp = 0.0
            command.kd = 0.0
            frame.commands.append(command)
        return frame

    def _new_frame(self, source: str, sequence: int, command_type: int = 0) -> JointCommands:
        """创建带通用头部字段的 JointCommands。"""
        frame = JointCommands()
        frame.sequence = int(sequence)
        frame.source = source
        frame.type = command_type
        return frame

    def _new_joint_command(self, joint_config: JointConfig) -> JointCommand:
        """创建单关节命令，并填充通用字段。"""
        command = JointCommand()
        command.joint_name = joint_config.name
        command.velocity = 0.0
        command.effort = 0.0
        command.enable = joint_config.enable_default
        return command
