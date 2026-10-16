"""Python RL manager 的模型注册表。

根目录 config/rl_models.yaml 只作为统一入口索引；每个模型的控制频率、
raw action 维度、run 关节 profile 等细节放在包内独立配置文件中。
prepare 起身准备姿态由独立 prepare.yaml 管理，不再挂在具体模型配置下。
初版只管理模型元数据，不加载 ONNX/Torch，也不创建真实推理 session。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import platform
from typing import Dict, Iterable, Optional

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import yaml

REQUIRED_JOINT_PROFILES = ("run",)


@dataclass(frozen=True)
class ModelSequence:
    """命名模型序列，用于把多个模型按 motion_end 串联起来。"""
    models: tuple[str, ...] = ()
    return_home: bool = True


@dataclass(frozen=True)
class ModelSwitchRules:
    """模型切换规则。"""
    pair_map: Dict[str, str] = field(default_factory=dict)
    sequences: Dict[str, ModelSequence] = field(default_factory=dict)


@dataclass(frozen=True)
class JointProfileJoint:
    """单个关节在某个 joint profile 下的目标位置和 PD 参数。"""
    joint_name: str
    position: float
    kp: float
    kd: float


@dataclass(frozen=True)
class JointProfile:
    """run 或 prepare.yaml 中任一 profile 的逐关节配置。"""
    name: str
    joints: Dict[str, JointProfileJoint] = field(default_factory=dict)
    alias_of: str = ""
    ankle_space_profiles: Dict[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class RegistryRlModelInfo:
    """单个 RL 模型的完整元数据和配置。"""
    # 模型注册名称，用于服务切换和状态展示。
    name: str
    # 单模型配置文件路径。
    config_path: str
    # 模型文件路径。初版只保存，不打开文件。
    file: str
    # policy backend 模块路径，例如 mock、onnx、torch 后端。
    policy_backend: str
    # policy adapter 模块路径，例如 walk/stand 的语义适配器。
    policy_adapter: str
    # 模型类型，例如 onnx、torchscript。初版不据此创建推理后端。
    model_type: str
    # 模型期望控制频率，仅作为元数据保留。
    control_frequency_hz: float
    # 观测维度，仅作为元数据保留。
    observation_dim: int
    # 模型 backend 原始输出 raw_action 维度；最终关节位置维度由 adapter.decode_action 负责转换。
    action_dim: int
    # 当前模型踝关节是串联空间（true）还是并联空间（false）。robot_control 据此决定踝关节空间转换。
    ankle_in_series: bool
    # 模型运行参考 profile。walk 模型只要求 run；旧配置中额外 profile 会被保留但不强制。
    joint_profiles: Dict[str, JointProfile]
    # 系统级准备姿态 profile，例如 stand/squat/sit/lie。由独立 prepare.yaml 加载。
    prepare_profiles: Dict[str, JointProfile] = field(default_factory=dict)
    # 面向查询服务的展示名称，不参与模型控制。
    alias: str = ""
    # 面向查询服务的业务分类，不参与模型控制。
    category: str = "其他类"
    # 面向查询服务的配置时长，不参与模型控制。
    duration_sec: float = 0.0
    # 是否有固定配置时长，仅供查询服务返回。
    has_duration: bool = False


def _find_default_models_config() -> str:
    """从当前目录向上查找 config/rl_models.yaml。"""
    current = Path.cwd()
    for candidate_root in [current, *current.parents]:
        candidate = candidate_root / "config" / "rl_models.yaml"
        if candidate.exists():
            return str(candidate)
    raise FileNotFoundError("config/rl_models.yaml not found")


def _find_default_prepare_config(robot_type: str = "humanoid") -> str:
    """根据 robot_type 查找对应的 prepare 配置文件。"""
    robot_dir = "dog" if robot_type == "dog" else "humanoid"
    current = Path.cwd()
    for candidate_root in [current, *current.parents]:
        candidate = candidate_root / "config" / robot_dir / "default_prepare.yaml"
        if candidate.exists():
            return str(candidate)
    raise FileNotFoundError(f"config/{robot_dir}/default_prepare.yaml not found")


def _robot_rl_manager_share_dir() -> Optional[Path]:
    """查询 robot_rl_manager 包安装后的 share 目录。"""
    try:
        return Path(get_package_share_directory("robot_rl_manager")).resolve()
    except PackageNotFoundError:
        return None


def _resolve_path(path_text: str, base_dir: Path, package_share_dir: Optional[Path]) -> str:
    """解析 YAML 中的路径。

    绝对路径直接返回；相对路径优先按 robot_rl_manager 的 share 目录解析，
    找不到对应文件时再按当前 rl_models.yaml 所在目录解析，方便单元测试和开发临时配置。
    """
    path = Path(path_text)
    if path.is_absolute():
        return str(path)
    if package_share_dir is not None and path.parts and path.parts[0] in {"config", "model", "policy"}:
        return str((package_share_dir / path).resolve())
    if package_share_dir is not None:
        package_candidate = (package_share_dir / path).resolve()
        if package_candidate.exists():
            return str(package_candidate)
    return str((base_dir / path).resolve())


def _detect_policy_backend(model_type: str, model_name: str = "") -> str:
    """根据 model_type 自动选择策略后端类路径。"""
    if model_type == "onnx":
        return "robot_rl_manager.policy.onnx.OnnxPolicyBackend"
    elif model_type == "pt":
        return "robot_rl_manager.policy.torch.TorchPolicyBackend"
    elif model_type == "pipeline":
        return "robot_rl_manager.policy.pipeline.PipelinePolicyBackend"
    elif model_type == "rknn":
        return "robot_rl_manager.policy.rknn.RknnPolicyBackend"
    else:
        raise ValueError(
            f"unknown model_type '{model_type}' for model '{model_name}'; "
            f"expected 'onnx', 'pt', 'pipeline', or 'rknn'"
        )


def _normalize_runtime_platform(runtime_platform: str = "") -> str:
    """返回用于选择模型 artifact 的平台名。"""
    value = str(runtime_platform).strip()
    if value:
        return value
    return platform.machine().strip() or "unknown"


class ModelRegistry:
    """加载、校验和切换 RL 模型元数据。"""

    def __init__(
        self,
        config_path: str = "",
        default_model_name: str = "walk",
        prepare_profiles_config: str = "",
        robot_type: str = "humanoid",
        runtime_platform: str = "",
    ) -> None:
        # _models 只保存元数据；不持有 ONNX Runtime/Torch 对象，避免初版引入重依赖。
        self._models: Dict[str, RegistryRlModelInfo] = {}
        self._active_model_name: Optional[str] = None
        self._prepare_profiles = load_prepare_profiles(prepare_profiles_config, robot_type=robot_type)
        self._runtime_platform = _normalize_runtime_platform(runtime_platform)
        self._switch_rules = ModelSwitchRules()
        if config_path:
            self.load_from_yaml(config_path, default_model_name)
        else:
            self.load_from_yaml(_find_default_models_config(), default_model_name)

    @property
    def active_model(self) -> RegistryRlModelInfo:
        """返回当前活动模型元数据。"""
        if self._active_model_name is None:
            raise RuntimeError("no active RL model")
        return self._models[self._active_model_name]

    @property
    def switch_rules(self) -> ModelSwitchRules:
        """返回模型切换规则。"""
        return self._switch_rules

    @property
    def runtime_platform(self) -> str:
        """返回当前 registry 用于 artifact 选择的平台名。"""
        return self._runtime_platform

    def names(self) -> list[str]:
        """返回已注册模型名称列表。"""
        return list(self._models.keys())

    def get(self, name: str) -> RegistryRlModelInfo:
        """按名称查询模型。"""
        if name not in self._models:
            raise KeyError(f"unknown RL model: {name}")
        return self._models[name]

    def add_model(
        self,
        model_name: str,
        model_path: str,
        model_type: str,
        action_dim: int = 0,
        control_frequency_hz: float = 50.0,
        observation_dim: int = 0,
        config_path: str = "",
        adapter_path: str = "",
    ) -> RegistryRlModelInfo:
        """注册模型并加载完整元数据。

        提供 config_path 时，自动从 YAML 配置中读取观察/动作维度和关节增益配置，
        确保注册后的模型可直接切换运行。
        """
        if not model_name:
            raise ValueError("model_name cannot be empty")
        if not model_type:
            raise ValueError("model_type cannot be empty")

        if config_path:
            config_file = Path(config_path)
            if not config_file.exists():
                raise FileNotFoundError(f"model config not found: {config_path}")
            with config_file.open("r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
            model_data = data.get("model", {})
            joint_profiles_data = data.get("joint_profiles", {})
            joint_profiles = self._parse_joint_profiles(model_name, joint_profiles_data)
            control_frequency_hz = float(
                model_data.get("control_frequency_hz", control_frequency_hz)
            )
            observation_dim = int(model_data.get("observation_dim", observation_dim))
            action_dim = int(model_data.get("action_dim", action_dim))
            ankle_in_series = bool(model_data.get("ankle_in_series", True))
        else:
            joint_profiles = {"run": JointProfile(name="run")}
            ankle_in_series = True

        # 根据 model_type 自动选择后端
        policy_backend = _detect_policy_backend(model_type, model_name)

        # 转换 adapter 文件路径为类路径
        policy_adapter = self._resolve_adapter_class_path(adapter_path) if adapter_path else ""

        model = RegistryRlModelInfo(
            name=model_name,
            config_path=config_path,
            file=model_path,
            policy_backend=policy_backend,
            policy_adapter=policy_adapter,
            model_type=model_type,
            control_frequency_hz=control_frequency_hz,
            observation_dim=int(observation_dim),
            action_dim=int(action_dim),
            ankle_in_series=ankle_in_series,
            joint_profiles=joint_profiles,
            prepare_profiles=dict(self._prepare_profiles),
            alias=model_name,
            category="其他类",
            duration_sec=0.0,
            has_duration=False,
        )
        self._models[model_name] = model
        if self._active_model_name is None:
            self._active_model_name = model_name
        return model

    def switch(self, model_name: str) -> RegistryRlModelInfo:
        """按名称切换当前活动模型。"""
        model = self.get(model_name)
        self._active_model_name = model.name
        return model

    def validate_action_dim(self, joint_count: int, model_name: Optional[str] = None) -> None:
        """兼容旧接口：只校验 action_dim 自身有效，不再要求等于关节数量。"""
        del joint_count
        model = self.get(model_name) if model_name else self.active_model
        self._validate_action_dim_value(model)

    def validate_active_model(self, joint_count: int, joint_names: Iterable[str]) -> None:
        """校验当前活动模型的 raw action 维度和模型所需 joint profile。"""
        self.validate_model(self.active_model.name, joint_count, joint_names)

    def validate_model(self, model_name: str, joint_count: int, joint_names: Iterable[str]) -> None:
        """校验指定模型是否匹配当前机器人关节配置。"""
        del joint_count
        model = self.get(model_name)
        self._validate_action_dim_value(model)

        known_joints = set(joint_names)
        for profile_name in REQUIRED_JOINT_PROFILES:
            if profile_name not in model.joint_profiles:
                raise ValueError(f"model {model.name} missing joint profile: {profile_name}")

        for profile in model.joint_profiles.values():
            self._validate_profile_joints(profile, known_joints, "joint profile")
        for profile in model.prepare_profiles.values():
            if profile.alias_of or profile.ankle_space_profiles:
                continue
            self._validate_profile_joints(profile, known_joints, "prepare profile")
        self._validate_prepare_profile_references(model.prepare_profiles)

    def _validate_action_dim_value(self, model: RegistryRlModelInfo) -> None:
        """校验模型 raw action 维度是正数。"""
        if model.action_dim <= 0:
            raise ValueError(f"action_dim must be > 0 for model {model.name}")

    def _validate_profile_joints(
        self,
        profile: JointProfile,
        known_joints: set[str],
        profile_kind: str,
    ) -> None:
        """校验 profile 覆盖所有已知关节且 kp/kd 有效。"""
        seen = set()
        for joint_name, joint in profile.joints.items():
            if joint_name in seen:
                raise ValueError(f"duplicate {profile_kind} joint in {profile.name}: {joint_name}")
            seen.add(joint_name)
            if joint_name not in known_joints:
                raise ValueError(f"unknown joint in {profile.name}: {joint_name}")
            if joint.kp < 0.0:
                raise ValueError(f"kp must be >= 0 for joint {joint_name}")
            if joint.kd < 0.0:
                raise ValueError(f"kd must be >= 0 for joint {joint_name}")
        missing_joints = known_joints - set(profile.joints.keys())
        if missing_joints:
            missing_text = ", ".join(sorted(missing_joints))
            raise ValueError(f"missing {profile_kind} joint in {profile.name}: {missing_text}")

    def _validate_prepare_profile_references(self, profiles: Dict[str, JointProfile]) -> None:
        """校验 prepare profile 的 alias/select 引用存在且不会形成环。"""
        for profile in profiles.values():
            for target in self._prepare_profile_targets(profile):
                if target not in profiles:
                    raise ValueError(f"prepare profile {profile.name} references unknown profile: {target}")

        visiting: set[str] = set()
        visited: set[str] = set()

        def visit(profile_name: str, stack: list[str]) -> None:
            if profile_name in visiting:
                cycle = " -> ".join([*stack, profile_name])
                raise ValueError(f"prepare profile alias cycle: {cycle}")
            if profile_name in visited:
                return

            visiting.add(profile_name)
            for target in self._prepare_profile_targets(profiles[profile_name]):
                visit(target, [*stack, profile_name])
            visiting.remove(profile_name)
            visited.add(profile_name)

        for profile_name in profiles:
            visit(profile_name, [])

    @staticmethod
    def _prepare_profile_targets(profile: JointProfile) -> list[str]:
        if profile.alias_of:
            return [profile.alias_of]
        return [target for target in profile.ankle_space_profiles.values() if target]

    def load_from_yaml(self, config_path: str, default_model_name: str = "walk") -> None:
        """从根 rl_models.yaml 加载索引，并继续加载每个模型自己的配置文件。"""
        root_path = Path(config_path).resolve()
        with root_path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}

        models = data.get("models", {})
        if not models:
            raise ValueError("rl_models.yaml contains no models")
        declared_model_names = set(str(name) for name in models.keys())

        rules_data = data.get("model_switch_rules", {}) or {}

        self._models.clear()
        package_share_dir = _robot_rl_manager_share_dir()
        for name, item in models.items():
            artifact = self._select_platform_artifact(str(name), item)
            if artifact is None:
                continue
            config_text = str(item.get("config_path", ""))
            if not config_text:
                raise ValueError(f"model {name} missing config path")
            config_path_resolved = _resolve_path(config_text, root_path.parent, package_share_dir)
            self._models[str(name)] = self._load_model_config(
                str(name),
                item,
                artifact,
                Path(config_path_resolved),
                root_path.parent,
                package_share_dir,
            )

        if default_model_name not in self._models:
            raise ValueError(
                f"default model '{default_model_name}' is not available for "
                f"runtime_platform '{self._runtime_platform}'"
            )
        loaded_model_names = set(self._models.keys())
        pair_map = {}
        for pair in rules_data.get("pairs", []):
            if len(pair) >= 2 and pair[0] in loaded_model_names and pair[1] in loaded_model_names:
                pair_map[pair[1]] = pair[0]
        sequences = self._parse_model_sequences(
            rules_data.get("sequences", {}),
            declared_model_names,
            loaded_model_names,
        )
        self._switch_rules = ModelSwitchRules(
            pair_map=pair_map,
            sequences=sequences,
        )
        self._active_model_name = default_model_name

    def _select_platform_artifact(self, model_name: str, index_item: dict) -> Optional[dict]:
        artifacts = index_item.get("artifacts")
        if not isinstance(artifacts, dict) or not artifacts:
            raise ValueError(f"model {model_name} missing artifacts")
        if self._runtime_platform in artifacts:
            artifact = artifacts[self._runtime_platform]
        else:
            return None
        if not isinstance(artifact, dict):
            raise ValueError(f"model {model_name} artifact for {self._runtime_platform} must be a map")
        if not str(artifact.get("backend", "")):
            raise ValueError(f"model {model_name} artifact missing backend for {self._runtime_platform}")
        return artifact

    @staticmethod
    def _parse_model_sequences(
        sequences_data: object,
        declared_model_names: set[str],
        loaded_model_names: set[str],
    ) -> Dict[str, ModelSequence]:
        """解析命名模型序列，并在加载时拒绝有歧义或无效的配置。"""
        if not sequences_data:
            return {}
        if not isinstance(sequences_data, dict):
            raise ValueError("model_switch_rules.sequences must be a map")

        sequences: Dict[str, ModelSequence] = {}
        for name, item in sequences_data.items():
            sequence_name = str(name)
            if sequence_name in declared_model_names:
                raise ValueError(f"sequence name conflicts with model: {sequence_name}")
            if not isinstance(item, dict):
                raise ValueError(f"sequence {sequence_name} must be a map")
            raw_models = item.get("models", [])
            if not isinstance(raw_models, list) or not raw_models:
                raise ValueError(f"sequence {sequence_name} must define non-empty models")
            sequence_models = tuple(str(model_name) for model_name in raw_models)
            unknown_models = [model_name for model_name in sequence_models if model_name not in declared_model_names]
            if unknown_models:
                unknown_text = ", ".join(unknown_models)
                raise ValueError(f"sequence {sequence_name} references unknown model: {unknown_text}")
            unavailable_models = [model_name for model_name in sequence_models if model_name not in loaded_model_names]
            if unavailable_models:
                continue
            sequences[sequence_name] = ModelSequence(
                models=sequence_models,
                return_home=bool(item.get("return_home", True)),
            )
        return sequences

    def _load_model_config(
        self,
        model_name: str,
        index_item: dict,
        artifact: dict,
        config_path: Path,
        index_base_dir: Path,
        package_share_dir: Optional[Path],
    ) -> RegistryRlModelInfo:
        """加载单个模型配置文件，并与根索引中的路径信息合并。"""
        with config_path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}

        model_data = data.get("model", {})
        joint_profiles_data = data.get("joint_profiles", {})
        joint_profiles = self._parse_joint_profiles(model_name, joint_profiles_data)

        # 根索引负责管理平台相关模型文件和后端；单模型配置负责模型维度和关节 profile。
        model_file = _resolve_path(
            str(artifact.get("model_path", "")),
            index_base_dir,
            package_share_dir,
        )
        adapter_path_text = str(index_item.get("adapter_path", ""))
        if not adapter_path_text:
            raise ValueError(f"model {model_name} missing adapter_path")
        policy_adapter = self._resolve_adapter_class_path(adapter_path_text)
        model_type = str(artifact.get("backend", ""))
        policy_backend = _detect_policy_backend(model_type, model_name)
        has_duration = bool(index_item.get("has_duration", False))

        return RegistryRlModelInfo(
            name=str(model_data.get("name", model_name)),
            config_path=str(config_path),
            file=model_file,
            policy_backend=policy_backend,
            policy_adapter=policy_adapter,
            model_type=model_type,
            control_frequency_hz=float(model_data.get("control_frequency_hz", 50.0)),
            observation_dim=int(model_data.get("observation_dim", 0)),
            action_dim=int(model_data.get("action_dim", 0)),
            ankle_in_series=bool(model_data.get("ankle_in_series", True)),
            joint_profiles=joint_profiles,
            prepare_profiles=dict(self._prepare_profiles),
            alias=str(index_item.get("alias", model_name)),
            category=str(index_item.get("category", "其他类")),
            duration_sec=float(index_item.get("duration_sec", 0.0)),
            has_duration=has_duration,
        )

    @staticmethod
    def _resolve_adapter_class_path(adapter_path: str) -> str:
        """将 adapter 文件路径转换为可导入的类路径。

        转换规则：
          - 绝对路径（含 robot_rl_manager）→ 按最后一个 robot_rl_manager 推导模块路径
          - 相对路径（如 policy/adapters/heart.py）→ 直接拼接 robot_rl_manager 前缀
        类名按文件名 PascalCase + "PolicyAdapter" 约定。
        示例：
          install/.../robot_rl_manager/policy/adapters/heart.py
          → robot_rl_manager.policy.adapters.heart.HeartPolicyAdapter

          policy/adapters/stand.py
          → robot_rl_manager.policy.adapters.stand.StandPolicyAdapter
        """
        path = Path(adapter_path)
        stem = path.stem
        parts = path.parts

        indices = [i for i, p in enumerate(parts) if p == "robot_rl_manager"]
        if indices:
            pkg_idx = indices[-1]
            module_parts = [p.replace(".py", "") for p in parts[pkg_idx:] if p]
            module = ".".join(module_parts)
        elif not path.is_absolute():
            module_parts = ["robot_rl_manager"]
            for p in parts:
                p_clean = p.replace(".py", "")
                if p_clean:
                    module_parts.append(p_clean)
            module = ".".join(module_parts)
        else:
            # 绝对路径但不在 robot_rl_manager 包内：直接返回路径本身，
            # factory 会按 .py 文件处理。
            return adapter_path

        class_name = "".join(word.capitalize() for word in stem.split("_")) + "PolicyAdapter"
        return f"{module}.{class_name}"

    def _parse_joint_profiles(
        self,
        model_name: str,
        joint_profiles_data: dict,
    ) -> Dict[str, JointProfile]:
        """解析单模型逐关节 profile 配置。

        模型配置只保留运行态 run profile。准备姿态由 prepare.yaml 统一管理，
        阻尼命令由 rl_manager 直接生成，不再兼容单模型 prepare/damping profile。
        """
        joint_profiles_data = joint_profiles_data or {}
        profiles: Dict[str, JointProfile] = {}
        for profile_name in REQUIRED_JOINT_PROFILES:
            if profile_name not in joint_profiles_data:
                raise ValueError(f"model {model_name} missing joint profile: {profile_name}")
            profile_data = joint_profiles_data.get(profile_name) or {}
            profiles[profile_name] = self._parse_joint_profile(profile_name, profile_data)
        return profiles

    def _parse_joint_profile(self, profile_name: str, profile_data: dict) -> JointProfile:
        """解析单个 profile 的 joints 数组。"""
        alias_of = str(profile_data.get("alias_of", "") or "")
        select_data = profile_data.get("select_by_ankle_space", {}) or {}
        if alias_of and select_data:
            raise ValueError(f"profile {profile_name} cannot use both alias_of and select_by_ankle_space")
        if alias_of:
            return JointProfile(name=profile_name, alias_of=alias_of)
        if select_data:
            series = str(select_data.get("series", "") or "")
            parallel = str(select_data.get("parallel", "") or "")
            if not series or not parallel:
                raise ValueError(f"profile {profile_name} select_by_ankle_space requires series and parallel")
            return JointProfile(
                name=profile_name,
                ankle_space_profiles={"series": series, "parallel": parallel},
            )

        joints: Dict[str, JointProfileJoint] = {}
        for item in profile_data.get("joints", []) or []:
            joint_name = str(item.get("joint_name", ""))
            if not joint_name:
                raise ValueError(f"joint_name cannot be empty in profile {profile_name}")
            if joint_name in joints:
                raise ValueError(f"duplicate joint profile joint in {profile_name}: {joint_name}")
            position = item.get("position")
            kp = item.get("kp")
            kd = item.get("kd")
            if position is None:
                raise ValueError(f"position is required for joint {joint_name} in profile {profile_name}")
            if kp is None:
                raise ValueError(f"kp is required for joint {joint_name} in profile {profile_name}")
            if kd is None:
                raise ValueError(f"kd is required for joint {joint_name} in profile {profile_name}")
            if float(kp) < 0.0:
                raise ValueError(f"kp must be >= 0 for joint {joint_name} in profile {profile_name}")
            if float(kd) < 0.0:
                raise ValueError(f"kd must be >= 0 for joint {joint_name} in profile {profile_name}")
            joints[joint_name] = JointProfileJoint(
                joint_name=joint_name,
                position=float(position),
                kp=float(kp),
                kd=float(kd),
            )

        return JointProfile(
            name=profile_name,
            joints=joints,
        )


def load_prepare_profiles(config_path: str = "", robot_type: str = "humanoid") -> Dict[str, JointProfile]:
    """加载独立 prepare.yaml 中的起身准备姿态。"""
    path = Path(config_path or _find_default_prepare_config(robot_type)).resolve()
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}

    profiles_data = data.get("prepare_profiles", {})
    if not profiles_data:
        raise ValueError("prepare.yaml contains no prepare_profiles")

    parser = ModelRegistry.__new__(ModelRegistry)
    profiles: Dict[str, JointProfile] = {}
    for profile_name, profile_data in profiles_data.items():
        profile_name = str(profile_name)
        if not profile_name:
            raise ValueError("prepare profile name cannot be empty")
        profiles[profile_name] = parser._parse_joint_profile(profile_name, profile_data or {})
    return profiles
