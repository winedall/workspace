"""里程计模型注册表。

两级索引，与 robot_rl_manager 的 rl_models.yaml 同构但更轻：

    注册表（config/odometry_models.yaml）
        models:
          <model_name>:
            config_path:  "config/<model_name>.yaml"      每模型参数
            model_path:   "model/<model_name>.onnx"       ONNX 图
            adapter_path: "adapters/<model_name>.py"      输入适配器

    每模型配置（config/<model_name>.yaml，与 rl_manager 同构）
        model:
          name / type / control_frequency_hz / observation_dim
          history_len / odom_dim / use_base_acceleration / obs_layout

adapter 类名按文件名 PascalCase + "OdometryAdapter" 约定，
如 ``adapters/z01_std_odom_no_acc.py`` → ``Z01StdOdomNoAccOdometryAdapter``。

注册表内的相对路径都相对**包根目录**书写，解析顺序为：
包 share 目录 → 注册表上两级 → 注册表所在目录，后两者覆盖未 colcon build、
直接跑源码树的情况。
"""

from __future__ import annotations

import importlib
import importlib.util
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Type

import yaml

from .adapters.base import BaseOdometryAdapter
from .joint_order import (
    OBS_LAYOUT_NO_ACC,
    OBS_LAYOUT_WITH_ACC,
    ODOM_DIM,
    ODOM_HISTORY_LEN,
    ODOM_STEP_DT,
    observation_spec,
)

# control_frequency_hz 缺省值，与模型训练步长一致。
DEFAULT_CONTROL_FREQUENCY_HZ = 1.0 / ODOM_STEP_DT

# 相对路径解析时优先尝试的前缀目录。
PACKAGE_NAME = "robot_odometry"


class OdometryModelError(ValueError):
    """模型注册表或模型配置不合法。"""


@dataclass(frozen=True)
class OdometryModelInfo:
    """单个里程计模型的完整加载信息。"""

    name: str
    #: ONNX 图路径。
    model_path: str
    #: adapter 类路径（点分模块或 .py 文件路径）。
    adapter_class_path: str
    #: 单帧观测维度。
    obs_dim: int
    #: 历史窗口帧数。
    history_len: int
    #: 位移增量维度。
    odom_dim: int
    #: 模型训练时的采样频率，单位 Hz。节点据此驱动推理定时器。
    control_frequency_hz: float
    #: 观测是否包含基座加速度。
    use_base_acceleration: bool
    #: 观测布局说明，便于日志核对。
    obs_layout: str


def _package_share_dir() -> Optional[Path]:
    """返回本包安装后的 share 目录；未安装时返回 None。"""
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(PACKAGE_NAME)).resolve()
    except Exception:  # noqa: BLE001 - 未安装/非 ROS 环境
        return None


def _candidate_roots(registry_path: Path) -> list[Path]:
    """返回相对路径的候选根目录，按优先级排列。

    注册表位于 ``<root>/config/odometry_models.yaml``，里面的路径
    （``config/x.yaml``、``model/x.onnx``）都是相对包根目录写的，
    因此依次尝试：包 share 目录 → 注册表上两级 → 注册表所在目录。
    后两者覆盖未 colcon build、直接跑源码树的情况。
    """
    roots: list[Path] = []
    share_dir = _package_share_dir()
    if share_dir is not None:
        roots.append(share_dir)
    registry_dir = registry_path.parent
    roots.append(registry_dir.parent)
    roots.append(registry_dir)
    return roots


def _resolve_path(path_text: str, roots: list[Path]) -> str:
    """解析 YAML 中的相对路径。

    绝对路径直接返回；相对路径按 ``roots`` 顺序逐个尝试，取第一个存在的。
    都不存在时返回按第一个根拼出的路径，便于报错信息指出预期的位置。
    """
    path = Path(path_text)
    if path.is_absolute():
        return str(path)

    for root in roots:
        candidate = root / path
        if candidate.exists():
            return str(candidate.resolve())

    return str((roots[0] / path).resolve())


def _resolve_adapter_class_path(adapter_path: str, roots: list[Path]) -> str:
    """把 adapter 文件路径转成可导入的类路径。

    规则与 robot_rl_manager 一致：类名 = 文件名 PascalCase + "OdometryAdapter"。

        adapters/z01_std_odom_no_acc.py
            → robot_odometry.adapters.z01_std_odom_no_acc.Z01StdOdomNoAccOdometryAdapter
    """
    path = Path(_resolve_path(adapter_path, roots))

    class_name = (
        "".join(word.capitalize() for word in path.stem.split("_"))
        + "OdometryAdapter"
    )

    parts = path.parts
    indices = [i for i, part in enumerate(parts) if part == PACKAGE_NAME]
    if indices:
        module_parts = [
            part.replace(".py", "")
            for part in parts[indices[-1]:]
            if part
        ]
        module = ".".join(module_parts)
        return f"{module}.{class_name}"

    # 绝对路径但不在本包内：交给工厂按 .py 文件加载。
    return adapter_path


def load_adapter_class(class_path: str, base_dir: Optional[Path] = None) -> Type[BaseOdometryAdapter]:
    """按类路径或 .py 文件路径加载 adapter 类。"""
    if not class_path:
        raise OdometryModelError("adapter 类路径为空")

    if class_path.endswith(".py"):
        fpath = Path(class_path)
        if not fpath.is_absolute() and base_dir is not None:
            fpath = base_dir / fpath
        fpath = fpath.resolve()
        if not fpath.exists():
            raise FileNotFoundError(f"adapter 文件不存在: {fpath}")
        class_name = (
            "".join(word.capitalize() for word in fpath.stem.split("_"))
            + "OdometryAdapter"
        )
        spec = importlib.util.spec_from_file_location(fpath.stem, str(fpath))
        if spec is None or spec.loader is None:
            raise ImportError(f"无法从文件加载 adapter: {fpath}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return getattr(module, class_name)

    module_name, separator, class_name = class_path.rpartition(".")
    if not separator:
        raise OdometryModelError(f"非法的 adapter 类路径: {class_path}")
    module = importlib.import_module(module_name)
    return getattr(module, class_name)


class OdometryModelRegistry:
    """加载注册表并提供按名查询。"""

    def __init__(self, registry_path: str) -> None:
        self._registry_path = Path(registry_path)
        if not self._registry_path.is_file():
            raise FileNotFoundError(
                f"里程计模型注册表不存在: {self._registry_path}"
            )
        self._roots = _candidate_roots(self._registry_path)

        with self._registry_path.open(encoding="utf-8") as handle:
            document = yaml.safe_load(handle) or {}
        if not isinstance(document, dict):
            raise OdometryModelError("注册表根节点必须是映射")

        self._default_model_name = str(
            document.get("default_model_name", "")
        )
        model_entries = document.get("models") or {}
        if not isinstance(model_entries, dict) or not model_entries:
            raise OdometryModelError("注册表缺少 models 段或为空")

        self._entries: Dict[str, Dict[str, Any]] = {
            str(name): dict(entry or {})
            for name, entry in model_entries.items()
        }
        self._cache: Dict[str, OdometryModelInfo] = {}

    @property
    def default_model_name(self) -> str:
        """注册表声明的默认模型名。"""
        return self._default_model_name

    @property
    def model_names(self) -> list[str]:
        """已注册的模型名列表。"""
        return sorted(self._entries)

    def resolve_model_name(self, requested: str) -> str:
        """把请求的模型名解析为实际可用的名字。

        请求为空时回退到注册表默认值；两者都为空则报错。
        """
        name = (requested or "").strip() or self._default_model_name
        if not name:
            raise OdometryModelError(
                "未指定模型名，且注册表没有 default_model_name"
            )
        if name not in self._entries:
            raise OdometryModelError(
                f"未知的里程计模型 '{name}'，已注册: {self.model_names}"
            )
        return name

    def get(self, model_name: str) -> OdometryModelInfo:
        """返回指定模型的加载信息（带缓存）。"""
        if model_name in self._cache:
            return self._cache[model_name]

        entry = self._entries.get(model_name)
        if entry is None:
            raise OdometryModelError(
                f"未知的里程计模型 '{model_name}'，已注册: {self.model_names}"
            )

        config_path_text = str(entry.get("config_path", "")).strip()
        model_path_text = str(entry.get("model_path", "")).strip()
        adapter_path_text = str(entry.get("adapter_path", "")).strip()
        for field_name, value in (
            ("config_path", config_path_text),
            ("model_path", model_path_text),
            ("adapter_path", adapter_path_text),
        ):
            if not value:
                raise OdometryModelError(
                    f"模型 '{model_name}' 缺少 {field_name}"
                )

        config_path = Path(_resolve_path(config_path_text, self._roots))
        if not config_path.is_file():
            raise FileNotFoundError(
                f"模型 '{model_name}' 的配置文件不存在: {config_path}"
            )
        with config_path.open(encoding="utf-8") as handle:
            document = yaml.safe_load(handle) or {}
        if not isinstance(document, dict):
            raise OdometryModelError(
                f"模型 '{model_name}' 的配置根节点必须是映射"
            )
        # 与 robot_rl_manager 的每模型配置同构：参数都放在 model: 块下。
        config = document.get("model")
        if not isinstance(config, dict):
            raise OdometryModelError(
                f"模型 '{model_name}' 的配置缺少 model: 块（见 "
                f"{config_path.name} 的示例）"
            )

        declared_name = str(config.get("name", "")).strip()
        if declared_name and declared_name != model_name:
            raise OdometryModelError(
                f"模型 '{model_name}' 的配置里 name='{declared_name}' "
                f"与注册表键名不一致"
            )

        use_base_acceleration = bool(config.get("use_base_acceleration", False))
        expected_dim, expected_layout, _ = observation_spec(
            use_base_acceleration
        )

        obs_dim = int(config.get("observation_dim", 0))
        if obs_dim != expected_dim:
            raise OdometryModelError(
                f"模型 '{model_name}' 的 observation_dim={obs_dim} 与 "
                f"use_base_acceleration={use_base_acceleration} 不符，"
                f"应为 {expected_dim}"
            )

        # obs_layout 是必填的：它是「这份配置描述的是哪种输入」的唯一显式声明，
        # 也是复制配置加新模型时唯一会暴露语义错配的地方。省略它等于放弃校验，
        # 因此这里不接受缺省。
        declared_layout = config.get("obs_layout")
        if declared_layout is None:
            raise OdometryModelError(
                f"模型 '{model_name}' 的配置缺少 obs_layout（必填）。"
                f"应当逐字写明观测布局，当前代码支持：\n"
                f"  {OBS_LAYOUT_NO_ACC}\n"
                f"  {OBS_LAYOUT_WITH_ACC}"
            )
        if str(declared_layout) != expected_layout:
            raise OdometryModelError(
                f"模型 '{model_name}' 的 obs_layout 与当前代码不一致。\n"
                f"  配置: {declared_layout}\n"
                f"  期望: {expected_layout}"
            )

        # 采样频率决定历史窗口的物理时长（50 帧 @ F Hz = 50/F 秒），
        # 模型是按训练时的 step_dt 学的，换频率会改变窗口语义，因此这里严格校验。
        control_frequency_hz = float(
            config.get("control_frequency_hz", DEFAULT_CONTROL_FREQUENCY_HZ)
        )
        expected_frequency_hz = 1.0 / ODOM_STEP_DT
        if abs(control_frequency_hz - expected_frequency_hz) > 1e-6:
            raise OdometryModelError(
                f"模型 '{model_name}' 的 control_frequency_hz="
                f"{control_frequency_hz} 与模型训练步长不符，应为 "
                f"{expected_frequency_hz}（ODOM_STEP_DT={ODOM_STEP_DT}s）。"
                f"历史窗口 {ODOM_HISTORY_LEN} 帧的物理时长会随之改变。"
            )

        model_path = _resolve_path(model_path_text, self._roots)
        if not Path(model_path).is_file():
            raise FileNotFoundError(
                f"模型 '{model_name}' 的 ONNX 图不存在: {model_path}"
            )

        info = OdometryModelInfo(
            name=model_name,
            model_path=model_path,
            adapter_class_path=_resolve_adapter_class_path(
                adapter_path_text, self._roots
            ),
            obs_dim=obs_dim,
            history_len=int(config.get("history_len", ODOM_HISTORY_LEN)),
            odom_dim=int(config.get("odom_dim", ODOM_DIM)),
            control_frequency_hz=control_frequency_hz,
            use_base_acceleration=use_base_acceleration,
            obs_layout=expected_layout,
        )
        self._cache[model_name] = info
        return info
