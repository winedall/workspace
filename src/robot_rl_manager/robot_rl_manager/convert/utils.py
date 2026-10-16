"""Shared helpers for RKNN conversion tooling."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import yaml


def load_yaml(path: Path) -> dict[str, Any]:
    """读取 YAML 文件。

    Args:
        path: YAML 文件路径。

    Returns:
        解析后的字典。若文件为空，返回空字典。

    Notes:
        这里只接受字典根节点，避免把列表或标量误当成配置结构。
    """
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a map: {path}")
    return data


def write_yaml(path: Path, data: dict[str, Any]) -> None:
    """写出 YAML 文件。

    Args:
        path: 输出文件路径。
        data: 需要写入的字典数据。

    Returns:
        None。
    """
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=True)


def flatten_shape(shape: Iterable[int | None]) -> int:
    """计算张量 shape 的元素总数。

    Args:
        shape: 张量维度序列，未知维度可用 `None` 或 0 表示。

    Returns:
        维度乘积。未知维度按 1 处理，用于补全默认 mean/std 长度。
    """
    total = 1
    for dim in shape:
        value = int(dim) if dim not in (None, 0) else 1
        total *= max(value, 1)
    return total


def normalize_prefix_list(values: object) -> tuple[str, ...]:
    """把 YAML 里的前缀列表标准化为 tuple。

    Args:
        values: YAML 读取出来的对象，期望是 list 或 None。

    Returns:
        去掉空字符串后的前缀元组。

    Raises:
        ValueError: 当输入不是列表时抛出，防止配置格式错误被静默吞掉。
    """
    if values is None:
        return ()
    if not isinstance(values, list):
        raise ValueError("prefix list must be a YAML sequence")
    result = []
    for value in values:
        text = str(value)
        if text:
            result.append(text)
    return tuple(result)


@dataclass(frozen=True)
class ResolvedPaths:
    """Resolved source/target paths for a conversion job."""

    repo_root: Path
    package_root: Path
    manifest_path: Path
    registry_path: Path


def find_repo_root(start: Path | None = None) -> Path:
    """从给定位置向上查找仓库根目录。

    Args:
        start: 起始路径。通常是当前模块路径、manifest 路径或 registry 路径。

    Returns:
        仓库根目录 `zl_robot`。

    Notes:
        这里以 `config/humanoid/rl_models.yaml` 作为锚点，保证源码启动和
        安装后启动都能找到同一个仓库根。
    """
    current = (start or Path(__file__)).resolve()
    for candidate in [current, *current.parents]:
        # 以 rl_models.yaml 作为仓库锚点，能兼容从源码、脚本或打包环境启动。
        if (candidate / "config" / "humanoid" / "rl_models.yaml").exists():
            return candidate
    raise FileNotFoundError(
        f"unable to locate repository root from {current}; "
        "expected config/humanoid/rl_models.yaml above it"
    )


def resolve_repo_paths(
    manifest_path: Path | None = None,
    registry_path: Path | None = None,
) -> ResolvedPaths:
    """统一解析转换工具需要的关键路径。

    Args:
        manifest_path: 可选的转换 manifest 路径。
        registry_path: 可选的运行时模型注册表路径。

    Returns:
        包含仓库根目录、包目录、manifest 路径和注册表路径的结构体。
    """
    current = Path(__file__).resolve()
    probe = manifest_path or registry_path or current
    repo_root = find_repo_root(probe)
    # 包目录固定在 src/robot_rl_manager，注册表里的相对路径也是按这个目录写的。
    package_root = repo_root / "src" / "robot_rl_manager"
    manifest = (manifest_path or current.with_name("manifest.yaml")).resolve()
    registry = (registry_path or (repo_root / "config" / "humanoid" / "rl_models.yaml")).resolve()
    if not registry.exists():
        raise FileNotFoundError(f"rl_models.yaml not found: {registry}")
    return ResolvedPaths(
        repo_root=repo_root,
        package_root=package_root,
        manifest_path=manifest,
        registry_path=registry,
    )
