"""policy backend/adapter 动态创建工具。

class_path 参数支持两种格式：
  - 点分模块路径，如 robot_rl_manager.policy.adapters.heart.HeartPolicyAdapter
  - .py 文件路径（绝对路径或相对路径），如 /path/to/heart.py
"""

from __future__ import annotations

import importlib
import importlib.util
from pathlib import Path
from typing import Type

from robot_rl_manager.policy.adapter import PolicyAdapter
from robot_rl_manager.policy.backend import PolicyBackend


def _load_class(class_path: str) -> Type[object]:
    """按 'package.module.ClassName' 加载类。"""
    if not class_path:
        raise ValueError("policy class path cannot be empty")
    module_name, separator, class_name = class_path.rpartition(".")
    if not separator or not module_name or not class_name:
        raise ValueError(f"invalid policy class path: {class_path}")
    module = importlib.import_module(module_name)
    return getattr(module, class_name)


def _load_class_from_file(file_path: str) -> Type[object]:
    """从 .py 文件加载策略类，类名按文件名 PascalCase + 类型后缀约定。"""
    fpath = Path(file_path).resolve()
    if not fpath.exists():
        raise FileNotFoundError(f"policy file not found: {file_path}")
    stem = fpath.stem
    class_name = "".join(word.capitalize() for word in stem.split("_")) + "PolicyAdapter"
    spec = importlib.util.spec_from_file_location(stem, str(fpath))
    if spec is None or spec.loader is None:
        raise ImportError(f"failed to load spec from: {file_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return getattr(module, class_name)


def create_policy_backend(class_path: str) -> PolicyBackend:
    """创建 policy backend。

    backend 只负责 model_input -> raw action，不持有关节配置或机器人语义。
    """
    if not class_path:
        raise ValueError("policy backend class_path cannot be empty")
    if class_path.endswith(".py"):
        backend_class = _load_class_from_file(class_path)
        return backend_class()
    backend_class = _load_class(class_path)
    return backend_class()


def create_policy_adapter(class_path: str) -> PolicyAdapter:
    """创建 policy adapter。"""
    if not class_path:
        raise ValueError("policy adapter class_path cannot be empty")
    if class_path.endswith(".py"):
        adapter_class = _load_class_from_file(class_path)
        return adapter_class()
    adapter_class = _load_class(class_path)
    return adapter_class()
