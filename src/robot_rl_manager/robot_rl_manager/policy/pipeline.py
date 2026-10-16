"""Pipeline policy backend — 用于脚本化/预置流水线动作。

该 backend 不加载任何神经网络模型，infer 返回固定长度的零向量。
Adapter 的 decode_action 完全忽略 raw_action，直接按预定义轨迹输出关节位置。
"""

from __future__ import annotations

from typing import List

from robot_rl_manager.model_registry import RegistryRlModelInfo
from robot_rl_manager.policy.backend import ModelInput


class PipelinePolicyBackend:

    def __init__(self) -> None:
        self._ready = False

    def load(self, model_info: RegistryRlModelInfo) -> None:
        self._ready = True

    def reset(self) -> None:
        pass

    def infer(self, model_input: ModelInput) -> List[float]:
        return [0.0] * 1

    def is_ready(self) -> bool:
        return self._ready

    def close(self) -> None:
        self._ready = False
