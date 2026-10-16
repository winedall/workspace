"""RL policy 运行后端接口。

该模块只定义 robot_rl_manager 内部的统一 policy runtime 形状。真实 ONNX/Torch
后端后续应实现同一接口，节点层不直接依赖具体推理库。
"""

from __future__ import annotations

from typing import Dict, List, Protocol

from robot_rl_manager.model_registry import RegistryRlModelInfo


ModelInput = Dict[str, List[float]]


class PolicyBackend(Protocol):
    """RL policy 后端统一接口。"""

    def load(self, model_info: RegistryRlModelInfo) -> None:
        """加载或切换到指定模型。"""

    def reset(self) -> None:
        """重置 policy 内部状态。"""

    def infer(self, model_input: ModelInput) -> List[float]:
        """根据 adapter 构造的模型输入字典输出 action 向量。"""

    def is_ready(self) -> bool:
        """返回后端是否已经加载完成并可推理。"""

    def close(self) -> None:
        """释放后端资源。"""
