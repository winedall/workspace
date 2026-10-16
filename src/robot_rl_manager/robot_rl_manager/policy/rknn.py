"""RKNN policy backend.

该 backend 只负责 RKNN 模型加载、输入整理、推理和资源释放；观测语义和
动作解释仍由 adapter/runtime 负责。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, List

import numpy as np

from robot_rl_manager.model_registry import RegistryRlModelInfo
from robot_rl_manager.policy.backend import ModelInput


class RknnPolicyBackend:
    """使用 RKNNLite 执行 RL policy 的后端。"""

    def __init__(self) -> None:
        self._rknn = None
        self._model_info: RegistryRlModelInfo | None = None
        self._runtime_options: dict[str, Any] = {
            "core_mask": 7,
        }

    def configure_runtime_options(self, options: dict[str, Any]) -> None:
        """配置 RKNNLite runtime 参数。"""
        self._runtime_options.update(options)

    def load(self, model_info: RegistryRlModelInfo) -> None:
        """加载 RKNN 模型并初始化 runtime。"""
        if not model_info.file:
            raise ValueError(f"model file is empty for {model_info.name}")
        model_path = Path(model_info.file)
        if not model_path.exists():
            raise FileNotFoundError(f"RKNN model file not found: {model_path}")
        try:
            from rknnlite.api import RKNNLite
        except ImportError as exc:
            raise RuntimeError("rknnlite is required for RknnPolicyBackend") from exc

        rknn = RKNNLite()
        ret = rknn.load_rknn(str(model_path))
        if ret != 0:
            raise RuntimeError(f"failed to load RKNN model: {model_path}")

        init_kwargs = {}
        core_mask = int(self._runtime_options.get("core_mask", 0))
        if core_mask > 0:
            init_kwargs["core_mask"] = core_mask
        ret = rknn.init_runtime(**init_kwargs)
        if ret != 0:
            rknn.release()
            raise RuntimeError(f"failed to init RKNN runtime for {model_info.name}")

        self._rknn = rknn
        self._model_info = model_info

    def reset(self) -> None:
        """RKNN feed-forward 模型默认没有隐状态。"""

    def infer(self, model_input: ModelInput) -> List[float]:
        """执行一次 RKNN 推理。"""
        if self._rknn is None or self._model_info is None:
            raise RuntimeError("RKNN policy backend is not ready")
        if not model_input:
            raise ValueError("RKNN model_input cannot be empty")

        inputs = []
        for values in model_input.values():
            input_array = np.asarray(values, dtype=np.float32)
            if input_array.ndim == 1:
                input_array = input_array.reshape(1, -1)
            inputs.append(input_array)

        outputs = self._rknn.inference(inputs=inputs)
        if not outputs:
            raise RuntimeError(f"RKNN model produced no outputs: {self._model_info.name}")
        action = np.asarray(outputs[0], dtype=np.float32).reshape(-1).tolist()
        return [float(value) for value in action]

    def is_ready(self) -> bool:
        """返回 RKNN runtime 是否已初始化。"""
        return self._rknn is not None and self._model_info is not None

    def close(self) -> None:
        """释放 RKNN runtime。"""
        if self._rknn is not None:
            self._rknn.release()
        self._rknn = None
        self._model_info = None
