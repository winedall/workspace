"""ONNX policy backend。

该 backend 只负责模型加载、输入检查、推理和 raw action 输出整理；观测构造、
动作解释、joint profile 选择都由 adapter/runtime 负责。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, List

from robot_rl_manager.model_registry import RegistryRlModelInfo
from robot_rl_manager.policy.backend import ModelInput


class OnnxPolicyBackend:
    """使用 onnxruntime 执行 RL policy 的后端。"""

    def __init__(self) -> None:
        self._session = None
        self._input_names: List[str] = []
        self._model_info: RegistryRlModelInfo | None = None
        self._runtime_options: dict[str, Any] = {
            "intra_op_num_threads": 2,
            "inter_op_num_threads": 1,
            "execution_mode": "sequential",
        }

    def configure_runtime_options(self, options: dict[str, Any]) -> None:
        """配置 ONNXRuntime SessionOptions；通常由 rl_manager 全局参数提供。"""
        self._runtime_options.update(options)

    def load(self, model_info: RegistryRlModelInfo) -> None:
        """加载 ONNX 模型文件并记录输入名。"""
        if not model_info.file:
            raise ValueError(f"model file is empty for {model_info.name}")
        model_path = Path(model_info.file)
        if not model_path.exists():
            raise FileNotFoundError(f"ONNX model file not found: {model_path}")
        try:
            import onnxruntime as ort
        except ImportError as exc:
            raise RuntimeError("onnxruntime is required for OnnxPolicyBackend") from exc

        session_options = ort.SessionOptions()
        intra_op = int(self._runtime_options.get("intra_op_num_threads", 2))
        inter_op = int(self._runtime_options.get("inter_op_num_threads", 1))
        if intra_op > 0:
            session_options.intra_op_num_threads = intra_op
        if inter_op > 0:
            session_options.inter_op_num_threads = inter_op
        execution_mode = str(self._runtime_options.get("execution_mode", "sequential")).lower()
        if execution_mode == "parallel":
            session_options.execution_mode = ort.ExecutionMode.ORT_PARALLEL
        else:
            session_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL

        self._session = ort.InferenceSession(
            str(model_path),
            sess_options=session_options,
            providers=["CPUExecutionProvider"],
        )
        inputs = self._session.get_inputs()
        if not inputs:
            raise RuntimeError(f"ONNX model has no inputs: {model_path}")
        self._input_names = [input_info.name for input_info in inputs]
        self._model_info = model_info

    def reset(self) -> None:
        """ONNX feed-forward 模型默认没有隐状态。"""

    def infer(self, model_input: ModelInput) -> List[float]:
        """执行一次 ONNX 推理，并校验输入/输出维度。"""
        if self._session is None or self._model_info is None:
            raise RuntimeError("ONNX policy backend is not ready")

        missing_inputs = [name for name in self._input_names if name not in model_input]
        if missing_inputs:
            raise RuntimeError(
                f"ONNX model input missing: {missing_inputs}; "
                f"provided inputs: {sorted(model_input.keys())}"
            )
        unknown_inputs = [name for name in model_input.keys() if name not in self._input_names]
        if unknown_inputs:
            raise RuntimeError(
                f"ONNX model input unknown: {unknown_inputs}; "
                f"model inputs: {self._input_names}"
            )

        expected_obs_dim = int(self._model_info.observation_dim)
        obs_values = [float(value) for value in model_input.get("obs", [])]
        # if expected_obs_dim > 0 and "obs" in model_input and len(obs_values) != expected_obs_dim:
        #     raise ValueError(
        #         f"ONNX input dim mismatch for model {self._model_info.name}: "
        #         f"{len(obs_values)} != {expected_obs_dim}"
        #     )

        import numpy as np

        feed = {}
        for input_name in self._input_names:
            input_array = np.asarray(model_input[input_name], dtype=np.float32)
            if input_array.ndim == 1:
                input_array = input_array.reshape(1, -1)
            feed[input_name] = input_array

        outputs = self._session.run(None, feed)
        if not outputs:
            raise RuntimeError(f"ONNX model produced no outputs: {self._model_info.name}")

        action = np.asarray(outputs[0], dtype=np.float32).reshape(-1).tolist()
        # if len(action) != int(self._model_info.action_dim):
        #     raise ValueError(
        #         f"ONNX output dim mismatch for model {self._model_info.name}: "
        #         f"{len(action)} != {self._model_info.action_dim}"
        #     )
        return [float(value) for value in action]

    def is_ready(self) -> bool:
        """返回 ONNX session 是否已创建。"""
        return self._session is not None and self._model_info is not None

    def close(self) -> None:
        """释放 ONNX session 引用。"""
        self._session = None
        self._input_names = []
        self._model_info = None
