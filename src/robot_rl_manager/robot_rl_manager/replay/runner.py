"""Replay 推理后端的统一适配层。"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from platform import machine
from typing import Any, Mapping, Protocol, Sequence, runtime_checkable
import importlib
import os

import numpy as np


@runtime_checkable
class ReplayRunner(Protocol):
    """所有 replay 推理后端都要遵循的最小接口。"""

    def load(self, model_path: Path) -> None:
        """Load the model into the backend."""

    def run(self, inputs: Mapping[str, np.ndarray]) -> list[np.ndarray]:
        """Run inference for a mapping of named inputs."""

    def close(self) -> None:
        """Release backend resources."""


def _import_optional(module_name: str) -> Any:
    """延迟导入可选依赖，避免没有安装 RKNN / ONNX 时直接炸掉导入阶段。"""
    return importlib.import_module(module_name)


def _to_numpy_array(value: Any) -> np.ndarray:
    """把输入统一转成 NumPy 数组，便于不同后端复用。"""
    return np.asarray(value)


def _missing_input_error(model_inputs: Mapping[str, np.ndarray], required_names: Sequence[str]) -> RuntimeError:
    """构造统一的模型输入缺失错误。

    Args:
        model_inputs: 调用者传入的输入字典。
        required_names: 后端实际需要的输入名顺序。

    Returns:
        带有上下文信息的运行时错误，便于直接定位 sample 文件和模型输入契约不一致的问题。
    """
    provided_names = ", ".join(sorted(str(name) for name in model_inputs.keys())) or "none"
    expected_names = ", ".join(str(name) for name in required_names) or "unknown"
    return RuntimeError(
        "replay sample is missing required model inputs: "
        f"expected [{expected_names}], provided [{provided_names}]"
    )


def _call_first_available(obj: Any, method_names: Sequence[str], *args: Any, **kwargs: Any) -> Any:
    """按顺序调用对象上第一个存在的方法名。

    这个 helper 用来兼容不同版本 RKNN / RKNNLite 的轻微 API 差异。
    """
    for method_name in method_names:
        method = getattr(obj, method_name, None)
        if method is not None:
            return method(*args, **kwargs)
    raise AttributeError(f"{type(obj).__name__} does not provide any of: {', '.join(method_names)}")


@dataclass
class OnnxRuntimeRunner:
    """ONNX Runtime 的懒加载封装。"""

    session_options: Mapping[str, Any] | None = None
    providers: Sequence[str] | None = None
    expected_input_names: tuple[str, ...] = ()
    expected_input_dtypes: tuple[np.dtype[Any] | None, ...] = ()
    expected_input_shapes: tuple[tuple[int, ...], ...] = ()
    _session: Any = field(default=None, init=False, repr=False)

    def load(self, model_path: Path) -> None:
        """加载 ONNX 模型，并缓存输入名顺序。"""
        ort = _import_optional("onnxruntime")
        session_options = None
        if self.session_options is not None:
            session_options = ort.SessionOptions()
            for key, value in self.session_options.items():
                setattr(session_options, key, value)
        self._session = ort.InferenceSession(
            str(model_path),
            sess_options=session_options,
            providers=list(self.providers) if self.providers is not None else None,
        )
        session_inputs = tuple(self._session.get_inputs())
        session_input_names = tuple(info.name for info in session_inputs)
        if not self.expected_input_names:
            raise RuntimeError("replay sample manifest must provide expected input_names")
        if session_input_names != self.expected_input_names:
            raise RuntimeError(
                f"ONNX model input names mismatch: expected {self.expected_input_names}, got {session_input_names}"
            )
        if len(self.expected_input_dtypes) != len(self.expected_input_names):
            raise RuntimeError("replay sample manifest must provide expected input dtypes")
        if len(self.expected_input_shapes) != len(self.expected_input_names):
            raise RuntimeError("replay sample manifest must provide expected input shapes")

    def run(self, inputs: Mapping[str, np.ndarray]) -> list[np.ndarray]:
        """按模型输入名执行一次推理。"""
        if self._session is None:
            raise RuntimeError("ONNX Runtime runner is not loaded")
        if self.expected_input_names:
            try:
                feed = {}
                for index, name in enumerate(self.expected_input_names):
                    array = _to_numpy_array(inputs[name])
                    expected_dtype = self.expected_input_dtypes[index]
                    if expected_dtype is None:
                        raise RuntimeError(f"missing expected dtype for ONNX input '{name}'")
                    if array.dtype != expected_dtype:
                        raise RuntimeError(
                            f"ONNX input dtype mismatch for '{name}': expected {expected_dtype}, got {array.dtype}"
                        )
                    expected_shape = self.expected_input_shapes[index]
                    if array.shape != expected_shape:
                        raise RuntimeError(
                            f"ONNX input shape mismatch for '{name}': expected {expected_shape}, got {array.shape}"
                        )
                    feed[name] = array
            except KeyError as exc:
                raise _missing_input_error(inputs, self.expected_input_names) from exc
        else:
            raise RuntimeError("replay sample manifest must provide expected input contract")
        outputs = self._session.run(None, feed)
        return [np.asarray(output) for output in outputs]

    def close(self) -> None:
        """释放会话引用。"""
        self._session = None


@dataclass
class RknnRunner:
    """RKNN / RKNNLite 的懒加载封装。"""

    runtime_mode: str = "auto"
    expected_input_names: tuple[str, ...] = ()
    expected_input_dtypes: tuple[np.dtype[Any] | None, ...] = ()
    expected_input_shapes: tuple[tuple[int, ...], ...] = ()
    _runner: Any = field(default=None, init=False, repr=False)

    def _resolve_runtime_mode(self) -> str:
        """根据显式参数、环境变量和机器架构决定使用 toolkit 还是 lite。"""
        runtime_mode = self.runtime_mode.lower()
        if runtime_mode in {"toolkit", "toolkit2", "rknn"}:
            return "toolkit2"
        if runtime_mode in {"lite", "rknnlite"}:
            return "lite"
        env_mode = os.environ.get("ROBOT_RL_MANAGER_RKNN_RUNTIME", "").strip().lower()
        if env_mode in {"toolkit", "toolkit2", "rknn"}:
            return "toolkit2"
        if env_mode in {"lite", "rknnlite"}:
            return "lite"
        return "lite" if machine().lower() in {"aarch64", "arm64"} else "toolkit2"

    def load(self, model_path: Path) -> None:
        """加载 RKNN 文件并初始化 runtime。"""
        runtime_mode = self._resolve_runtime_mode()
        if runtime_mode == "lite":
            module = _import_optional("rknnlite.api")
            runner_cls = module.RKNNLite
        else:
            module = _import_optional("rknn.api")
            runner_cls = module.RKNN

        runner = runner_cls()
        load_result = _call_first_available(runner, ("load_rknn", "load_model"), str(model_path))
        if load_result not in (None, 0):
            raise RuntimeError(f"failed to load RKNN model: {model_path}")
        init_result = _call_first_available(runner, ("init_runtime",),)
        if init_result not in (None, 0):
            raise RuntimeError(f"failed to initialize RKNN runtime for: {model_path}")
        self._runner = runner

    def run(self, inputs: Mapping[str, np.ndarray]) -> list[np.ndarray]:
        """按模型输入顺序执行 RKNN 推理。"""
        if self._runner is None:
            raise RuntimeError("RKNN runner is not loaded")
        if self.expected_input_names:
            try:
                ordered_inputs = []
                for index, name in enumerate(self.expected_input_names):
                    array = _to_numpy_array(inputs[name])
                    expected_dtype = self.expected_input_dtypes[index]
                    if expected_dtype is None:
                        raise RuntimeError(f"missing expected dtype for RKNN input '{name}'")
                    if array.dtype != expected_dtype:
                        raise RuntimeError(
                            f"RKNN input dtype mismatch for '{name}': expected {expected_dtype}, got {array.dtype}"
                        )
                    expected_shape = self.expected_input_shapes[index]
                    if array.shape != expected_shape:
                        raise RuntimeError(
                            f"RKNN input shape mismatch for '{name}': expected {expected_shape}, got {array.shape}"
                        )
                    ordered_inputs.append(array)
            except KeyError as exc:
                raise _missing_input_error(inputs, self.expected_input_names) from exc
        else:
            raise RuntimeError("replay sample manifest must provide expected input contract")
        outputs = self._runner.inference(inputs=ordered_inputs)
        return [np.asarray(output) for output in outputs]

    def close(self) -> None:
        """释放 RKNN runtime 资源。"""
        if self._runner is not None:
            release = getattr(self._runner, "release", None)
            if release is not None:
                release()
        self._runner = None


def create_replay_runner(backend: str, *, rknn_runtime_mode: str = "auto", **kwargs: Any) -> ReplayRunner:
    """按 backend 名称创建对应的 replay runner。"""
    normalized = backend.lower()
    if normalized in {"onnx", "onnxruntime"}:
        return OnnxRuntimeRunner(**kwargs)
    if normalized in {"rknn", "rknnruntime", "rknn-lite", "rknnlite"}:
        return RknnRunner(runtime_mode=rknn_runtime_mode, **kwargs)
    raise ValueError(f"unknown replay backend: {backend}")
