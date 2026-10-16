"""Verification helpers for RKNN conversion."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class VerificationResult:
    """Summary of a conversion verification run."""

    sample_count: int
    max_diff: float
    mean_diff: float


def _make_input_array(shape: Iterable[int], seed: int) -> np.ndarray:
    """生成单个固定种子的测试输入。

    Args:
        shape: 输入张量形状。
        seed: 随机种子，用于保证验证样本可复现。

    Returns:
        float32 的随机输入数组。
    """
    rng = np.random.default_rng(seed)
    dims = [int(dim) if int(dim) > 0 else 1 for dim in shape]
    return rng.standard_normal(dims).astype(np.float32)


def build_synthetic_inputs(input_shapes: list[list[int]], sample_seed: int) -> list[np.ndarray]:
    """批量生成模型验证用的合成输入。

    Args:
        input_shapes: 每个模型输入的 shape 列表。
        sample_seed: 本次样本的基础随机种子。

    Returns:
        与输入数量一致的 numpy 数组列表。

    Notes:
        这里不接真实机器人数据，而是用固定种子生成样本，便于重复验证和定位误差。
    """
    return [_make_input_array(shape, sample_seed + index) for index, shape in enumerate(input_shapes)]


def verify_rknn_against_onnx(
    rknn,
    onnx_path: Path,
    input_shapes: list[list[int]],
    sample_count: int = 3,
    atol: float = 1.0e-4,
    rtol: float = 1.0e-4,
) -> VerificationResult:
    """比较 RKNN 和 ONNX Runtime 的输出差异。

    Args:
        rknn: 已初始化 runtime 的 RKNN 对象。
        onnx_path: 参与对比的 ONNX 模型路径，通常是补丁后的临时模型。
        input_shapes: 模型输入 shape 列表。
        sample_count: 生成多少组随机样本用于比较。
        atol: 绝对误差阈值。
        rtol: 相对误差阈值。

    Returns:
        汇总结果，包含样本数、最大差值和平均差值。

    Raises:
        ValueError: 输入数量与 shape 数量不一致时抛出。
        RuntimeError: 任一输出形状不一致或数值差异超限时抛出。
    """
    import onnxruntime as ort

    session = ort.InferenceSession(str(onnx_path))
    input_names = [info.name for info in session.get_inputs()]
    if len(input_names) != len(input_shapes):
        raise ValueError(
            f"input count mismatch: ONNX has {len(input_names)} inputs, "
            f"manifest provides {len(input_shapes)} shapes"
        )

    max_diff = 0.0
    mean_diffs: list[float] = []
    for sample_index in range(sample_count):
        test_inputs = build_synthetic_inputs(input_shapes, sample_seed=sample_index + 1)
        feed = {
            name: np.asarray(value, dtype=np.float32)
            for name, value in zip(input_names, test_inputs, strict=True)
        }

        # RKNN 和 ONNX Runtime 都跑同一组输入，再逐输出比较数值差异。
        rknn_outputs = rknn.inference(inputs=test_inputs)
        onnx_outputs = session.run(None, feed)
        if not rknn_outputs or not onnx_outputs:
            raise RuntimeError("model produced no outputs during verification")
        if len(rknn_outputs) != len(onnx_outputs):
            raise RuntimeError(
                f"output count mismatch: RKNN={len(rknn_outputs)}, ONNX={len(onnx_outputs)}"
            )

        sample_diffs: list[float] = []
        for output_index, (rknn_output, onnx_output) in enumerate(
            zip(rknn_outputs, onnx_outputs, strict=True)
        ):
            rknn_array = np.asarray(rknn_output, dtype=np.float32)
            onnx_array = np.asarray(onnx_output, dtype=np.float32)
            if rknn_array.shape != onnx_array.shape:
                raise RuntimeError(
                    "output shape mismatch at "
                    f"index {output_index}: RKNN={rknn_array.shape}, ONNX={onnx_array.shape}"
                )

            abs_diff = np.abs(rknn_array - onnx_array)
            sample_max = float(np.max(abs_diff))
            sample_mean = float(np.mean(abs_diff))
            if not np.allclose(rknn_array, onnx_array, atol=atol, rtol=rtol):
                raise RuntimeError(
                    f"verification failed on sample {sample_index}, output {output_index}: "
                    f"max diff={sample_max:.6e}, mean diff={sample_mean:.6e}, "
                    f"atol={atol}, rtol={rtol}"
                )
            max_diff = max(max_diff, sample_max)
            sample_diffs.append(sample_mean)

        mean_diffs.append(float(sum(sample_diffs) / max(len(sample_diffs), 1)))

    return VerificationResult(
        sample_count=sample_count,
        max_diff=max_diff,
        mean_diff=float(sum(mean_diffs) / max(len(mean_diffs), 1)),
    )
