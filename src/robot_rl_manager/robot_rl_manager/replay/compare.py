"""Replay 输出的数值对比工具。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np

ArrayInput = np.ndarray | Sequence[float] | Sequence[int]
OutputCollection = Mapping[str, ArrayInput] | Sequence[ArrayInput]


@dataclass(frozen=True)
class ArrayComparisonResult:
    """单个输出张量对比的统计结果。"""

    left_shape: tuple[int, ...]
    right_shape: tuple[int, ...]
    shape_match: bool
    allclose: bool
    max_abs: float | None
    mean_abs: float | None
    p95_abs: float | None
    num_elements: int


@dataclass(frozen=True)
class OutputComparisonResult:
    """一个命名输出或按序输出的对比结果。"""

    key: str
    left_index: int | None
    right_index: int | None
    result: ArrayComparisonResult


@dataclass(frozen=True)
class OutputSetComparisonResult:
    """多路输出的聚合对比结果。"""

    comparisons: tuple[OutputComparisonResult, ...]
    missing_left: tuple[str, ...]
    missing_right: tuple[str, ...]
    shape_match: bool
    allclose: bool
    max_abs: float | None
    mean_abs: float | None
    p95_abs: float | None


def _coerce_array(value: ArrayInput) -> np.ndarray:
    """把 list / tuple / ndarray 统一转成 NumPy 数组。"""
    return np.asarray(value)


def _format_key(index: int, name: str | None = None) -> str:
    """为未命名输出生成稳定键名。"""
    return name if name is not None else str(index)


def _pairwise_metrics(left: np.ndarray, right: np.ndarray) -> tuple[float, float, float, int]:
    """计算两组等形状数组的绝对误差统计。"""
    abs_diff = np.abs(left.astype(np.float64, copy=False) - right.astype(np.float64, copy=False))
    flat = np.ravel(abs_diff)
    count = int(flat.size)
    if count == 0:
        return 0.0, 0.0, 0.0, 0
    max_abs = float(np.max(flat))
    mean_abs = float(np.mean(flat))
    p95_abs = float(np.percentile(flat, 95))
    return max_abs, mean_abs, p95_abs, count


def compare_output_arrays(
    left: ArrayInput,
    right: ArrayInput,
    atol: float,
    rtol: float,
) -> ArrayComparisonResult:
    """对比两个数组，并返回形状和误差统计。

    只要 shape 不一致，就直接标记失败，不继续做逐元素误差统计。
    """
    left_array = _coerce_array(left)
    right_array = _coerce_array(right)
    shape_match = left_array.shape == right_array.shape
    if not shape_match:
        return ArrayComparisonResult(
            left_shape=left_array.shape,
            right_shape=right_array.shape,
            shape_match=False,
            allclose=False,
            max_abs=None,
            mean_abs=None,
            p95_abs=None,
            num_elements=0,
        )

    max_abs, mean_abs, p95_abs, num_elements = _pairwise_metrics(left_array, right_array)
    allclose = bool(np.allclose(left_array, right_array, atol=atol, rtol=rtol))
    return ArrayComparisonResult(
        left_shape=left_array.shape,
        right_shape=right_array.shape,
        shape_match=True,
        allclose=allclose,
        max_abs=max_abs,
        mean_abs=mean_abs,
        p95_abs=p95_abs,
        num_elements=num_elements,
    )


def _normalize_collection(
    outputs: OutputCollection,
    output_names: Sequence[str] | None = None,
) -> list[tuple[str, int, np.ndarray]]:
    """把映射或序列输出统一归一化成可比对的三元组列表。

    返回值中的每一项都是 ``(key, index, array)``，方便后续按 key 对齐。
    """
    if isinstance(outputs, Mapping):
        normalized: list[tuple[str, int, np.ndarray]] = []
        for index, (name, value) in enumerate(outputs.items()):
            normalized.append((str(name), index, _coerce_array(value)))
        return normalized

    arrays = [_coerce_array(value) for value in outputs]
    if output_names is not None and len(output_names) != len(arrays):
        raise ValueError(
            f"output_names length mismatch: {len(output_names)} names for {len(arrays)} outputs"
        )
    normalized = []
    seen_names: set[str] = set()
    for index, value in enumerate(arrays):
        name = output_names[index] if output_names is not None else None
        key = _format_key(index, name)
        if key in seen_names:
            # 这里必须显式拒绝重复输出名，避免后面的字典覆盖把前一个输出悄悄吃掉。
            raise ValueError(f"duplicate output name detected: {key}")
        seen_names.add(key)
        normalized.append((key, index, value))
    return normalized


def compare_output_sets(
    left: OutputCollection,
    right: OutputCollection,
    atol: float,
    rtol: float,
    output_names: Sequence[str] | None = None,
) -> OutputSetComparisonResult:
    """对比多路输出，并聚合成一份结果。

    这里既支持 dict 风格的命名输出，也支持 list/tuple 风格的顺序输出。
    """
    left_items = _normalize_collection(left, output_names)
    right_items = _normalize_collection(right, output_names)

    left_map = {key: (index, array) for key, index, array in left_items}
    right_map = {key: (index, array) for key, index, array in right_items}

    shared_keys = [key for key, _, _ in left_items if key in right_map]
    comparisons: list[OutputComparisonResult] = []
    collected_diffs: list[np.ndarray] = []
    for key in shared_keys:
        left_index, left_array = left_map[key]
        right_index, right_array = right_map[key]
        result = compare_output_arrays(left_array, right_array, atol=atol, rtol=rtol)
        comparisons.append(
            OutputComparisonResult(
                key=key,
                left_index=left_index,
                right_index=right_index,
                result=result,
            )
        )
        if result.shape_match:
            diff = np.abs(
                left_array.astype(np.float64, copy=False)
                - right_array.astype(np.float64, copy=False)
            )
            flat_diff = np.ravel(diff)
            if flat_diff.size:
                collected_diffs.append(flat_diff)

    missing_left = tuple(key for key in right_map.keys() if key not in left_map)
    missing_right = tuple(key for key in left_map.keys() if key not in right_map)
    shape_match = (
        not missing_left
        and not missing_right
        and all(result.result.shape_match for result in comparisons)
    )
    allclose = (
        not missing_left
        and not missing_right
        and all(result.result.allclose for result in comparisons)
    )

    if not missing_left and not missing_right and all(result.result.shape_match for result in comparisons):
        if collected_diffs:
            flat_diffs = np.concatenate(collected_diffs)
            aggregate_max = float(np.max(flat_diffs))
            aggregate_mean = float(np.mean(flat_diffs))
            aggregate_p95 = float(np.percentile(flat_diffs, 95))
        else:
            aggregate_max = 0.0
            aggregate_mean = 0.0
            aggregate_p95 = 0.0
    else:
        aggregate_max = None
        aggregate_mean = None
        aggregate_p95 = None

    return OutputSetComparisonResult(
        comparisons=tuple(comparisons),
        missing_left=missing_left,
        missing_right=missing_right,
        shape_match=shape_match,
        allclose=allclose,
        max_abs=aggregate_max,
        mean_abs=aggregate_mean,
        p95_abs=aggregate_p95,
    )
