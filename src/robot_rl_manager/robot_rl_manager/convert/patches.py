"""ONNX graph patch helpers for RKNN conversion."""

from __future__ import annotations

import tempfile
from pathlib import Path
from typing import Callable, Iterable

import numpy as np


PatchFn = Callable[[Path], Path]


def _replace_graph_node_with_constant(model, node_index: int, output_name: str, array: np.ndarray) -> None:
    """把指定节点替换成 Constant 常量节点。

    Args:
        model: 已加载的 ONNX ModelProto 对象。
        node_index: 要替换的节点在 graph.node 里的索引。
        output_name: 原节点输出名，替换后需要继续沿用它。
        array: 要固化成常量的 numpy 数组。

    Notes:
        这里保留输出名不变，是为了不破坏后续节点的输入引用关系。
    """
    from onnx import helper, numpy_helper

    const_node = helper.make_node(
        "Constant",
        inputs=[],
        outputs=[output_name],
        value=numpy_helper.from_array(array, name=output_name),
    )
    model.graph.node.remove(model.graph.node[node_index])
    model.graph.node.insert(node_index, const_node)


def random_normal_like_to_zero(onnx_path: Path) -> Path:
    """Replace RandomNormalLike nodes with deterministic zero constants.

    Args:
        onnx_path: 原始 ONNX 模型路径。

    Returns:
        若图中存在需要修补的节点，返回临时修补后的 ONNX 路径；
        否则直接返回原始路径。

    Notes:
        旧版 RK3588 脚本里对部分模型做过同类修补，这里把它抽成标准化 patch。
        这个 patch 的目标不是“改模型语义”，而是去掉部署时不该出现的随机算子。
    """
    import onnx

    model = onnx.load(str(onnx_path))
    try:
        model = onnx.shape_inference.infer_shapes(model, strict_mode=False)
    except Exception:
        # 如果 shape inference 失败，就不要硬猜图结构，直接保留原图交给 RKNN 报真实错误。
        return onnx_path

    tensor_shapes: dict[str, list[int]] = {}
    for input_info in model.graph.input:
        shape = [dim.dim_value for dim in input_info.type.tensor_type.shape.dim]
        tensor_shapes[input_info.name] = shape
    for value_info in model.graph.value_info:
        shape = [dim.dim_value for dim in value_info.type.tensor_type.shape.dim]
        tensor_shapes[value_info.name] = shape

    replacements: list[tuple[int, str, np.ndarray]] = []
    for node_index, node in enumerate(model.graph.node):
        if node.op_type != "RandomNormalLike":
            continue
        if not node.input:
            continue
        input_name = node.input[0]
        if input_name not in tensor_shapes:
            continue
        shape = [dim if dim > 0 else 1 for dim in tensor_shapes[input_name]]
        zeros = np.zeros(shape, dtype=np.float32)
        replacements.append((node_index, node.output[0], zeros))

    if not replacements:
        return onnx_path

    # 逆序替换，避免前面的节点删除后把后续索引冲掉。
    for node_index, output_name, array in reversed(replacements):
        _replace_graph_node_with_constant(model, node_index, output_name, array)

    with tempfile.NamedTemporaryFile(suffix=".onnx", delete=False) as handle:
        temp_file = Path(handle.name)
    onnx.save(model, str(temp_file))
    return temp_file


PATCH_REGISTRY: dict[str, PatchFn] = {
    "random_normal_like_to_zero": random_normal_like_to_zero,
}


def apply_patches(onnx_path: Path, patch_names: Iterable[str]) -> tuple[Path, list[Path]]:
    """按顺序应用多个 patch。

    Args:
        onnx_path: 原始 ONNX 模型路径。
        patch_names: manifest 里声明的 patch 名称列表。

    Returns:
        一个二元组：
        - 最终可用于 RKNN 转换的 ONNX 路径
        - 所有临时补丁文件路径，用于后续清理

    Raises:
        ValueError: 当 manifest 引用了不存在的 patch 名称时抛出。
    """
    current = onnx_path
    temp_paths: list[Path] = []
    for patch_name in patch_names:
        if patch_name not in PATCH_REGISTRY:
            raise ValueError(f"unknown patch: {patch_name}")
        patched = PATCH_REGISTRY[patch_name](current)
        if patched != current:
            temp_paths.append(patched)
            current = patched
    return current, temp_paths
