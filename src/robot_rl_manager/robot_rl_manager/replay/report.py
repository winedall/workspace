"""Replay 对比结果的纯序列化工具。"""

from __future__ import annotations

from dataclasses import dataclass, fields, is_dataclass
from pathlib import Path
from typing import Any, Mapping
import json
import math

from robot_rl_manager.replay.compare import OutputComparisonResult, OutputSetComparisonResult


@dataclass(frozen=True)
class ReplayReport:
    """可直接序列化的 replay 报告结构。"""

    model_name: str
    comparison: OutputSetComparisonResult
    baseline_backend: str | None = None
    rknn_backend: str | None = None
    sample_path: str | Path | None = None
    metadata: Mapping[str, Any] | None = None


def _jsonable(value: Any) -> Any:
    """把 dataclass / Path / Mapping 等对象递归转成 JSON 友好结构。"""
    if is_dataclass(value):
        return {field.name: _jsonable(getattr(value, field.name)) for field in fields(value)}
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_jsonable(item) for item in value]
    if isinstance(value, list):
        return [_jsonable(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def report_to_dict(report: ReplayReport) -> dict[str, Any]:
    """把报告对象转换成普通 Python 数据结构。"""
    return _jsonable(report)


def report_to_json(report: ReplayReport, *, indent: int = 2) -> str:
    """把报告序列化成 JSON 字符串。"""
    return json.dumps(report_to_dict(report), indent=indent, sort_keys=True)


def _format_metric(value: float | None) -> str:
    """把数值指标格式化成易读文本。"""
    if value is None:
        return "n/a"
    return f"{value:.6g}"


def _format_shape(shape: tuple[int, ...]) -> str:
    """把 shape 格式化成标准括号字符串。"""
    return "(" + ", ".join(str(dim) for dim in shape) + ")"


def _escape_markdown(value: Any) -> str:
    """对 Markdown 特殊字符做最小转义，避免报告被截断或错排版。"""
    text = str(value)
    return text.replace("\\", "\\\\").replace("|", "\\|").replace("`", "\\`").replace("\n", "<br>")


def _comparison_table_row(comparison: OutputComparisonResult) -> str:
    """生成单行 Markdown 表格内容。"""
    result = comparison.result
    return (
        f"| {_escape_markdown(comparison.key)} | {comparison.left_index} | {comparison.right_index} | "
        f"{'yes' if result.shape_match else 'no'} | {'yes' if result.allclose else 'no'} | "
        f"{_format_metric(result.max_abs)} | {_format_metric(result.mean_abs)} | "
        f"{_format_metric(result.p95_abs)} | {_format_shape(result.left_shape)} | "
        f"{_format_shape(result.right_shape)} |"
    )


def report_to_markdown(report: ReplayReport) -> str:
    """把报告序列化成简洁的 Markdown 摘要。

    这个输出主要给人工调试和评审使用，不做文件写入。
    """
    comparison = report.comparison
    lines = [
        f"# Replay Report: {_escape_markdown(report.model_name)}",
        "",
        "## Summary",
        "",
        f"- Baseline backend: {_escape_markdown(report.baseline_backend or 'n/a')}",
        f"- RKNN backend: {_escape_markdown(report.rknn_backend or 'n/a')}",
        f"- Sample path: {_escape_markdown(report.sample_path or 'n/a')}",
        f"- Shape match: {'yes' if comparison.shape_match else 'no'}",
        f"- All close: {'yes' if comparison.allclose else 'no'}",
        f"- Max abs: {_format_metric(comparison.max_abs)}",
        f"- Mean abs: {_format_metric(comparison.mean_abs)}",
        f"- P95 abs: {_format_metric(comparison.p95_abs)}",
    ]
    if report.metadata:
        lines.extend(["", "## Metadata", ""])
        for key, value in report.metadata.items():
            lines.append(f"- {_escape_markdown(key)}: {_escape_markdown(value)}")

    lines.extend(
        [
            "",
            "## Outputs",
            "",
            "| key | left idx | right idx | shape match | allclose | max abs | mean abs | p95 abs | left shape | right shape |",
            "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
        ]
    )
    for item in comparison.comparisons:
        lines.append(_comparison_table_row(item))

    if comparison.missing_left or comparison.missing_right:
        lines.extend(["", "## Missing Outputs", ""])
        if comparison.missing_left:
            lines.append(
                f"- Missing from left: {', '.join(_escape_markdown(item) for item in comparison.missing_left)}"
            )
        if comparison.missing_right:
            lines.append(
                f"- Missing from right: {', '.join(_escape_markdown(item) for item in comparison.missing_right)}"
            )

    return "\n".join(lines).rstrip() + "\n"
