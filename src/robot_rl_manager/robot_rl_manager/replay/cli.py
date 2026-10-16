"""Replay 离线验证工具的命令行入口。"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from robot_rl_manager.replay.capture import (
    split_replay_sequence_sample,
    write_replay_sequence_sample,
)
from robot_rl_manager.replay.compare import compare_output_sets
from robot_rl_manager.replay.report import ReplayReport, report_to_json
from robot_rl_manager.replay.manifest import ReplayThresholds
from robot_rl_manager.replay.runner import create_replay_runner
from robot_rl_manager.replay.sequence_manifest import (
    find_sequence_repo_root,
    load_replay_sequence_manifest,
)


def _resolve_path(value: str, default: Path | None = None, *, base_dir: Path | None = None) -> Path:
    """把用户输入的路径解析成绝对路径。

    Args:
        value: 用户显式传入的路径文本。
        default: 未显式传参时使用的默认路径。
        base_dir: 相对路径的解析基准，默认用当前工作目录。

    Notes:
        replay 的 sample / 输出文件允许用仓库相对路径，所以这里必须显式指定
        base_dir，不能盲目依赖当前工作目录。
    """
    base_dir = base_dir or Path.cwd().resolve()
    if value:
        path = Path(value).expanduser()
        if path.is_absolute():
            return path.resolve()
        return (base_dir / path).resolve()
    if default is None:
        raise ValueError("a default path is required when no explicit path is provided")
    default_path = Path(default).expanduser()
    if default_path.is_absolute():
        return default_path.resolve()
    return (base_dir / default_path).resolve()


def _default_outputs_path(sample_path: Path, name: str) -> Path:
    """根据 sample 文件名派生输出文件路径。"""
    return sample_path.with_name(name)


def _default_sample_path(repo_root: Path, model_name: str) -> Path:
    """根据模型名推导标准 replay 序列样本路径。"""
    return repo_root / "config" / "data" / "samples" / model_name / "inputs_seq.npz"


def _is_repo_managed_sample(path: Path, repo_root: Path) -> bool:
    """判断 sample 是否落在仓库内的标准模型目录下。

    这里把 `repo_root/config/data/samples/` 视作受版本管理的标准样本区，capture 不允许直接覆盖。
    """
    managed_dir = (repo_root / "config" / "data" / "samples").resolve()
    return path.resolve().is_relative_to(managed_dir)


def _save_output_arrays(path: Path, outputs: list[np.ndarray]) -> None:
    """把模型输出按 ``output_0`` / ``output_1`` ... 的形式保存成 npz。"""
    payload = {f"output_{index}": np.asarray(output) for index, output in enumerate(outputs)}
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(path, **payload)


def _format_metric(value: float | None) -> str:
    """把数值指标格式化成便于终端阅读的短文本。"""
    if value is None:
        return "n/a"
    return f"{value:.6g}"


def _thresholds_pass(
    comparison,
    thresholds: ReplayThresholds,
) -> bool:
    """判断对比结果是否满足当前 replay 阈值。

    这里保留 strict allclose 作为参考，但最终放行标准使用更适合 RKNN 的
    误差阈值组合，避免 float16 路径下的正常偏差被误判为失败。
    """
    if not comparison.shape_match:
        return False
    if comparison.max_abs is None or comparison.mean_abs is None or comparison.p95_abs is None:
        return False
    return (
        comparison.max_abs <= thresholds.max_abs
        and comparison.mean_abs <= thresholds.mean_abs
        and comparison.p95_abs <= thresholds.p95_abs
    )


def _print_compare_summary(
    *,
    model_name: str,
    sample_path: Path,
    comparison,
    thresholds: ReplayThresholds,
    baseline_output_path: Path,
    rknn_output_path: Path,
    report_path: Path,
) -> None:
    """向终端打印 replay compare 的结论摘要。"""
    judged_pass = _thresholds_pass(comparison, thresholds)
    verdict = "PASS" if judged_pass else "FAIL"
    print(f"[replay][compare] model={model_name}")
    print(f"[replay][compare] verdict={verdict}")
    print(
        "[replay][compare] "
        f"shape_match={'yes' if comparison.shape_match else 'no'} "
        f"strict_allclose={'yes' if comparison.allclose else 'no'}"
    )
    print(
        "[replay][compare] thresholds="
        f"max_abs<={_format_metric(thresholds.max_abs)} "
        f"mean_abs<={_format_metric(thresholds.mean_abs)} "
        f"p95_abs<={_format_metric(thresholds.p95_abs)}"
    )
    print(
        "[replay][compare] metrics="
        f"max_abs={_format_metric(comparison.max_abs)} "
        f"mean_abs={_format_metric(comparison.mean_abs)} "
        f"p95_abs={_format_metric(comparison.p95_abs)}"
    )
    print(f"[replay][compare] sample={sample_path}")
    print(f"[replay][compare] baseline_output={baseline_output_path}")
    print(f"[replay][compare] rknn_output={rknn_output_path}")
    print(f"[replay][compare] report={report_path}")


def _stack_sequence_outputs(sequence_outputs: list[list[np.ndarray]]) -> list[np.ndarray]:
    """把逐帧输出堆叠回按输出 index 分组的序列数组。"""
    if not sequence_outputs:
        raise ValueError("replay output sequence cannot be empty")
    output_count = len(sequence_outputs[0])
    for frame_index, outputs in enumerate(sequence_outputs[1:], start=1):
        if len(outputs) != output_count:
            raise ValueError(
                f"output count mismatch at frame {frame_index}: expected {output_count}, got {len(outputs)}"
            )

    stacked: list[np.ndarray] = []
    for output_index in range(output_count):
        stacked.append(np.stack([np.asarray(frame[output_index]) for frame in sequence_outputs], axis=0))
    return stacked


def _run_model_sequence(
    model_path: Path,
    backend: str,
    sequence_inputs: list[dict[str, np.ndarray]],
    input_specs,
) -> list[np.ndarray]:
    """按时间顺序逐帧执行基线或 RKNN 推理。"""
    if not sequence_inputs:
        raise ValueError("replay input sequence cannot be empty")
    expected_input_names = tuple(spec.name for spec in input_specs)
    expected_input_dtypes = tuple(np.dtype(spec.dtype) for spec in input_specs)
    expected_input_shapes = tuple(tuple(int(dim) for dim in spec.shape) for spec in input_specs)
    runner = create_replay_runner(
        backend,
        expected_input_names=expected_input_names,
        expected_input_dtypes=expected_input_dtypes,
        expected_input_shapes=expected_input_shapes,
    )
    try:
        runner.load(model_path)
        sequence_outputs: list[list[np.ndarray]] = []
        for frame_inputs in sequence_inputs:
            sequence_outputs.append(runner.run(frame_inputs))
        return _stack_sequence_outputs(sequence_outputs)
    finally:
        runner.close()


def _load_replay_sample(path: Path) -> dict[str, np.ndarray]:
    """从 npz 中加载 replay sample。"""
    with np.load(path, allow_pickle=False) as data:
        return {name: np.asarray(data[name]) for name in data.files}


def _load_replay_sequence_sample(path: Path) -> list[dict[str, np.ndarray]]:
    """从 npz 中加载 replay 序列 sample。"""
    return split_replay_sequence_sample(_load_replay_sample(path))


def build_parser() -> argparse.ArgumentParser:
    """构造 replay 命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        description="Replay ONNX and RKNN outputs for RL models."
    )
    parser.add_argument("--model", help="Name of the model to replay.")
    parser.add_argument(
        "--manifest",
        default="",
        help="Sequence manifest path, defaults to manifest.yaml beside the sample file.",
    )
    parser.add_argument(
        "--sample",
        default="",
        help="Input sample npz path, defaults to config/data/samples/<model>/inputs_seq.npz.",
    )
    parser.add_argument(
        "--baseline-output",
        default="",
        help="Baseline output npz path.",
    )
    parser.add_argument(
        "--rknn-output",
        default="",
        help="RKNN output npz path.",
    )
    parser.add_argument(
        "--run-baseline",
        action="store_true",
        help="Run the baseline backend and write baseline_outputs.npz.",
    )
    parser.add_argument(
        "--run-rknn",
        action="store_true",
        help="Run the RKNN backend and write rknn_outputs.npz.",
    )
    parser.add_argument(
        "--compare",
        action="store_true",
        help="Compare baseline and RKNN outputs and write report.json.",
    )
    parser.add_argument(
        "--capture",
        action="store_true",
        help="Normalize and write replay sample inputs.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """解析参数并编排 replay 的离线文件流程。"""
    parser = build_parser()
    args = parser.parse_args(argv)

    repo_root = find_sequence_repo_root(Path(__file__))

    action_requested = bool(args.capture or args.run_baseline or args.run_rknn or args.compare)
    if not action_requested:
        return 0

    if not args.model:
        parser.error("--model is required when replay actions are requested")

    default_sample_path = _default_sample_path(repo_root, args.model)
    # sample 支持仓库相对路径，例如 config/data/samples/t4_heart/inputs_seq.npz。
    sample_path = _resolve_path(args.sample, default=default_sample_path, base_dir=repo_root)
    manifest_path = _resolve_path(
        args.manifest,
        default=sample_path.parent / "manifest.yaml",
        base_dir=repo_root,
    )
    if not manifest_path.exists():
        parser.error(f"sequence manifest not found: {manifest_path}")

    manifest = load_replay_sequence_manifest(manifest_path)
    if manifest.model.name != args.model:
        parser.error(
            f"sequence manifest model mismatch: expected {args.model}, got {manifest.model.name}"
        )
    if manifest.sample.path != sample_path:
        parser.error(
            f"sequence manifest sample mismatch: expected {sample_path}, got {manifest.sample.path}"
        )
    sample_sequence: list[dict[str, np.ndarray]] | None = None

    if args.capture and _is_repo_managed_sample(sample_path, repo_root):
        parser.error(
            f"--capture must not overwrite the managed sample under {repo_root / 'config' / 'data' / 'samples'}; "
            "write to a temporary path or update the source sample intentionally"
        )

    if args.capture or args.run_baseline or args.run_rknn:
        sample_sequence = split_replay_sequence_sample(_load_replay_sample(sample_path))
        if manifest.sample.inputs and tuple(sample_sequence[0].keys()) != tuple(spec.name for spec in manifest.sample.inputs):
            parser.error(
                f"sequence manifest input names mismatch sample keys: "
                f"expected {[spec.name for spec in manifest.sample.inputs]}, got {list(sample_sequence[0].keys())}"
            )

    if args.capture:
        # capture 只做归一化和重写，不生成新的轨迹样本。
        write_replay_sequence_sample(sample_path, sample_sequence or [])
        if not (args.run_baseline or args.run_rknn or args.compare):
            return 0

    baseline_output_path = _resolve_path(
        args.baseline_output,
        _default_outputs_path(sample_path, "baseline_outputs.npz"),
        base_dir=repo_root,
    )
    rknn_output_path = _resolve_path(
        args.rknn_output,
        _default_outputs_path(sample_path, "rknn_outputs.npz"),
        base_dir=repo_root,
    )
    report_path = _default_outputs_path(sample_path, "report.json")

    if args.run_baseline:
        if sample_sequence is None:
            sample_sequence = _load_replay_sequence_sample(sample_path)
        baseline_outputs = _run_model_sequence(
            manifest.model.onnx_path,
            manifest.runtime.baseline_backend,
            sample_sequence,
            manifest.sample.inputs,
        )
        _save_output_arrays(baseline_output_path, baseline_outputs)

    if args.run_rknn:
        if sample_sequence is None:
            sample_sequence = _load_replay_sequence_sample(sample_path)
        rknn_outputs = _run_model_sequence(
            manifest.model.rknn_path,
            manifest.runtime.rknn_backend,
            sample_sequence,
            manifest.sample.inputs,
        )
        _save_output_arrays(rknn_output_path, rknn_outputs)

    if args.compare:
        # compare 直接对比已经落盘的 baseline / rknn 输出文件。
        baseline_outputs = _load_replay_sample(baseline_output_path)
        rknn_outputs = _load_replay_sample(rknn_output_path)
        comparison = compare_output_sets(
            baseline_outputs,
            rknn_outputs,
            atol=manifest.runtime.verify.atol,
            rtol=manifest.runtime.verify.rtol,
        )
        report = ReplayReport(
            model_name=manifest.model.name,
            baseline_backend=manifest.runtime.baseline_backend,
            rknn_backend=manifest.runtime.rknn_backend,
            sample_path=sample_path,
            metadata={
                "manifest": str(manifest.path),
                "baseline_output": str(baseline_output_path),
                "rknn_output": str(rknn_output_path),
            },
            comparison=comparison,
        )
        report_path.write_text(report_to_json(report), encoding="utf-8")
        _print_compare_summary(
            model_name=manifest.model.name,
            sample_path=sample_path,
            comparison=comparison,
            thresholds=manifest.runtime.verify,
            baseline_output_path=baseline_output_path,
            rknn_output_path=rknn_output_path,
            report_path=report_path,
        )
    return 0
