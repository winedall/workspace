"""Replay 序列样本的 manifest 读写工具。"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any, Mapping

import numpy as np
import yaml

from robot_rl_manager.replay.manifest import ReplayThresholds

if TYPE_CHECKING:
    from robot_rl_manager.model_registry import RegistryRlModelInfo
else:
    RegistryRlModelInfo = Any

DEFAULT_TARGET_PLATFORM = "rk3588"
DEFAULT_BASELINE_BACKEND = "onnxruntime"
DEFAULT_RKNN_BACKEND = "rknnlite"
DEFAULT_SAMPLE_KIND = "sequence"


@dataclass(frozen=True)
class ReplaySequenceInputSpec:
    """单个模型输入在序列样本中的契约描述。"""

    name: str
    dtype: str
    shape: tuple[int, ...]


@dataclass(frozen=True)
class ReplaySequenceModelSpec:
    """序列样本关联的模型信息。"""

    name: str
    config_path: Path
    onnx_path: Path
    rknn_path: Path
    model_type: str
    control_frequency_hz: float
    observation_dim: int
    action_dim: int


@dataclass(frozen=True)
class ReplaySequenceSampleSpec:
    """序列样本信息。"""

    path: Path
    kind: str
    sequence_length: int
    inputs: tuple[ReplaySequenceInputSpec, ...]


@dataclass(frozen=True)
class ReplaySequenceRuntimeSpec:
    """replay 运行时默认参数。"""

    target_platform: str = DEFAULT_TARGET_PLATFORM
    baseline_backend: str = DEFAULT_BASELINE_BACKEND
    rknn_backend: str = DEFAULT_RKNN_BACKEND
    verify: ReplayThresholds = field(default_factory=ReplayThresholds)


@dataclass(frozen=True)
class ReplaySequenceManifest:
    """完整的序列样本 manifest。"""

    path: Path
    repo_root: Path
    model: ReplaySequenceModelSpec
    sample: ReplaySequenceSampleSpec
    runtime: ReplaySequenceRuntimeSpec


def find_sequence_repo_root(start: Path | None = None) -> Path:
    """从给定路径向上查找 replay 约定的仓库根目录。

    Args:
        start: 起始路径。通常传当前模块路径、样本目录或仓库任意子路径。

    Returns:
        仓库根目录，也就是同时包含 ``config/data/samples`` 和
        ``src/robot_rl_manager`` 的目录。
    """
    current = (start or Path(__file__)).resolve()
    for candidate in [current, *current.parents]:
        if (candidate / "config").is_dir() and (candidate / "src" / "robot_rl_manager").is_dir():
            return candidate
    raise FileNotFoundError(f"could not locate replay repo root from {current}")


def _resolve_repo_root(manifest_path: Path) -> Path:
    """从 manifest 文件路径反推仓库根目录。"""
    manifest_path = manifest_path.resolve()
    for candidate in manifest_path.parents:
        if (candidate / "config").is_dir() and (candidate / "src" / "robot_rl_manager").is_dir():
            return candidate
    raise FileNotFoundError(f"could not locate robot_rl_manager repo root for {manifest_path}")


def _repo_relative_path(repo_root: Path, value: str | Path) -> str:
    """把路径转换成 repo-relative 文本，无法转换时保留绝对路径。"""
    path = Path(value).expanduser().resolve()
    try:
        return str(path.relative_to(repo_root))
    except ValueError:
        return str(path)


def _resolve_path(repo_root: Path, value: str | Path) -> Path:
    """解析 manifest 中的路径文本。"""
    path = Path(value).expanduser()
    if path.is_absolute():
        return path.resolve()
    return (repo_root / path).resolve()


def _require_mapping(
    value: Any,
    field_name: str,
    manifest_path: Path,
    *,
    required: bool = False,
) -> Mapping[str, Any]:
    """校验 manifest 字段必须是 mapping。"""
    if value is None:
        if required:
            raise ValueError(f"sequence manifest field '{field_name}' is required: {manifest_path}")
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"sequence manifest field '{field_name}' must be a mapping: {manifest_path}")
    return value


def _normalize_frame(sample: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
    """把录制到的单帧样本归一化为序列帧格式。"""
    normalized: dict[str, np.ndarray] = {}
    for key, value in sample.items():
        array = np.asarray(value)
        if array.dtype.kind == "f" and array.dtype != np.float32:
            array = array.astype(np.float32, copy=False)
        array = np.expand_dims(array, axis=0)
        normalized[str(key)] = array
    return normalized


def build_replay_sequence_manifest(
    *,
    repo_root: Path,
    sample_path: Path,
    model_info: RegistryRlModelInfo,
    frames: list[Mapping[str, np.ndarray]],
    sequence_length: int,
    runtime: ReplaySequenceRuntimeSpec | None = None,
) -> ReplaySequenceManifest:
    """根据当前录制帧和模型元数据构造 manifest。"""
    if not frames:
        raise ValueError("replay sequence frames cannot be empty")
    normalized_frames = [_normalize_frame(frame) for frame in frames]
    first_frame = normalized_frames[0]
    input_specs = tuple(
        ReplaySequenceInputSpec(
            name=name,
            dtype=str(np.asarray(array).dtype),
            shape=tuple(int(dim) for dim in np.asarray(array).shape),
        )
        for name, array in first_frame.items()
    )
    model_spec = ReplaySequenceModelSpec(
        name=str(model_info.name),
        config_path=Path(str(model_info.config_path)),
        onnx_path=Path(str(model_info.file)),
        rknn_path=Path(str(Path(str(model_info.file)).with_suffix(".rknn"))),
        model_type=str(model_info.model_type),
        control_frequency_hz=float(model_info.control_frequency_hz),
        observation_dim=int(model_info.observation_dim),
        action_dim=int(model_info.action_dim),
    )
    sample_spec = ReplaySequenceSampleSpec(
        path=Path(sample_path),
        kind=DEFAULT_SAMPLE_KIND,
        sequence_length=int(sequence_length),
        inputs=input_specs,
    )
    return ReplaySequenceManifest(
        path=Path(sample_path).parent / "manifest.yaml",
        repo_root=Path(repo_root),
        model=model_spec,
        sample=sample_spec,
        runtime=runtime or ReplaySequenceRuntimeSpec(),
    )


def write_replay_sequence_manifest(path: Path, manifest: ReplaySequenceManifest) -> None:
    """把序列样本 manifest 写成 YAML。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    repo_root = manifest.repo_root
    payload = {
        "model": {
            "name": manifest.model.name,
            "config_path": _repo_relative_path(repo_root, manifest.model.config_path),
            "onnx_path": _repo_relative_path(repo_root, manifest.model.onnx_path),
            "rknn_path": _repo_relative_path(repo_root, manifest.model.rknn_path),
            "model_type": manifest.model.model_type,
            "control_frequency_hz": manifest.model.control_frequency_hz,
            "observation_dim": manifest.model.observation_dim,
            "action_dim": manifest.model.action_dim,
        },
        "sample": {
            "kind": manifest.sample.kind,
            "path": _repo_relative_path(repo_root, manifest.sample.path),
            "sequence_length": manifest.sample.sequence_length,
            "inputs": [
                {
                    "name": spec.name,
                    "dtype": spec.dtype,
                    "shape": [int(dim) for dim in spec.shape],
                }
                for spec in manifest.sample.inputs
            ],
        },
        "runtime": {
            "target_platform": manifest.runtime.target_platform,
            "baseline_backend": manifest.runtime.baseline_backend,
            "rknn_backend": manifest.runtime.rknn_backend,
            "verify": {
                "atol": manifest.runtime.verify.atol,
                "rtol": manifest.runtime.verify.rtol,
                "max_abs": manifest.runtime.verify.max_abs,
                "mean_abs": manifest.runtime.verify.mean_abs,
                "p95_abs": manifest.runtime.verify.p95_abs,
            },
        },
    }
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(payload, stream, sort_keys=False, allow_unicode=True)


def load_replay_sequence_manifest(path: Path) -> ReplaySequenceManifest:
    """加载并严格校验序列样本 manifest。"""
    manifest_path = path.expanduser().resolve()
    raw = manifest_path.read_text(encoding="utf-8")
    data = yaml.safe_load(raw) or {}
    if not isinstance(data, dict):
        raise ValueError(f"sequence manifest must be a mapping: {manifest_path}")

    repo_root = _resolve_repo_root(manifest_path)

    model_data = _require_mapping(data.get("model"), "model", manifest_path, required=True)
    sample_data = _require_mapping(data.get("sample"), "sample", manifest_path, required=True)
    runtime_data = _require_mapping(data.get("runtime"), "runtime", manifest_path)
    verify_data = _require_mapping(runtime_data.get("verify"), "runtime.verify", manifest_path)

    input_items = sample_data.get("inputs")
    if not isinstance(input_items, list) or not input_items:
        raise ValueError(f"sequence manifest field 'sample.inputs' must be a non-empty list: {manifest_path}")

    inputs: list[ReplaySequenceInputSpec] = []
    for index, item in enumerate(input_items):
        if not isinstance(item, dict):
            raise ValueError(
                f"sequence manifest field 'sample.inputs[{index}]' must be a mapping: {manifest_path}"
            )
        name = str(item.get("name", "")).strip()
        dtype = str(item.get("dtype", "")).strip()
        shape_value = item.get("shape")
        if not name:
            raise ValueError(f"sequence manifest input[{index}] name is required: {manifest_path}")
        if not dtype:
            raise ValueError(f"sequence manifest input[{index}] dtype is required: {manifest_path}")
        if not isinstance(shape_value, list) or not shape_value:
            raise ValueError(f"sequence manifest input[{index}] shape is required: {manifest_path}")
        inputs.append(
            ReplaySequenceInputSpec(
                name=name,
                dtype=dtype,
                shape=tuple(int(dim) for dim in shape_value),
            )
        )

    sample_kind = str(sample_data.get("kind", "")).strip()
    if sample_kind != DEFAULT_SAMPLE_KIND:
        raise ValueError(
            f"sequence manifest sample.kind must be '{DEFAULT_SAMPLE_KIND}', got '{sample_kind}': {manifest_path}"
        )

    model = ReplaySequenceModelSpec(
        name=str(model_data.get("name", "")).strip(),
        config_path=_resolve_path(repo_root, model_data.get("config_path", "")),
        onnx_path=_resolve_path(repo_root, model_data.get("onnx_path", "")),
        rknn_path=_resolve_path(repo_root, model_data.get("rknn_path", "")),
        model_type=str(model_data.get("model_type", "")).strip(),
        control_frequency_hz=float(model_data.get("control_frequency_hz", 0.0)),
        observation_dim=int(model_data.get("observation_dim", 0)),
        action_dim=int(model_data.get("action_dim", 0)),
    )
    if not model.name:
        raise ValueError(f"sequence manifest field 'model.name' is required: {manifest_path}")

    sample_path_value = sample_data.get("path")
    if not sample_path_value:
        raise ValueError(f"sequence manifest field 'sample.path' is required: {manifest_path}")
    sample_path = _resolve_path(repo_root, sample_path_value)

    sample = ReplaySequenceSampleSpec(
        path=sample_path,
        kind=sample_kind,
        sequence_length=int(sample_data.get("sequence_length", 0)),
        inputs=tuple(inputs),
    )
    if sample.sequence_length <= 0:
        raise ValueError(f"sequence manifest field 'sample.sequence_length' must be > 0: {manifest_path}")

    runtime = ReplaySequenceRuntimeSpec(
        target_platform=str(runtime_data.get("target_platform", DEFAULT_TARGET_PLATFORM)),
        baseline_backend=str(runtime_data.get("baseline_backend", DEFAULT_BASELINE_BACKEND)),
        rknn_backend=str(runtime_data.get("rknn_backend", DEFAULT_RKNN_BACKEND)),
        verify=ReplayThresholds(
            atol=float(verify_data.get("atol", ReplayThresholds().atol)),
            rtol=float(verify_data.get("rtol", ReplayThresholds().rtol)),
            max_abs=float(verify_data.get("max_abs", ReplayThresholds().max_abs)),
            mean_abs=float(verify_data.get("mean_abs", ReplayThresholds().mean_abs)),
            p95_abs=float(verify_data.get("p95_abs", ReplayThresholds().p95_abs)),
        ),
    )

    return ReplaySequenceManifest(
        path=manifest_path,
        repo_root=repo_root,
        model=model,
        sample=sample,
        runtime=runtime,
    )
