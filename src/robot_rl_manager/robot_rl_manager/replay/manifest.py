"""Load and resolve replay manifest entries."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping

import yaml

DEFAULT_ATOL = 1.0e-4
DEFAULT_RTOL = 1.0e-4
DEFAULT_MAX_ABS = 5.0e-2
DEFAULT_MEAN_ABS = 1.0e-2
DEFAULT_P95_ABS = 3.0e-2
DEFAULT_MODEL_DIR = "model"
DEFAULT_SAMPLE_DIR = "config/data/samples"


@dataclass(frozen=True)
class ReplayThresholds:
    """Numeric thresholds used when comparing replay outputs."""

    atol: float = DEFAULT_ATOL
    rtol: float = DEFAULT_RTOL
    max_abs: float = DEFAULT_MAX_ABS
    mean_abs: float = DEFAULT_MEAN_ABS
    p95_abs: float = DEFAULT_P95_ABS


@dataclass(frozen=True)
class ReplayDefaults:
    """Top-level manifest defaults."""

    target_platform: str = "rk3588"
    baseline_backend: str = "onnxruntime"
    rknn_backend: str = "rknnlite"
    verify: ReplayThresholds = ReplayThresholds()


@dataclass(frozen=True)
class ReplayModelSpec:
    """Normalized manifest entry for a single replay model."""

    name: str
    sample: str | None = None
    onnx: str | None = None
    rknn: str | None = None
    verify: ReplayThresholds | None = None
    need_capture: bool = False


@dataclass(frozen=True)
class ReplayManifest:
    """Loaded replay manifest plus its repository root."""

    path: Path
    repo_root: Path
    defaults: ReplayDefaults
    models: Mapping[str, ReplayModelSpec]


@dataclass(frozen=True)
class ReplayJob:
    """Resolved replay job with absolute file paths."""

    model_name: str
    sample_path: Path
    onnx_path: Path
    rknn_path: Path
    target_platform: str
    baseline_backend: str
    rknn_backend: str
    verify: ReplayThresholds
    need_capture: bool = False


def _resolve_repo_root(manifest_path: Path) -> Path:
    """Resolve the replay package root from the manifest location.

    The replay manifest lives inside ``robot_rl_manager/replay`` and the model/config
    directories sit in the repository root. We therefore walk upward until we find a
    directory that contains both ``config/`` and ``src/robot_rl_manager``.
    """
    manifest_path = manifest_path.resolve()
    for candidate in manifest_path.parents:
        config_dir = candidate / "config"
        source_dir = candidate / "src" / "robot_rl_manager"
        if config_dir.is_dir() and source_dir.is_dir():
            return candidate
    raise FileNotFoundError(f"could not locate robot_rl_manager repo root for {manifest_path}")


def _resolve_package_root(repo_root: Path) -> Path:
    """返回 robot_rl_manager 包的源码根目录。

    replay 的模型文件仍放在 ``src/robot_rl_manager`` 下，而样本和报告文件
    放在仓库根目录下的 ``config/data/samples``。因此需要同时保留两个基准目录。
    """
    if repo_root.name == "robot_rl_manager" and (repo_root / "model").is_dir():
        return repo_root.resolve()
    package_root = (repo_root / "src" / "robot_rl_manager").resolve()
    if not package_root.is_dir():
        raise FileNotFoundError(f"could not locate robot_rl_manager package root under {repo_root}")
    return package_root


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"replay manifest must be a mapping: {path}")
    return data


def _require_mapping(
    value: Any,
    field_name: str,
    manifest_path: Path,
    *,
    required: bool = False,
) -> Mapping[str, Any]:
    """Validate that a manifest value is mapping-like before normalization."""
    if value is None:
        if required:
            raise ValueError(f"replay manifest field '{field_name}' is required: {manifest_path}")
        return {}
    if not isinstance(value, dict):
        raise ValueError(
            f"replay manifest field '{field_name}' must be a mapping: {manifest_path}"
        )
    return value


def _coerce_thresholds(data: Mapping[str, Any] | None, fallback: ReplayThresholds) -> ReplayThresholds:
    data = data or {}
    return ReplayThresholds(
        atol=float(data.get("atol", fallback.atol)),
        rtol=float(data.get("rtol", fallback.rtol)),
        max_abs=float(data.get("max_abs", fallback.max_abs)),
        mean_abs=float(data.get("mean_abs", fallback.mean_abs)),
        p95_abs=float(data.get("p95_abs", fallback.p95_abs)),
    )


def _normalize_model_spec(name: str, data: Mapping[str, Any], defaults: ReplayDefaults) -> ReplayModelSpec:
    verify = _coerce_thresholds(data.get("verify"), defaults.verify)
    return ReplayModelSpec(
        name=name,
        sample=str(data["sample"]) if data.get("sample") else None,
        onnx=str(data["onnx"]) if data.get("onnx") else None,
        rknn=str(data["rknn"]) if data.get("rknn") else None,
        verify=verify,
        need_capture=bool(data.get("need_capture", False)),
    )


def load_replay_manifest(path: Path) -> ReplayManifest:
    """Load and normalize a replay manifest YAML file."""
    manifest_path = path.expanduser().resolve()
    raw = _load_yaml(manifest_path)
    defaults_data = _require_mapping(raw.get("defaults"), "defaults", manifest_path)
    verify_data = _require_mapping(defaults_data.get("verify"), "defaults.verify", manifest_path)
    defaults = ReplayDefaults(
        target_platform=str(defaults_data.get("target_platform", "rk3588")),
        baseline_backend=str(defaults_data.get("baseline_backend", "onnxruntime")),
        rknn_backend=str(defaults_data.get("rknn_backend", "rknnlite")),
        verify=ReplayThresholds(
            atol=float(verify_data.get("atol", DEFAULT_ATOL)),
            rtol=float(verify_data.get("rtol", DEFAULT_RTOL)),
            max_abs=float(verify_data.get("max_abs", DEFAULT_MAX_ABS)),
            mean_abs=float(verify_data.get("mean_abs", DEFAULT_MEAN_ABS)),
            p95_abs=float(verify_data.get("p95_abs", DEFAULT_P95_ABS)),
        ),
    )

    models_data = _require_mapping(raw.get("models"), "models", manifest_path, required=True)

    models = {
        name: _normalize_model_spec(name, _require_mapping(spec, f"models.{name}", manifest_path), defaults)
        for name, spec in models_data.items()
    }
    return ReplayManifest(
        path=manifest_path,
        repo_root=_resolve_repo_root(manifest_path),
        defaults=defaults,
        models=MappingProxyType(models),
    )


def _resolve_repo_path(base_dir: Path, relative_path: str) -> Path:
    path = Path(relative_path)
    if path.is_absolute():
        return path.resolve()
    return (base_dir / path).resolve()


def resolve_replay_job(manifest: ReplayManifest, model_name: str) -> ReplayJob:
    """Resolve a model entry into concrete file paths and thresholds."""
    if model_name not in manifest.models:
        raise KeyError(f"unknown replay model: {model_name}")

    spec = manifest.models[model_name]
    repo_root = manifest.repo_root
    package_root = _resolve_package_root(repo_root)
    sample_rel = spec.sample or f"{DEFAULT_SAMPLE_DIR}/{model_name}/inputs_seq.npz"
    onnx_rel = spec.onnx or f"{DEFAULT_MODEL_DIR}/{model_name}.onnx"
    rknn_rel = spec.rknn or f"{DEFAULT_MODEL_DIR}/{model_name}.rknn"

    return ReplayJob(
        model_name=model_name,
        sample_path=_resolve_repo_path(repo_root, sample_rel),
        onnx_path=_resolve_repo_path(package_root, onnx_rel),
        rknn_path=_resolve_repo_path(package_root, rknn_rel),
        target_platform=manifest.defaults.target_platform,
        baseline_backend=manifest.defaults.baseline_backend,
        rknn_backend=manifest.defaults.rknn_backend,
        verify=spec.verify or manifest.defaults.verify,
        need_capture=spec.need_capture,
    )
