"""Unified ONNX -> RKNN conversion pipeline."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

try:
    import numpy as np
    import onnx

    if hasattr(onnx, "_mapping"):
        # RKNN Toolkit2 2.3.2 仍然依赖旧版 onnx.mapping 接口，这里做运行时兼容。
        if not hasattr(onnx, "mapping"):
            onnx.mapping = onnx._mapping  # type: ignore[attr-defined]
        mapping = onnx.mapping  # type: ignore[attr-defined]
        if hasattr(mapping, "TENSOR_TYPE_MAP"):
            if not hasattr(mapping, "TENSOR_TYPE_TO_NP_TYPE"):
                mapping.TENSOR_TYPE_TO_NP_TYPE = {
                    k: np.dtype(v.np_dtype) for k, v in mapping.TENSOR_TYPE_MAP.items()
                }
            if not hasattr(mapping, "NP_TYPE_TO_TENSOR_TYPE"):
                reverse: dict[object, int] = {}
                for tensor_type, meta in mapping.TENSOR_TYPE_MAP.items():
                    np_dtype = np.dtype(meta.np_dtype)
                    reverse[np_dtype] = tensor_type
                    reverse[np_dtype.type] = tensor_type
                    reverse[meta.np_dtype] = tensor_type
                    reverse[meta.np_dtype.type] = tensor_type
                mapping.NP_TYPE_TO_TENSOR_TYPE = reverse
except Exception:
    pass

from robot_rl_manager.convert.patches import apply_patches
from robot_rl_manager.convert.utils import find_repo_root, flatten_shape, load_yaml, normalize_prefix_list
from robot_rl_manager.convert.verify import verify_rknn_against_onnx


@dataclass(frozen=True)
class ModelConvertSpec:
    """单个模型的转换配置。

    这些字段都来自 `manifest.yaml`，用于控制：
    - 是否需要 patch
    - RKNN target platform
    - 是否启用验证
    - 输入 shape / mean / std 的显式覆盖
    """

    patches: tuple[str, ...] = ()
    target_platform: str = "rk3588"
    verify_enabled: bool = True
    verify_sample_count: int = 3
    verify_atol: float = 1.0e-4
    verify_rtol: float = 1.0e-4
    input_size_list: tuple[tuple[int, ...], ...] = ()
    mean_values: tuple[tuple[float, ...], ...] = ()
    std_values: tuple[tuple[float, ...], ...] = ()


@dataclass(frozen=True)
class ConvertManifest:
    """整个转换系统的总配置。

    allow/exclude 用于筛选哪些模型允许进入 RKNN 转换流程；
    defaults 是所有模型共享的默认配置；
    models 用于覆盖单个模型的特殊配置。
    """

    allow_model_prefixes: tuple[str, ...] = ("t4_",)
    exclude_model_prefixes: tuple[str, ...] = ("t4_kd_",)
    defaults: ModelConvertSpec = ModelConvertSpec()
    models: dict[str, ModelConvertSpec] = field(default_factory=dict)


@dataclass(frozen=True)
class RegistryModel:
    """从运行时注册表解析出来的模型描述。

    这里保留了 runtime 侧最关心的三个路径：
    - model_path: 模型文件本体
    - config_path: 模型配置
    - adapter_path: 语义适配器
    """

    name: str
    model_type: str
    model_path: Path
    config_path: Path
    adapter_path: Path


def load_convert_manifest(path: Path) -> ConvertManifest:
    """读取转换 manifest。

    Args:
        path: `convert/manifest.yaml` 的路径。

    Returns:
        归一化后的 `ConvertManifest` 对象。

    Notes:
        这里会把 YAML 里的 list/标量统一整理成 tuple / dataclass，便于后续代码使用。
    """
    data = load_yaml(path)
    defaults_data = data.get("defaults", {}) or {}
    defaults = ModelConvertSpec(
        patches=tuple(str(item) for item in defaults_data.get("patches", []) or []),
        target_platform=str(defaults_data.get("target_platform", "rk3588")),
        verify_enabled=bool((defaults_data.get("verify", {}) or {}).get("enabled", True)),
        verify_sample_count=int((defaults_data.get("verify", {}) or {}).get("sample_count", 3)),
        verify_atol=float((defaults_data.get("verify", {}) or {}).get("atol", 1.0e-4)),
        verify_rtol=float((defaults_data.get("verify", {}) or {}).get("rtol", 1.0e-4)),
        input_size_list=tuple(tuple(int(v) for v in shape) for shape in defaults_data.get("input_size_list", []) or []),
        mean_values=tuple(tuple(float(v) for v in values) for values in defaults_data.get("mean_values", []) or []),
        std_values=tuple(tuple(float(v) for v in values) for values in defaults_data.get("std_values", []) or []),
    )

    allow = normalize_prefix_list(defaults_data.get("allow_model_prefixes"))
    exclude = normalize_prefix_list(defaults_data.get("exclude_model_prefixes"))
    if not allow:
        allow = ("t4_",)
    if not exclude:
        exclude = ("t4_kd_",)

    models_data = data.get("models", {}) or {}
    if not isinstance(models_data, dict):
        raise ValueError("manifest models must be a map")

    models: dict[str, ModelConvertSpec] = {}
    for model_name, item in models_data.items():
        if not isinstance(item, dict):
            raise ValueError(f"manifest entry for {model_name} must be a map")
        verify_data = item.get("verify", {}) or {}
        models[str(model_name)] = ModelConvertSpec(
            patches=tuple(str(name) for name in item.get("patches", []) or []),
            target_platform=str(item.get("target_platform", defaults.target_platform)),
            verify_enabled=bool(verify_data.get("enabled", defaults.verify_enabled)),
            verify_sample_count=int(verify_data.get("sample_count", defaults.verify_sample_count)),
            verify_atol=float(verify_data.get("atol", defaults.verify_atol)),
            verify_rtol=float(verify_data.get("rtol", defaults.verify_rtol)),
            input_size_list=tuple(
                tuple(int(v) for v in shape)
                for shape in item.get("input_size_list", []) or []
            ),
            mean_values=tuple(
                tuple(float(v) for v in values)
                for values in item.get("mean_values", []) or []
            ),
            std_values=tuple(
                tuple(float(v) for v in values)
                for values in item.get("std_values", []) or []
            ),
        )

    return ConvertManifest(
        allow_model_prefixes=allow,
        exclude_model_prefixes=exclude,
        defaults=defaults,
        models=models,
    )


def load_registry_models(registry_path: Path) -> dict[str, RegistryModel]:
    """读取运行时模型注册表。

    Args:
        registry_path: `config/humanoid/rl_models.yaml` 路径。

    Returns:
        模型名到 `RegistryModel` 的映射。

    Notes:
        registry 中路径通常是相对 `src/robot_rl_manager` 目录写的，所以这里要显式做一次根目录拼接。
    """
    data = load_yaml(registry_path)
    models = data.get("models", {}) or {}
    if not isinstance(models, dict):
        raise ValueError("registry models must be a map")

    repo_root = find_repo_root(registry_path)
    # registry 里的相对路径是按 src/robot_rl_manager 为根目录写的，不是仓库根目录。
    root_dir = repo_root / "src" / "robot_rl_manager"
    resolved: dict[str, RegistryModel] = {}
    for model_name, item in models.items():
        if not isinstance(item, dict):
            continue
        artifact = _select_onnx_artifact(item)
        if artifact is None:
            continue
        model_path_text = str(artifact.get("model_path", ""))
        config_path_text = str(item.get("config_path", ""))
        adapter_path_text = str(item.get("adapter_path", ""))
        model_type = str(artifact.get("backend", ""))
        if not model_path_text or not config_path_text or not adapter_path_text:
            continue
        resolved[str(model_name)] = RegistryModel(
            name=str(model_name),
            model_type=model_type,
            model_path=(root_dir / model_path_text).resolve(),
            config_path=(root_dir / config_path_text).resolve(),
            adapter_path=(root_dir / adapter_path_text).resolve(),
        )
    return resolved


def _select_onnx_artifact(index_item: dict) -> dict | None:
    """从平台 artifacts 中选择可作为 RKNN 转换源的 ONNX artifact。"""
    artifacts = index_item.get("artifacts")
    if not isinstance(artifacts, dict):
        return None
    for artifact in artifacts.values():
        if not isinstance(artifact, dict):
            continue
        if str(artifact.get("backend", "")).lower() == "onnx" and str(artifact.get("model_path", "")):
            return artifact
    return None


def _infer_input_shapes(onnx_path: Path) -> list[list[int]]:
    """从 ONNX 文件里推断输入 shape。

    Args:
        onnx_path: ONNX 模型路径。

    Returns:
        每个输入张量的 shape 列表。未知维度会被补成 1。
    """
    import onnx

    model = onnx.load(str(onnx_path))
    shapes: list[list[int]] = []
    for input_info in model.graph.input:
        dims: list[int] = []
        for dim in input_info.type.tensor_type.shape.dim:
            value = int(dim.dim_value) if dim.dim_value > 0 else 1
            dims.append(value)
        if not dims:
            raise ValueError(f"unable to infer input shape for {input_info.name}")
        shapes.append(dims)
    return shapes


def _select_spec(manifest: ConvertManifest, model_name: str) -> ModelConvertSpec:
    """按模型名选择转换配置。

    Args:
        manifest: 总 manifest。
        model_name: 当前要转换的模型名。

    Returns:
        模型专属配置；如果没有专属条目，则返回 defaults。
    """
    return manifest.models.get(model_name, manifest.defaults)


def _check_prefix_rules(manifest: ConvertManifest, model_name: str) -> None:
    """校验模型是否允许进入转换流程。

    Args:
        manifest: 总 manifest。
        model_name: 当前模型名。

    Raises:
        ValueError: 模型不在白名单，或者命中黑名单时抛出。
    """
    if not any(model_name.startswith(prefix) for prefix in manifest.allow_model_prefixes):
        raise ValueError(f"model {model_name} is not enabled by allow_model_prefixes")
    if any(model_name.startswith(prefix) for prefix in manifest.exclude_model_prefixes):
        raise ValueError(f"model {model_name} is excluded by exclude_model_prefixes")


def _as_nested_float_lists(values: tuple[tuple[float, ...], ...], shapes: list[list[int]], fill: float) -> list[list[float]]:
    """把 mean/std 配置整理成 RKNN 需要的二维列表。

    Args:
        values: manifest 中显式给出的 mean/std。
        shapes: 输入 shape 列表。
        fill: 未显式配置时的默认填充值。

    Returns:
        适合直接传给 `rknn.config()` 的二维 float list。
    """
    if values:
        return [list(item) for item in values]
    # manifest 没有显式给 mean/std 时，按输入 shape 自动补成全 0 / 全 1。
    return [[float(fill)] * flatten_shape(shape) for shape in shapes]


def _cleanup_rknn_check_artifacts(base_dir: Path) -> None:
    """清理 RKNN Toolkit2 生成的中间检查文件。

    Args:
        base_dir: 需要扫描的目录，通常是当前工作目录。

    Notes:
        RKNN 在 build 过程中会在当前目录生成若干 `check*_*.onnx` 之类的检查图。
        这些文件只用于调试和中间过程观察，不应该作为最终产物保留。
    """
    for pattern in ("check*.onnx", "check*.json", "check*.txt", "check*.rknn"):
        for path in base_dir.glob(pattern):
            try:
                path.unlink()
            except Exception:
                pass


def convert_model(
    model_name: str,
    manifest: ConvertManifest,
    registry_models: dict[str, RegistryModel],
    repo_root: Path,
    keep_patched_onnx: bool = False,
    force_verify: bool = False,
    skip_verify: bool = False,
    output_override: Path | None = None,
) -> Path:
    """执行单个模型的 ONNX -> RKNN 转换。

    Args:
        model_name: 要转换的模型名。
        manifest: 转换总配置。
        registry_models: 运行时注册表解析结果。
        repo_root: 仓库根目录。
        keep_patched_onnx: 是否保留临时补丁 ONNX 文件。
        force_verify: 强制开启验证，即使 manifest 里关闭了验证。
        skip_verify: 跳过数值验证，只保留导出流程。
        output_override: 手动指定 RKNN 输出路径。

    Returns:
        生成的 `.rknn` 文件路径。

    Raises:
        FileNotFoundError / ValueError / RuntimeError:
            分别对应模型文件不存在、配置不合法、RKNN 转换流程失败。

    Notes:
        这里的主流程是：
        1. 读注册表和 manifest
        2. 选出模型专属配置
        3. 必要时先 patch ONNX
        4. 调 `rknn.load_onnx()` / `build()` / `export_rknn()`
        5. 可选做 ONNX Runtime 数值验证

        验证失败默认只记录 warning，不阻断导出流程。这样可以把验证结果
        当成“质量检查”而不是“导出门禁”，避免 float16 数值差异导致整次
        批量转换中断。
    """
    _check_prefix_rules(manifest, model_name)
    if model_name not in registry_models:
        raise KeyError(f"unknown model in registry: {model_name}")

    registry_model = registry_models[model_name]
    if registry_model.model_type.lower() != "onnx":
        raise ValueError(f"model {model_name} is not an ONNX model: {registry_model.model_type}")

    spec = _select_spec(manifest, model_name)
    source_onnx = registry_model.model_path
    if not source_onnx.exists():
        raise FileNotFoundError(f"source ONNX not found: {source_onnx}")

    # 默认输出和原 ONNX 同名，方便人工排查和替换。
    target_rknn = output_override or source_onnx.with_suffix(".rknn")
    input_shapes = list(spec.input_size_list) if spec.input_size_list else _infer_input_shapes(source_onnx)
    mean_values = _as_nested_float_lists(spec.mean_values, input_shapes, 0.0)
    std_values = _as_nested_float_lists(spec.std_values, input_shapes, 1.0)

    patched_onnx = source_onnx
    temp_paths: list[Path] = []
    if spec.patches:
        # 如果 manifest 声明了 patch，就先生成临时修补版 ONNX。
        patched_onnx, temp_paths = apply_patches(source_onnx, spec.patches)

    rknn = None
    try:
        from rknn.api import RKNN

        rknn = RKNN(verbose=True)
        # 这些日志是给手工批量转换时看的，便于快速确认路径和配置是否正确。
        print(f"[convert] model={model_name}")
        print(f"[convert] source={source_onnx}")
        print(f"[convert] target={target_rknn}")
        print(f"[convert] input_shapes={input_shapes}")
        if mean_values:
            print(f"[convert] mean_values lengths={[len(v) for v in mean_values]}")
        if std_values:
            print(f"[convert] std_values lengths={[len(v) for v in std_values]}")
        if spec.patches:
            print(f"[convert] patches={list(spec.patches)}")

        rknn.config(
            mean_values=mean_values,
            std_values=std_values,
            target_platform=spec.target_platform,
        )
        ret = rknn.load_onnx(model=str(patched_onnx), input_size_list=[list(shape) for shape in input_shapes])
        if ret != 0:
            raise RuntimeError(f"RKNN load_onnx failed with code {ret}")
        ret = rknn.build(do_quantization=False)
        if ret != 0:
            raise RuntimeError(f"RKNN build failed with code {ret}")
        ret = rknn.export_rknn(str(target_rknn))
        if ret != 0:
            raise RuntimeError(f"RKNN export_rknn failed with code {ret}")
        ret = rknn.init_runtime()
        if ret != 0:
            raise RuntimeError(f"RKNN init_runtime failed with code {ret}")

        # 导出成功不代表数值验证一定能过；RKNN 常会因为 float16 误差出现小偏差。
        should_verify = (spec.verify_enabled or force_verify) and not skip_verify
        if should_verify:
            try:
                result = verify_rknn_against_onnx(
                    rknn,
                    patched_onnx,
                    [list(shape) for shape in input_shapes],
                    sample_count=spec.verify_sample_count,
                    atol=spec.verify_atol,
                    rtol=spec.verify_rtol,
                )
            except Exception as exc:
                # 默认把验证失败降级成 warning，保证导出产物仍然可用。
                print(f"[verify] warning: verification failed but export is kept: {exc}")
            else:
                print(
                    "[verify] "
                    f"samples={result.sample_count} "
                    f"max_diff={result.max_diff:.6e} "
                    f"mean_diff={result.mean_diff:.6e}"
                )

        return target_rknn
    finally:
        if rknn is not None:
            try:
                rknn.release()
            except Exception:
                pass
        # 这里顺手清掉 RKNN Toolkit2 在当前目录里生成的调试检查文件。
        _cleanup_rknn_check_artifacts(Path.cwd())
        if not keep_patched_onnx:
            for temp_path in temp_paths:
                try:
                    temp_path.unlink(missing_ok=True)
                except Exception:
                    pass
