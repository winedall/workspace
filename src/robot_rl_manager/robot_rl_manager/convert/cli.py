"""Command line entry for RKNN conversion."""

from __future__ import annotations

import argparse
from pathlib import Path

from robot_rl_manager.convert.converter import (
    convert_model,
    load_convert_manifest,
    load_registry_models,
)
from robot_rl_manager.convert.utils import resolve_repo_paths


def build_parser() -> argparse.ArgumentParser:
    """构造命令行参数解析器。

    Returns:
        argparse 解析器实例。

    Notes:
        这里保留成单独函数，方便后续测试时直接复用参数定义。
    """
    parser = argparse.ArgumentParser(description="Convert ONNX models to RKNN.")
    parser.add_argument("--model", help="要转换的模型名。")
    parser.add_argument(
        "--manifest",
        default="",
        help="转换 manifest 路径，默认使用包内的 manifest.yaml。",
    )
    parser.add_argument(
        "--registry",
        default="",
        help="运行时模型注册表路径，默认使用 config/humanoid/rl_models.yaml。",
    )
    parser.add_argument("--list", action="store_true", help="只列出可转换模型并退出。")
    parser.add_argument("--verify", action="store_true", help="即使 manifest 关闭验证，也强制执行数值验证。")
    # 某些模型在 RKNN float16 下能成功导出，但与 ONNX 的数值误差会超出默认阈值。
    parser.add_argument("--no-verify", action="store_true", help="跳过 RKNN/ONNX 数值验证步骤。")
    parser.add_argument("--keep-patched-onnx", action="store_true", help="保留临时生成的补丁 ONNX 文件。")
    parser.add_argument("--output", default="", help="手动指定 RKNN 输出路径。")
    return parser


def main(argv: list[str] | None = None) -> int:
    """CLI 主入口。

    Args:
        argv: 可选的参数列表，传 None 时使用 sys.argv。

    Returns:
        进程退出码，0 表示成功。

    Notes:
        逻辑顺序是：解析参数 -> 找 manifest / registry -> 过滤可转模型 ->
        执行转换。这样 CLI 层只负责调度，不承载具体转换细节。
    """
    parser = build_parser()
    args = parser.parse_args(argv)

    manifest_path = Path(args.manifest).resolve() if args.manifest else None
    registry_path = Path(args.registry).resolve() if args.registry else None
    resolved = resolve_repo_paths(manifest_path=manifest_path, registry_path=registry_path)
    manifest = load_convert_manifest(manifest_path or resolved.manifest_path)
    registry_path = registry_path or resolved.registry_path
    registry_models = load_registry_models(registry_path)

    # 这里只开放 manifest 白名单内的 ONNX 模型，避免误把 KD 或 PT 模型拉进来。
    eligible = sorted(
        name
        for name in registry_models
        if any(name.startswith(prefix) for prefix in manifest.allow_model_prefixes)
        and not any(name.startswith(prefix) for prefix in manifest.exclude_model_prefixes)
        and registry_models[name].model_type.lower() == "onnx"
    )

    if args.list:
        for name in eligible:
            print(name)
        return 0

    if not args.model:
        parser.error("--model is required unless --list is used")

    if args.model not in eligible:
        raise ValueError(
            f"model {args.model} is not eligible for RKNN conversion; "
            f"eligible models: {', '.join(eligible)}"
        )

    output_override = Path(args.output).resolve() if args.output else None
    # 真正的转换工作交给 converter.py，这里只把参数整理好后调用。
    convert_model(
        args.model,
        manifest,
        registry_models,
        repo_root=resolved.repo_root,
        keep_patched_onnx=bool(args.keep_patched_onnx),
        force_verify=bool(args.verify),
        skip_verify=bool(args.no_verify),
        output_override=output_override,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
