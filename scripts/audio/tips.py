#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MANIFEST = REPO_ROOT / "config/data/tips.yaml"
DEFAULT_CPP_OUT = REPO_ROOT / "src/robot_common/include/robot_common/audio_tips.hpp"
DEFAULT_PY_OUT = REPO_ROOT / "src/robot_audio/robot_audio/audio_tips.py"
DEFAULT_CACHE_DIR = REPO_ROOT / "config/data/temps/tts_cache"
KEY_PATTERN = re.compile(r"^[a-z][a-z0-9_]*$")
# release 包内模型相对 robot_audio share 目录的位置，用于 warmup 默认查找。
INSTALLED_MODEL_RELATIVE = Path("models/matcha-icefall-zh-baker")


def _get_package_share_dir(package_name: str) -> str:
    try:
        from ament_index_python import get_package_share_directory

        return get_package_share_directory(package_name)
    except Exception:
        return ""


def _constant_name(key: str) -> str:
    return key.upper()


def _load_tips(manifest: Path) -> dict[str, str]:
    # tips.yaml 是人工维护入口；这里统一转成字符串键和值，后续生成 C++/Python 常量共用。
    data = yaml.safe_load(manifest.read_text(encoding="utf-8")) or {}
    tips = data.get("tips", {})
    if not isinstance(tips, dict):
        raise ValueError("tips manifest must contain a mapping named 'tips'")
    parsed = {}
    for raw_key, raw_text in tips.items():
        key = str(raw_key)
        text = raw_text.strip() if isinstance(raw_text, str) else ""
        parsed[key] = text
    return parsed


def _validate_tips(tips: dict[str, str]) -> list[str]:
    # 常量名由 key.upper() 生成，因此 key 只允许小写、数字和下划线，避免生成非法代码。
    errors = []
    for key, text in tips.items():
        if not KEY_PATTERN.fullmatch(key):
            errors.append(f"invalid tip key '{key}': use lowercase letters, digits, and underscores")
        if not text:
            errors.append(f"tip '{key}' has empty text")
    return errors


def _load_keys(manifest: Path) -> list[str]:
    # 输出排序固定，避免 tips.yaml 顺序调整导致生成文件出现无意义 diff。
    tips = _load_tips(manifest)
    errors = _validate_tips(tips)
    if errors:
        raise ValueError("\n".join(errors))
    return sorted(tips)


def _write_cpp(keys: list[str], path: Path) -> None:
    lines = [
        "#ifndef ROBOT_COMMON__AUDIO_TIPS_HPP_",
        "#define ROBOT_COMMON__AUDIO_TIPS_HPP_",
        "",
        "namespace robot_common::audio_tips",
        "{",
        "",
    ]
    for key in keys:
        lines.append(f'inline constexpr const char * {_constant_name(key)} = "{key}";')
    lines.extend(
        [
            "",
            "}  // namespace robot_common::audio_tips",
            "",
            "#endif  // ROBOT_COMMON__AUDIO_TIPS_HPP_",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def _write_python(keys: list[str], path: Path) -> None:
    lines = ['"""Generated audio tip constants."""', ""]
    for key in keys:
        lines.append(f'{_constant_name(key)} = "{key}"')
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def _generate(args) -> int:
    keys = _load_keys(Path(args.manifest))
    _write_cpp(keys, Path(args.cpp_out))
    _write_python(keys, Path(args.py_out))
    print(f"generated {len(keys)} tip constants")
    return 0


def _check(args) -> int:
    tips = _load_tips(Path(args.manifest))
    errors = _validate_tips(tips)
    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        return 1
    print(f"tips manifest OK: {len(tips)} tips")
    return 0


def _resolve_model_dir(raw_path: str) -> Path:
    path_text = str(raw_path or "").strip()
    if path_text:
        return Path(path_text).expanduser().resolve()

    # 正常部署时模型随 robot_audio 安装到 share 目录；开发环境可通过 --model-dir 覆盖。
    share_dir = _get_package_share_dir("robot_audio")
    if share_dir:
        model_dir = Path(share_dir) / INSTALLED_MODEL_RELATIVE
        if model_dir.exists():
            return model_dir.resolve()

    raise FileNotFoundError(
        "sherpa-onnx模型目录不存在，请先source install/setup.bash，"
        "或通过--model-dir显式指定模型目录"
    )


def _import_tts_engine():
    try:
        from robot_audio.sherpa_tts_engine import SherpaOnnxTTSEngine

        return SherpaOnnxTTSEngine
    except ImportError:
        # 未 source install/setup.bash 时，允许直接从源码树运行 check/warmup。
        source_package = REPO_ROOT / "src/robot_audio"
        if source_package.exists():
            sys.path.insert(0, str(source_package))
            from robot_audio.sherpa_tts_engine import SherpaOnnxTTSEngine

            return SherpaOnnxTTSEngine
        raise


def _warmup(args) -> int:
    SherpaOnnxTTSEngine = _import_tts_engine()

    # warmup 会真正合成每条提示音，提前填充缓存，减少机器人运行时首次播报延迟。
    tips = _load_tips(Path(args.manifest))
    errors = _validate_tips(tips)
    if errors:
        raise ValueError("\n".join(errors))

    engine = SherpaOnnxTTSEngine(
        model_dir=str(_resolve_model_dir(args.model_dir)),
        num_threads=int(args.num_threads),
        provider=args.provider,
        cache_enabled=True,
        cache_dir=str(Path(args.cache_dir).expanduser().resolve()),
    )
    for key, text in sorted(tips.items()):
        audio_file, cached = engine._synthesize_to_file(text)
        if not audio_file:
            raise RuntimeError(f"failed to warmup tip '{key}'")
        status = "cached" if cached else "generated"
        print(f"{status}: {key} -> {audio_file}")
    print(f"warmed up {len(tips)} tips")
    return 0


def _add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--manifest", default=str(DEFAULT_MANIFEST))


def main() -> int:
    parser = argparse.ArgumentParser(description="Manage robot audio tip text and generated constants.")
    subparsers = parser.add_subparsers(dest="command")

    generate_parser = subparsers.add_parser("generate", help="generate C++ and Python tip constants")
    _add_common_args(generate_parser)
    generate_parser.add_argument("--cpp-out", default=str(DEFAULT_CPP_OUT))
    generate_parser.add_argument("--py-out", default=str(DEFAULT_PY_OUT))
    generate_parser.set_defaults(func=_generate)

    check_parser = subparsers.add_parser("check", help="validate tips.yaml")
    _add_common_args(check_parser)
    check_parser.set_defaults(func=_check)

    warmup_parser = subparsers.add_parser("warmup", help="pre-generate TTS cache for all tips")
    _add_common_args(warmup_parser)
    warmup_parser.add_argument("--model-dir", default="")
    warmup_parser.add_argument("--cache-dir", default=str(DEFAULT_CACHE_DIR))
    warmup_parser.add_argument("--num-threads", type=int, default=2)
    warmup_parser.add_argument("--provider", default="cpu")
    warmup_parser.set_defaults(func=_warmup)

    args = parser.parse_args()
    if not args.command:
        # 兼容旧用法：直接运行脚本时默认刷新生成的常量文件。
        args = parser.parse_args(["generate"])
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
