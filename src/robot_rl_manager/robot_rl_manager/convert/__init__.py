"""RKNN 离线转换工具入口。

这个子包提供一套配置驱动的 ONNX -> RKNN 转换流程，
用于在 x86 开发机上预先生成 RK3588 可部署的 `.rknn` 文件。
"""

from robot_rl_manager.convert.cli import main

__all__ = ["main"]
