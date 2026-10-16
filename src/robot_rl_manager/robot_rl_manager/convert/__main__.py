"""`python -m robot_rl_manager.convert` 的模块入口。

直接执行这个模块时，会调用 CLI 主入口去完成模型转换。
"""

from robot_rl_manager.convert.cli import main


if __name__ == "__main__":
    # 允许直接用 `python -m robot_rl_manager.convert` 运行整个转换工具。
    raise SystemExit(main())
