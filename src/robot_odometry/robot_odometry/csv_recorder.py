"""里程计与仿真真值的逐帧对比记录器。

列顺序按要求排列：位置真值 → 位置变化量真值 → 位置估计值 → 位置变化估计值，
末尾附加仿真世界系下的绝对真值位置，便于与其它工具对齐。

真值的「相对」列以本次运行起点为原点，与估计值共用同一个原点，因此
``gt_x - est_x`` 可以直接当作位置误差使用。
"""

from __future__ import annotations

import csv
import os
from pathlib import Path
from typing import TextIO

COLUMNS = (
    "t",            # 自节点启动的时刻（秒，单调递增，运行间隔体现为跳变）
    "step",         # 本次运行内的推理帧序号
    "gt_x",         # 位置真值（相对起点，米）
    "gt_y",
    "gt_dx",        # 位置变化量真值（米）
    "gt_dy",
    "est_x",        # 位置估计值（米）
    "est_y",
    "est_dx",       # 位置变化估计值（米）
    "est_dy",
    "gt_abs_x",     # 仿真世界系绝对真值（米，含初始高度）
    "gt_abs_y",
    "gt_abs_z",
)

# 未显式配置输出目录时的默认根目录，与 run.sh 的 recorder 约定保持一致。
DEFAULT_DIR_ENV = "ZL_ROBOT_RECORDER_DIR"
DEFAULT_SUBDIR = "odometry"


def resolve_output_dir(configured_dir: str) -> Path:
    """解析 CSV 输出目录。

    优先级：显式配置的 ``csv_dir`` → run.sh 导出的 ``ZL_ROBOT_RECORDER_DIR``
    （使记录与其他 recorder 产物落在同一个 run 目录下）→ 当前工作目录下的
    ``logs/odometry``（与仿真节点写 ``logs/mj_sim`` 的惯例一致）。
    """
    if configured_dir:
        return Path(configured_dir)
    env_dir = os.environ.get(DEFAULT_DIR_ENV, "")
    if env_dir:
        return Path(env_dir) / DEFAULT_SUBDIR
    return Path.cwd() / "logs" / DEFAULT_SUBDIR


class CsvRecorder:
    """逐帧写入里程计与真值对比表。"""

    def __init__(self, path: Path) -> None:
        self._path = Path(path)
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._file: TextIO = self._path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(COLUMNS)
        self._rows = 0

    @property
    def path(self) -> Path:
        """返回记录文件路径。"""
        return self._path

    @property
    def rows(self) -> int:
        """返回已写入的行数。"""
        return self._rows

    def write_row(
        self,
        time_sec: float,
        step: int,
        gt_xy: tuple[float, float],
        gt_delta_xy: tuple[float, float],
        est_xy: tuple[float, float],
        est_delta_xy: tuple[float, float],
        gt_abs_xyz: tuple[float, float, float],
    ) -> None:
        """写入一帧对比记录。"""
        self._writer.writerow(
            (
                f"{time_sec:.6f}",
                step,
                f"{gt_xy[0]:.6f}",
                f"{gt_xy[1]:.6f}",
                f"{gt_delta_xy[0]:.6f}",
                f"{gt_delta_xy[1]:.6f}",
                f"{est_xy[0]:.6f}",
                f"{est_xy[1]:.6f}",
                f"{est_delta_xy[0]:.6f}",
                f"{est_delta_xy[1]:.6f}",
                f"{gt_abs_xyz[0]:.6f}",
                f"{gt_abs_xyz[1]:.6f}",
                f"{gt_abs_xyz[2]:.6f}",
            )
        )
        # 50Hz 写入量很小，逐行落盘换取崩溃时不丢数据。
        self._file.flush()
        self._rows += 1

    def close(self) -> None:
        """关闭文件。"""
        if not self._file.closed:
            self._file.close()
