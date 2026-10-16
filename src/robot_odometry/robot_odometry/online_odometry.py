"""在线因果本体感觉里程计。

移植自训练仓库的 ``proprio_odometry/odometry/inference.py``（类 ``OnlineOdometry``），
推理语义逐条保持等价：

    第 1 帧：历史窗口用当前帧复制填满
    第 2 帧：滑入第 2 帧，窗口为 [obs0, obs0, ..., obs0, obs1]
    ...
    第 H 帧：窗口为 [obs0, obs1, ..., obs(H-1)]

    previous_odom 初始为 0，每步预测完成后立即写回，
    因此首帧的模型输入是「复制窗口 + 零反馈」。

无论历史是否填满，都使用同一套自回归逻辑，不区分 warmup / full 阶段——
这一点与训练侧的数据增强必须一致，改动这里会直接改变模型输入分布。

自回归反馈使用**归一化空间**的值（图输出的 ``next_previous_odom_norm``），
与参考实现中 ``self.previous_odom = pred_norm`` 完全一致；返回给调用方的
则是米制的位移增量。

本类不依赖 torch / onnxruntime，只依赖一个 ``run(obs_history, prev_norm)``
可调用对象，便于用假后端做纯逻辑单测。
"""

from __future__ import annotations

from collections import deque
from typing import Protocol, Tuple

import numpy as np

from .joint_order import OBS_DIM_NO_ACC, ODOM_DIM, ODOM_HISTORY_LEN


class OdometryBackend(Protocol):
    """里程计推理后端协议。"""

    def run(
        self,
        obs_history: np.ndarray,
        previous_odom_norm: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """输入历史窗口与上一步归一化反馈。

        返回 ``(米制增量 [2], 下一步归一化反馈 [2])``。
        """
        ...


class OnlineOdometry:
    """维护历史窗口与自回归状态，逐帧输出世界系位移增量。"""

    def __init__(
        self,
        backend: OdometryBackend,
        history_len: int = ODOM_HISTORY_LEN,
        obs_dim: int = OBS_DIM_NO_ACC,
        odom_dim: int = ODOM_DIM,
    ) -> None:
        if history_len <= 0:
            raise ValueError(f"history_len 必须为正，实际 {history_len}")

        self._backend = backend
        self._history_len = int(history_len)
        self._obs_dim = int(obs_dim)
        self._odom_dim = int(odom_dim)

        self._history: deque[np.ndarray] = deque(maxlen=self._history_len)
        self._previous_odom_norm = np.zeros(self._odom_dim, dtype=np.float32)
        self._received_frames = 0

        # 复用的窗口缓冲，避免每帧 stack 分配。
        self._window = np.zeros(
            (self._history_len, self._obs_dim), dtype=np.float32
        )

    @property
    def history_len(self) -> int:
        """历史窗口帧数。"""
        return self._history_len

    @property
    def received_frames(self) -> int:
        """自上次 reset 以来送入的帧数。"""
        return self._received_frames

    @property
    def previous_odom_norm(self) -> np.ndarray:
        """上一步的归一化反馈值，供诊断使用。"""
        return self._previous_odom_norm.copy()

    def reset(self) -> None:
        """清空历史与自回归状态，用于 episode / 控制流重启。"""
        self._history.clear()
        self._previous_odom_norm = np.zeros(self._odom_dim, dtype=np.float32)
        self._received_frames = 0

    def step(self, observation: np.ndarray) -> np.ndarray:
        """送入一帧 88 维观测，返回本步世界系位移增量 ``[dx, dy]``（米）。

        调用方负责积分：``p(t+1) = p(t) + step(...)``。
        """
        obs = np.asarray(observation, dtype=np.float32).reshape(-1)
        if obs.shape != (self._obs_dim,):
            raise ValueError(
                f"观测维度应为 [{self._obs_dim}]，实际 {obs.shape}"
            )

        self._received_frames += 1

        if not self._history:
            # 首帧：复制填满整个窗口，与训练侧 warmup 增强一致。
            for _ in range(self._history_len):
                self._history.append(obs.copy())
        else:
            self._history.append(obs.copy())

        np.stack(self._history, out=self._window)

        delta, next_norm = self._backend.run(
            self._window, self._previous_odom_norm
        )

        delta = np.asarray(delta, dtype=np.float32).reshape(-1)
        next_norm = np.asarray(next_norm, dtype=np.float32).reshape(-1)

        if delta.shape != (self._odom_dim,):
            raise ValueError(
                f"里程计输出维度应为 [{self._odom_dim}]，实际 {delta.shape}"
            )
        if next_norm.shape != (self._odom_dim,):
            raise ValueError(
                f"自回归反馈维度应为 [{self._odom_dim}]，实际 {next_norm.shape}"
            )

        # 自回归：无论 warmup 还是稳态，都立即把本步反馈写回。
        self._previous_odom_norm = next_norm.copy()

        return delta
