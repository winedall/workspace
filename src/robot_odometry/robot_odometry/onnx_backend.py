"""里程计 ONNX 模型后端。

模型接口契约（与训练侧导出对齐）：

    输入 obs_history          float32 [batch, 50, ODIM]
        未归一化的本体感觉历史窗口，按时间从旧到新排列。
        ODIM 由模型决定：88（不含基座加速度）或 91（含）。

    输入 previous_odom_norm   float32 [batch, 2]
        **归一化空间**的上一步位移增量预测；首帧为全 0。
        该值应当直接使用上一步的 next_previous_odom_norm，不要自行换算。

    输出 pred_delta           float32 [batch, 2]
        当前步的世界系位移增量预测，单位**米**（反归一化已完成）。

    输出 next_previous_odom_norm float32 [batch, 2]
        归一化空间的当前步预测，下一步直接回填给 previous_odom_norm。

观测归一化（obs_mean/obs_std）与目标反归一化（target_mean/target_std）都已
折进图的常量里，运行时不需要 normalization.npz。

自回归语义与训练仓库 ``odometry/inference.py`` 的 ``OnlineOdometry`` 等价：
参考实现维护 ``previous_odom``（归一化空间的模型原始输出），本图把该值的
产生（归一化）与消费（反归一化）都放在图内，调用方只处理米制量与反馈量。
"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

import numpy as np

from .joint_order import OBS_DIM_NO_ACC, OBS_DIM_WITH_ACC, ODOM_DIM, ODOM_HISTORY_LEN

# 图输入输出名称。导出侧按此命名，加载时按名绑定。
INPUT_OBS_HISTORY = "obs_history"
INPUT_PREV_ODOM_NORM = "previous_odom_norm"
OUTPUT_PRED_DELTA = "pred_delta"
OUTPUT_NEXT_ODOM_NORM = "next_previous_odom_norm"

class OnnxContractError(RuntimeError):
    """ONNX 模型的输入输出不符合里程计接口契约。"""


def _check_shapes(
    kind: str,
    actual: dict[str, Sequence[object]],
    expected: dict[str, list[int]],
) -> None:
    """逐项比对名称与形状，缺失或形状不符时给出可操作的报错。

    ``actual`` 为 ``{名称: 形状}``；形状里的非整数维（导出时的符号维，例如
    动态 batch 的 ``'batch'``）不参与数值比对，但维度数必须一致。
    """
    for name, shape in expected.items():
        if name not in actual:
            raise OnnxContractError(
                f"ONNX 模型{kind}缺少 '{name}'。"
                f"实际{kind}: {sorted(actual)}；"
                f"期望{kind}: {sorted(expected)}。"
                "接口契约见 robot_odometry/onnx_backend.py 模块文档，"
                "或 model/README.md。"
            )
        actual_shape = list(actual[name])
        if len(actual_shape) != len(shape) or any(
            isinstance(value, int) and value != expected_dim
            for value, expected_dim in zip(actual_shape, shape)
        ):
            raise OnnxContractError(
                f"ONNX 模型{kind} '{name}' 形状为 {actual_shape}，期望 {shape}。"
            )


def validate_io_contract(
    inputs: dict[str, Sequence[object]],
    outputs: dict[str, Sequence[object]],
    obs_dim: int,
    history_len: int = ODOM_HISTORY_LEN,
) -> None:
    """校验 ONNX 图的输入输出名称与形状是否满足里程计接口契约。"""
    _check_shapes(
        "输入",
        inputs,
        {
            INPUT_OBS_HISTORY: [1, history_len, obs_dim],
            INPUT_PREV_ODOM_NORM: [1, ODOM_DIM],
        },
    )
    _check_shapes(
        "输出",
        outputs,
        {
            OUTPUT_PRED_DELTA: [1, ODOM_DIM],
            OUTPUT_NEXT_ODOM_NORM: [1, ODOM_DIM],
        },
    )


class OnnxOdometryBackend:
    """加载并执行里程计 ONNX 模型。"""

    def __init__(
        self,
        model_path: str | Path,
        obs_dim: int = OBS_DIM_NO_ACC,
        history_len: int = ODOM_HISTORY_LEN,
        intra_op_num_threads: int = 1,
        inter_op_num_threads: int = 1,
    ) -> None:
        self._model_path = Path(model_path)
        self._obs_dim = int(obs_dim)
        self._history_len = int(history_len)

        if self._obs_dim not in (OBS_DIM_NO_ACC, OBS_DIM_WITH_ACC):
            raise ValueError(
                f"不支持的观测维度 {self._obs_dim}，"
                f"应为 {OBS_DIM_NO_ACC} 或 {OBS_DIM_WITH_ACC}"
            )

        if not self._model_path.is_file():
            raise FileNotFoundError(
                f"里程计模型不存在: {self._model_path}。"
                "请在 config/odometry_models.yaml 中检查该模型的 model_path，"
                "接口契约见 model/README.md。"
            )

        # 延迟导入：onnxruntime 只在真正加载模型时才需要，便于纯逻辑单测。
        import onnxruntime as ort

        options = ort.SessionOptions()
        options.intra_op_num_threads = int(intra_op_num_threads)
        options.inter_op_num_threads = int(inter_op_num_threads)
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL

        try:
            self._session = ort.InferenceSession(
                str(self._model_path),
                sess_options=options,
                providers=["CPUExecutionProvider"],
            )
        except Exception as exc:  # noqa: BLE001 - onnxruntime 抛出的异常类型不稳定
            raise OnnxContractError(
                f"加载里程计 ONNX 失败: {self._model_path}: {exc}"
            ) from exc

        validate_io_contract(
            {item.name: item.shape for item in self._session.get_inputs()},
            {item.name: item.shape for item in self._session.get_outputs()},
            obs_dim=self._obs_dim,
            history_len=self._history_len,
        )

        # 预分配输入缓冲，避免每帧在热路径上分配。
        self._obs_buffer = np.zeros(
            (1, self._history_len, self._obs_dim), dtype=np.float32
        )
        self._prev_buffer = np.zeros((1, ODOM_DIM), dtype=np.float32)

    @property
    def model_path(self) -> Path:
        """返回已加载的模型路径。"""
        return self._model_path

    @property
    def obs_dim(self) -> int:
        """返回模型期望的单帧观测维度。"""
        return self._obs_dim

    def run(
        self,
        obs_history: np.ndarray,
        previous_odom_norm: Sequence[float],
    ) -> tuple[np.ndarray, np.ndarray]:
        """执行一次前向。

        返回 ``(delta_xy, next_previous_odom_norm)``：前者是米制的世界系位移
        增量 ``[2]``，后者是应当回填给下一步的自回归反馈 ``[2]``。
        """
        history = np.asarray(obs_history, dtype=np.float32)
        if history.shape != (self._history_len, self._obs_dim):
            raise ValueError(
                f"obs_history 形状应为 [{self._history_len}, {self._obs_dim}]，"
                f"实际 {history.shape}"
            )

        self._obs_buffer[0] = history
        self._prev_buffer[0] = np.asarray(
            previous_odom_norm, dtype=np.float32
        ).reshape(ODOM_DIM)

        delta, next_norm = self._session.run(
            [OUTPUT_PRED_DELTA, OUTPUT_NEXT_ODOM_NORM],
            {
                INPUT_OBS_HISTORY: self._obs_buffer,
                INPUT_PREV_ODOM_NORM: self._prev_buffer,
            },
        )

        delta = np.asarray(delta, dtype=np.float32).reshape(-1)
        next_norm = np.asarray(next_norm, dtype=np.float32).reshape(-1)
        if delta.shape != (ODOM_DIM,) or next_norm.shape != (ODOM_DIM,):
            raise OnnxContractError(
                f"ONNX 模型输出维度应为 [{ODOM_DIM}]，"
                f"实际 delta={delta.shape}, next_norm={next_norm.shape}"
            )
        return delta, next_norm
