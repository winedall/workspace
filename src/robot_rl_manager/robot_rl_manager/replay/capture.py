"""Replay 样本的归一化和持久化工具。"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from robot_rl_manager.replay.sequence_manifest import (
    ReplaySequenceRuntimeSpec,
    find_sequence_repo_root,
    build_replay_sequence_manifest,
    write_replay_sequence_manifest,
)

def normalize_replay_sample(sample: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
    """把任意 sample 映射归一化为普通 ``dict``，并强制转成 NumPy 数组。

    Args:
        sample: 形如 ``{"obs": array(...)}`` 的输入样本映射。

    Returns:
        统一后的字典，键保持字符串，值都转换为 ``np.ndarray``。
    """
    normalized: dict[str, np.ndarray] = {}
    for key, value in sample.items():
        array = np.asarray(value)
        if array.dtype.kind == "f" and array.dtype != np.float32:
            array = array.astype(np.float32, copy=False)
        normalized[str(key)] = array
    return normalized


def _normalize_sequence_frame(sample: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
    """把单帧样本归一化为适合序列堆叠的帧格式。

    单输入模型的 ``build_model_input`` 往往返回一维观测向量，例如 ``(144,)``。
    为了让序列样本在磁盘上保留显式 batch 维，这里把每个张量都扩成 ``(1, ...)``。
    这样 replay 拆帧后拿到的就是与模型前向一致的单 batch 输入。
    """
    normalized = normalize_replay_sample(sample)
    framed: dict[str, np.ndarray] = {}
    for key, value in normalized.items():
        framed[key] = np.expand_dims(value, axis=0)
    return framed


def write_replay_sample(path: Path, sample: Mapping[str, np.ndarray]) -> None:
    """归一化 sample 后写成 ``.npz`` 文件。

    Args:
        path: 输出文件路径。
        sample: 待保存的样本输入映射。

    Notes:
        这里只负责保存，不负责生成新的机器人轨迹或录制逻辑。
    """
    sample_path = Path(path)
    sample_path.parent.mkdir(parents=True, exist_ok=True)
    normalized = normalize_replay_sample(sample)
    np.savez(sample_path, **normalized)


def build_sequence_sample_path(samples_root: Path, model_name: str) -> Path:
    """构造单个模型的默认序列样本路径。

    Args:
        samples_root: 样本根目录，通常是 ``config/data/samples``。
        model_name: 模型名称，例如 ``t4_heart``。

    Returns:
        该模型对应的默认序列样本路径。
    """
    return Path(samples_root) / model_name / "inputs_seq.npz"


def stack_replay_sample_sequence(frames: list[Mapping[str, np.ndarray]]) -> dict[str, np.ndarray]:
    """把多帧模型输入序列堆叠成一个可保存的 ``npz`` 字典。

    Args:
        frames: 连续帧输入列表，每一帧都是一个 ``{"obs": array(...)}`` 之类的映射。

    Returns:
        以输入名为 key、时间维为第 0 维的 stacked 数组字典。

    Raises:
        ValueError: 序列为空，或者不同帧的输入 key / shape 不一致时抛出。
    """
    if not frames:
        raise ValueError("replay sequence cannot be empty")

    normalized_frames = [_normalize_sequence_frame(frame) for frame in frames]
    return _stack_normalized_sequence_frames(normalized_frames)


def _stack_normalized_sequence_frames(
    frames: list[Mapping[str, np.ndarray]],
) -> dict[str, np.ndarray]:
    """把已经归一化过的序列帧直接堆叠成 npz 字典。

    Args:
        frames: 已经完成单帧归一化的连续帧输入列表。

    Returns:
        以输入名为 key、时间维为第 0 维的 stacked 数组字典。
    """
    if not frames:
        raise ValueError("replay sequence cannot be empty")

    expected_keys = tuple(frames[0].keys())
    for frame_index, frame in enumerate(frames[1:], start=1):
        if tuple(frame.keys()) != expected_keys:
            raise ValueError(
                "replay sequence frame keys mismatch at "
                f"index {frame_index}: expected {expected_keys}, got {tuple(frame.keys())}"
            )

    stacked: dict[str, np.ndarray] = {}
    for key in expected_keys:
        values = [np.asarray(frame[key]) for frame in frames]
        reference_shape = values[0].shape
        for frame_index, value in enumerate(values[1:], start=1):
            if value.shape != reference_shape:
                raise ValueError(
                    "replay sequence frame shape mismatch for "
                    f"key '{key}' at index {frame_index}: expected {reference_shape}, got {value.shape}"
                )
        stacked[key] = np.stack(values, axis=0)
    return stacked


def build_replay_sequence_bundle(
    *,
    samples_root: Path,
    model_info,
    frames: list[Mapping[str, np.ndarray]],
) -> tuple[Path, Path]:
    """写出序列样本和对应 manifest，并返回二者路径。"""
    sample_path = build_sequence_sample_path(samples_root, str(model_info.name))
    repo_root = find_sequence_repo_root(samples_root)
    manifest = build_replay_sequence_manifest(
        repo_root=repo_root,
        sample_path=sample_path,
        model_info=model_info,
        frames=frames,
        sequence_length=len(frames),
        runtime=ReplaySequenceRuntimeSpec(),
    )
    sample_path.parent.mkdir(parents=True, exist_ok=True)
    normalized_frames = [_normalize_sequence_frame(frame) for frame in frames]
    np.savez(sample_path, **_stack_normalized_sequence_frames(normalized_frames))
    manifest_path = sample_path.parent / "manifest.yaml"
    write_replay_sequence_manifest(manifest_path, manifest)
    return sample_path, manifest_path


def write_replay_sequence_sample(path: Path, frames: list[Mapping[str, np.ndarray]]) -> None:
    """把多帧模型输入序列写成 ``.npz`` 文件。

    Args:
        path: 输出文件路径。
        frames: 连续帧输入列表。
    """
    sample_path = Path(path)
    sample_path.parent.mkdir(parents=True, exist_ok=True)
    normalized = stack_replay_sample_sequence(frames)
    np.savez(sample_path, **normalized)


def split_replay_sequence_sample(sample: Mapping[str, np.ndarray]) -> list[dict[str, np.ndarray]]:
    """把 replay 样本拆成逐帧输入列表。

    Args:
        sample: 从 ``.npz`` 里读出来的输入映射。

    Returns:
        逐帧输入列表。

    Notes:
        序列样本的每个张量都必须至少有 2 个维度，并且第 0 维长度一致。
    """
    normalized = normalize_replay_sample(sample)
    if not normalized:
        return []

    values = list(normalized.values())
    if not values:
        return []

    for key, value in normalized.items():
        if value.ndim < 2:
            raise ValueError(
                f"replay sequence tensor '{key}' must have at least 2 dimensions, got {value.ndim}"
            )

    frame_count = values[0].shape[0]
    for key, value in normalized.items():
        if value.shape[0] != frame_count:
            raise ValueError(
                f"replay sequence length mismatch for key '{key}': "
                f"expected {frame_count}, got {value.shape[0]}"
            )

    frames: list[dict[str, np.ndarray]] = []
    for frame_index in range(frame_count):
        frame: dict[str, np.ndarray] = {}
        for key, value in normalized.items():
            frame[key] = np.asarray(value[frame_index])
        frames.append(frame)
    return frames


def write_model_input_sequence_sample(
    samples_root: Path,
    model_name: str,
    frames: list[Mapping[str, np.ndarray]],
) -> Path:
    """把模型输入序列写入默认目录结构。

    Args:
        samples_root: 样本根目录，通常是 ``config/data/samples``。
        model_name: 当前模型名称。
        frames: 连续帧输入列表。

    Returns:
        实际写入的序列样本文件路径。
    """
    sample_path = build_sequence_sample_path(samples_root, model_name)
    write_replay_sequence_sample(sample_path, frames)
    return sample_path


@dataclass
class LatestReplaySequenceRecorder:
    """按模型录制最新一段 replay 序列，并在结束时覆盖写盘。

    这个 recorder 只保存当前模型的最新一段序列，不保留历史 session。
    当模型切换时，外部调用 ``finalize(reason="model_switch")`` 即可落盘旧段并清空状态。
    """

    samples_root: Path
    model_name: str | None = None
    model_info: object | None = None
    mode: str = "always"
    max_seconds: float = 0.0
    max_frames: int = 0
    overwrite_latest: bool = True
    _frames: list[dict[str, np.ndarray]] = field(default_factory=list, init=False, repr=False)
    _start_time_sec: float | None = field(default=None, init=False, repr=False)
    _current_model_name: str | None = field(default=None, init=False, repr=False)
    _paused_model_name: str | None = field(default=None, init=False, repr=False)

    def append(self, model_info: object, sample: Mapping[str, np.ndarray], now_sec: float) -> Path | None:
        """追加一帧输入样本，并在满足停止条件时自动落盘。

        Args:
            model_info: 当前模型元数据。
            sample: 当前帧模型输入。
            now_sec: 当前时间戳，单位秒。

        Returns:
            如果本次调用触发了写盘，则返回实际落盘路径；否则返回 None。
        """
        model_name = str(getattr(model_info, "name"))
        if self._paused_model_name == model_name:
            return None
        if self._current_model_name != model_name:
            self._start_new_session(model_info, now_sec)

        normalized = normalize_replay_sample(sample)
        self._frames.append(normalized)
        self._current_model_name = model_name
        self.model_info = model_info

        if self._should_finalize(now_sec):
            sample_path = self.finalize(reason=self.mode)
            if sample_path is not None:
                self._paused_model_name = model_name
            return sample_path
        return None

    def finalize(self, reason: str = "manual") -> Path | None:
        """把当前序列落盘并清空会话状态。

        Args:
            reason: 结束原因，用于调试和日志。

        Returns:
            实际写入的样本路径；如果当前没有有效帧，则返回 None。
        """
        del reason
        if not self._frames or self._current_model_name is None:
            self._clear_session()
            return None
        if not self.overwrite_latest:
            # 当前产品形态只保留最新一份，这里保留参数接口但不执行其他命名策略。
            pass
        if self.model_info is None:
            raise RuntimeError("current model info is required to write replay sequence manifest")
        sample_path, _manifest_path = build_replay_sequence_bundle(
            samples_root=self.samples_root,
            model_info=self.model_info,
            frames=self._frames,
        )
        self._clear_session()
        return sample_path

    def pause_current_model(self) -> None:
        """暂停当前模型的序列录制，等待模型切换后再开启新会话。"""
        if self._current_model_name is None:
            return
        self._paused_model_name = self._current_model_name
        self._clear_session(clear_pause=False)

    def reset(self) -> None:
        """清空所有录制状态。"""
        self._clear_session(clear_pause=True)

    def has_active_session(self) -> bool:
        """返回当前是否正在录制有效序列。"""
        return self._current_model_name is not None and bool(self._frames)

    def _start_new_session(self, model_info: object, now_sec: float) -> None:
        """启动新的模型序列录制会话。"""
        model_name = str(getattr(model_info, "name"))
        self._frames = []
        self._start_time_sec = float(now_sec)
        self._current_model_name = model_name
        self.model_info = model_info
        if self._paused_model_name != model_name:
            self._paused_model_name = None

    def _should_finalize(self, now_sec: float) -> bool:
        """判断当前会话是否达到结束条件。"""
        if self.mode == "always":
            return False
        if self.mode == "time":
            if self.max_seconds <= 0.0 or self._start_time_sec is None:
                return False
            return (now_sec - self._start_time_sec) >= self.max_seconds
        if self.mode == "frames":
            if self.max_frames <= 0:
                return False
            return len(self._frames) >= self.max_frames
        raise ValueError(f"unknown replay sequence record mode: {self.mode}")

    def _clear_session(self, *, clear_pause: bool = False) -> None:
        """清空当前会话状态，但可选择保留暂停标记。"""
        self._frames = []
        self._start_time_sec = None
        self._current_model_name = None
        self.model_info = None
        if clear_pause:
            self._paused_model_name = None
