"""Replay helpers for offline RL model validation."""

from robot_rl_manager.replay.compare import (
    ArrayComparisonResult,
    OutputComparisonResult,
    OutputSetComparisonResult,
    compare_output_arrays,
    compare_output_sets,
)
from robot_rl_manager.replay.capture import (
    LatestReplaySequenceRecorder,
    build_sequence_sample_path,
    split_replay_sequence_sample,
    stack_replay_sample_sequence,
    write_model_input_sequence_sample,
    write_replay_sample,
    write_replay_sequence_sample,
)
from robot_rl_manager.replay.report import ReplayReport, report_to_dict, report_to_json, report_to_markdown
from robot_rl_manager.replay.runner import OnnxRuntimeRunner, ReplayRunner, RknnRunner, create_replay_runner

__all__ = [
    "ArrayComparisonResult",
    "OutputComparisonResult",
    "OutputSetComparisonResult",
    "ReplayReport",
    "ReplayRunner",
    "LatestReplaySequenceRecorder",
    "build_sequence_sample_path",
    "OnnxRuntimeRunner",
    "RknnRunner",
    "compare_output_arrays",
    "compare_output_sets",
    "create_replay_runner",
    "write_model_input_sequence_sample",
    "report_to_dict",
    "report_to_json",
    "report_to_markdown",
    "split_replay_sequence_sample",
    "stack_replay_sample_sequence",
    "write_replay_sample",
    "write_replay_sequence_sample",
]
