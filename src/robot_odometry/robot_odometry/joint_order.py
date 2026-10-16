"""T4-27DOF 里程计模型的关节顺序与观测布局定义。

观测布局必须与训练仓库 ``proprio_odometry/odometry/data.py`` 完全一致，
否则输入错位、预测失效。当前 schema（OBS_SCHEMA_VERSION = 3）：

    88 维（不含基座加速度）：
        joint_pos_current(27)
        + joint_vel(27)
        + base_quat(4)
        + base_ang_vel(3)
        + joint_pos_previous(27)

    91 维（含基座加速度）：
        joint_pos_current(27)
        + joint_vel(27)
        + base_quat(4)
        + base_ang_vel(3)
        + base_acc(3)
        + joint_pos_previous(27)

注意 ``base_acc`` 插在 ``base_ang_vel`` 与 ``joint_pos_previous`` 之间，因此
88 维版本不是简单地取 91 维的前 88 列，而是跳过 [61:64]。本模块用与训练侧
相同的列索引做选择，避免两处各自硬编码。

**action 不再是模型输入**。旧 schema（v2，含 action）下 88 维的 [61:88] 是
action，新 schema 下同样的 [61:88] 是 joint_pos_previous —— 维度相同、含义
完全不同。跨 schema 喂数据会静默出错，因此 ``observation_spec`` 同时返回
schema 版本号供校验。

27 个关节的顺序来源：``t4_27dof.xml`` 的 ``<joint>`` 定义顺序（body-tree
先序遍历），即 左臂7 → 右臂7 → waist_yaw → 左腿6 → 右腿6。

与部署侧的一致性：``robot_rl_manager`` 中 t4 系列策略的 action 维度同为 27，
``policy/adapters`` 的 ``AMP_POLICY_IDX`` 跳过 waist_pitch / waist_roll 后，
剩余的 27 个关节顺序与本文件完全相同。该一致性由 ``test_joint_order.py`` 守护。
"""

from __future__ import annotations

from typing import Mapping, Sequence, Tuple

import numpy as np

# 当前布局对应训练仓库的 OBS_SCHEMA_VERSION，仅用于人工对照，不参与任何校验：
# 布局的正确性由 OBS_LAYOUT_* 字符串与 ONNX 图的输入形状共同保证。
OBS_SCHEMA_VERSION = 3

# 两种布局的观测维度。
OBS_DIM_NO_ACC = 88
OBS_DIM_WITH_ACC = 91



# 各字段在布局中的列区间。
JOINT_POS_START, JOINT_POS_END = 0, 27
JOINT_VEL_START, JOINT_VEL_END = 27, 54
QUAT_START, QUAT_END = 54, 58
ANG_VEL_START, ANG_VEL_END = 58, 61
BASE_ACC_START, BASE_ACC_END = 61, 64
PREV_JOINT_POS_START, PREV_JOINT_POS_END = 64, 91

OBS_LAYOUT_NO_ACC = (
    "joint_pos_current(27) + joint_vel(27) + base_quat(4) + "
    "base_ang_vel(3) + joint_pos_previous(27)"
)
OBS_LAYOUT_WITH_ACC = (
    "joint_pos_current(27) + joint_vel(27) + base_quat(4) + "
    "base_ang_vel(3) + base_acc(3) + joint_pos_previous(27)"
)

# 喂给模型的列索引。无加速度版本跳过 base_acc 所在的 [61:64]。
OBS_INDICES_NO_ACC = np.concatenate(
    [np.arange(0, 61), np.arange(64, 91)]
).astype(np.int64)
OBS_INDICES_WITH_ACC = np.arange(0, 91, dtype=np.int64)

# 模型训练时的关节数量。
ODOM_JOINT_COUNT = 27

# 观测历史帧数（50 帧 @ 50Hz = 1 秒）。
ODOM_HISTORY_LEN = 50

# 里程计输出维度：世界系下相邻帧位移增量 (dx, dy)。
ODOM_DIM = 2

# 模型输入的采样周期，由训练侧 step_dt 决定。
ODOM_STEP_DT = 0.02

# 27 个关节名，顺序即模型输入顺序。
ODOM_JOINT_NAMES: Tuple[str, ...] = (
    # 左臂
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_yaw",
    "left_wrist_pitch",
    "left_wrist_roll",
    # 右臂
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
    "right_wrist_yaw",
    "right_wrist_pitch",
    "right_wrist_roll",
    # 腰（仅 yaw，waist_pitch/waist_roll 在 27DOF 模型中被固定）
    "waist_yaw",
    # 左腿
    "left_hip_pitch",
    "left_hip_roll",
    "left_hip_yaw",
    "left_knee",
    "left_ankle_pitch",
    "left_ankle_roll",
    # 右腿
    "right_hip_pitch",
    "right_hip_roll",
    "right_hip_yaw",
    "right_knee",
    "right_ankle_pitch",
    "right_ankle_roll",
)

if len(ODOM_JOINT_NAMES) != ODOM_JOINT_COUNT:
    raise AssertionError(
        f"ODOM_JOINT_NAMES 长度 {len(ODOM_JOINT_NAMES)} != {ODOM_JOINT_COUNT}"
    )

if len(set(ODOM_JOINT_NAMES)) != ODOM_JOINT_COUNT:
    raise AssertionError("ODOM_JOINT_NAMES 存在重复关节名")


def observation_spec(
    use_base_acceleration: bool,
) -> Tuple[int, str, np.ndarray]:
    """返回 ``(观测维度, 布局说明, 列索引)``。

    与训练仓库 ``data.observation_spec`` 同构，用于模型加载时校验配置。
    """
    if use_base_acceleration:
        return (OBS_DIM_WITH_ACC, OBS_LAYOUT_WITH_ACC, OBS_INDICES_WITH_ACC)
    return (OBS_DIM_NO_ACC, OBS_LAYOUT_NO_ACC, OBS_INDICES_NO_ACC)


def joints_by_name(
    states: Mapping[str, Sequence[float]],
) -> Tuple[np.ndarray, np.ndarray]:
    """按模型顺序把 ``{关节名: (位置, 速度)}`` 展开成 27 维位置与速度数组。

    ``states`` 允许包含模型不使用的额外关节（例如 head / waist_pitch /
    waist_roll），本函数只按名索引取用，不做位置假设，因此不受上游关节顺序影响。

    缺少任一模型关节时抛出 ``KeyError``，并在消息中列出全部缺失关节。
    """
    missing = [name for name in ODOM_JOINT_NAMES if name not in states]
    if missing:
        raise KeyError(
            "硬件关节状态缺少里程计模型所需的关节: " + ", ".join(missing)
        )

    positions = np.empty(ODOM_JOINT_COUNT, dtype=np.float32)
    velocities = np.empty(ODOM_JOINT_COUNT, dtype=np.float32)

    for index, name in enumerate(ODOM_JOINT_NAMES):
        position, velocity = states[name]
        positions[index] = float(position)
        velocities[index] = float(velocity)

    return positions, velocities


def build_storage_observation(
    joint_pos: np.ndarray,
    joint_vel: np.ndarray,
    base_quat: Sequence[float],
    base_ang_vel: Sequence[float],
    base_acc: Sequence[float],
    previous_joint_pos: np.ndarray,
) -> np.ndarray:
    """拼出 91 维「存储布局」观测，再用列索引裁成 88 或 91 维。

    与训练仓库 ``Trajectory.build_full_observation`` + ``build_observation``
    的组合等价，保证两处布局永远一致。
    """
    joint_pos = np.asarray(joint_pos, dtype=np.float32).reshape(-1)
    joint_vel = np.asarray(joint_vel, dtype=np.float32).reshape(-1)
    base_quat = np.asarray(base_quat, dtype=np.float32).reshape(-1)
    base_ang_vel = np.asarray(base_ang_vel, dtype=np.float32).reshape(-1)
    base_acc = np.asarray(base_acc, dtype=np.float32).reshape(-1)
    previous_joint_pos = np.asarray(previous_joint_pos, dtype=np.float32).reshape(-1)

    for name, array in (
        ("joint_pos", joint_pos),
        ("joint_vel", joint_vel),
        ("previous_joint_pos", previous_joint_pos),
    ):
        if array.shape != (ODOM_JOINT_COUNT,):
            raise ValueError(
                f"{name} 维度应为 [{ODOM_JOINT_COUNT}]，实际 {array.shape}"
            )
    for name, array, size in (
        ("base_quat", base_quat, 4),
        ("base_ang_vel", base_ang_vel, 3),
        ("base_acc", base_acc, 3),
    ):
        if array.shape != (size,):
            raise ValueError(f"{name} 维度应为 [{size}]，实际 {array.shape}")

    observation = np.concatenate(
        (
            joint_pos,
            joint_vel,
            base_quat,
            base_ang_vel,
            base_acc,
            previous_joint_pos,
        )
    )

    if observation.shape != (OBS_DIM_WITH_ACC,):
        raise AssertionError(
            f"存储布局维度应为 [{OBS_DIM_WITH_ACC}]，实际 {observation.shape}"
        )

    return observation


def select_observation(
    storage_observation: np.ndarray,
    use_base_acceleration: bool,
) -> np.ndarray:
    """把 91 维存储布局裁成模型实际需要的 88 或 91 维。"""
    _, _, indices = observation_spec(use_base_acceleration)
    selected = np.asarray(storage_observation, dtype=np.float32)[indices]
    expected = OBS_DIM_WITH_ACC if use_base_acceleration else OBS_DIM_NO_ACC
    if selected.shape != (expected,):
        raise ValueError(
            f"观测维度应为 [{expected}]，实际 {selected.shape}"
        )
    return selected.astype(np.float32, copy=False)
