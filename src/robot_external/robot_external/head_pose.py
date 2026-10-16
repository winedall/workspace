"""头部深度相机的正运动学计算（纯数学，与 ROS 解耦，便于单元测试）。

背景
----
``/head_pose`` 描述的是**头部深度相机相对于 odom 坐标系的位姿**。本仓库的
odom 与 base（``torso_link``）在 x、y 方向重合，但位于地面上，因此
``odom`` 相对 ``base`` 只差一个 +z 的高度偏置（默认 0.83 m，见
``base_height``）。

坐标系与关节链（全部取自 MuJoCo XML，见 robot_assets 的
``Z01_std_football_player/xml/Z01_std_football_player.xml``）::

    torso_link  --head_pitch(绕 y)-->  head_pitch_link   pos 0.00155 0 0.3155
                --head_yaw(绕 z)-->    head_yaw_link     pos 0        0 0.0795
                --fixed-->             depth_cam         pos 0.069    0.01 0.068

两个关节的原点与子 link 原点重合，因此正运动学等价于::

    p_base = t_pitch + Ry(pitch) · ( t_yaw + Rz(yaw) · t_cam )
    R_base = Ry(pitch) · Rz(yaw) · R_yaw_link_to_cam

姿态约定
--------
``R_yaw_link_to_cam`` 来自 XML 的 ``xyaxes="0 -1 0 0 0 1"``（x 轴指向
-y，y 轴指向 +z），即相机的光轴沿 ``head_yaw_link`` 的 +x。本模块输出
**ROS 光学系约定 x 右 / y 下 / z 前**，与图像话题使用的
``depth_cam_optical_frame`` 一致：深度图按内参反投影得到的三维点可以直接
左乘该位姿变换到 odom，不需要再补一次轴系旋转。

运动学推导用的中间坐标系（``head_yaw_link``）与输出光学系之间的常量旋转
为 ``Rz(-90°)·Ry(90°)``，其四元数为 ``(-0.5, 0.5, -0.5, 0.5)``。

基座姿态
--------
基座**倾斜**（roll / pitch）来自 IMU：节点订阅 ``/robot/imu/nav_all``
取基座姿态四元数，经 :func:`yaw_stripped_orientation` 去除航向 yaw 后
作为 ``base_tilt`` 传入本模块——odom 的航向按需求始终跟随 base
（见下），因此 yaw 不参与计算，只有相对重力的倾斜会改变输出。
机器人摔倒时躯干 roll/pitch 接近 ±90°，输出位置与姿态随之大幅变化。

基座高度仍用常数偏置近似（默认 0.83 m，见 ``base_height``）：odom 原点
始终位于地面、且 x/y 跟随 base。真实机器人躺倒后躯干离地高度远小于
0.83 m，而本模块不做高度估计——**摔倒后 z 的绝对值仍是站立近似值**，
这是本近似的已知边界（姿态与 x/y 的变化是真实的）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

# 四元数统一用 (x, y, z, w) 顺序存放，与 geometry_msgs/Quaternion 字段一致。
Quaternion = Tuple[float, float, float, float]
Vector3 = Tuple[float, float, float]

# head_yaw_link 本体系 -> ROS 光学系（x 右 / y 下 / z 前）的常量旋转四元数。
# 由 XML 的 xyaxes="0 -1 0 0 0 1" 推出：相机 x 轴 = -y_link，y 轴 = -z_link，
# z 轴 = +x_link，对应旋转 Rz(-90°)·Ry(90°)。
_OPTICAL_IN_YAW_LINK: Quaternion = (-0.5, 0.5, -0.5, 0.5)


@dataclass(frozen=True)
class HeadCameraChain:
    """头部相机安装链的几何常数（单位 m / rad）。

    默认值取自 MuJoCo XML，可通过 ROS 参数覆盖以适配真机装配或其它机型：

    - ``base_height``：odom 原点（地面）到 base 原点的 +z 偏置；
    - ``head_pitch_offset``：torso_link -> head_pitch_link 的平移；
    - ``head_yaw_offset``：head_pitch_link -> head_yaw_link 的平移；
    - ``camera_offset``：head_yaw_link -> 相机的平移。
    """

    base_height: float = 0.83
    head_pitch_offset: Vector3 = (0.00155, 0.0, 0.3155)
    head_yaw_offset: Vector3 = (0.0, 0.0, 0.0795)
    camera_offset: Vector3 = (0.069, 0.01, 0.068)


@dataclass(frozen=True)
class HeadCameraPose:
    """一次正运动学的结果：位置 (m) 与姿态四元数 (x, y, z, w)。"""

    position: Vector3
    orientation: Quaternion


def quaternion_multiply(lhs: Quaternion, rhs: Quaternion) -> Quaternion:
    """四元数乘法 ``lhs ⊗ rhs``（与旋转矩阵相乘同序）。"""
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_from_axis_angle(axis: Sequence[float], angle: float) -> Quaternion:
    """绕给定单位轴旋转 ``angle`` 弧度的四元数。"""
    norm = math.sqrt(sum(component * component for component in axis))
    if norm <= 0.0:
        raise ValueError("axis must be non-zero")
    half = 0.5 * angle
    scale = math.sin(half) / norm
    return (axis[0] * scale, axis[1] * scale, axis[2] * scale, math.cos(half))


def quaternion_normalize(quat: Quaternion) -> Quaternion:
    """归一化四元数；零四元数返回单位四元数。"""
    norm = math.sqrt(sum(component * component for component in quat))
    if norm <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(component / norm for component in quat)  # type: ignore[return-value]


def quaternion_rotate(quat: Quaternion, vector: Sequence[float]) -> Vector3:
    """用四元数旋转三维向量（``v' = q ⊗ v ⊗ q*``）。"""
    x, y, z, w = quat
    vx, vy, vz = vector
    # t = 2 * (q_vec × v)，结果 v' = v + w·t + q_vec × t，避免构造旋转矩阵。
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def yaw_stripped_orientation(quat: Quaternion) -> Quaternion:
    """去掉基座姿态中的航向 yaw，只保留相对重力的倾斜（roll/pitch）。

    odom 的航向按需求跟随 base（x/y、yaw 与 base 对齐），因此把 IMU 的
    世界系姿态接入 /head_pose 时必须先剥离 yaw：输出应只在机器人**倾斜**
    （行走晃动、摔倒）时变化，转向时保持稳定。

    实现上不走欧拉角分解（避免万向锁分支）：分别估计 base x 轴与 z 轴的
    永平投影方向作为航向，取投影更长的那条（长度 ≥ 1/√2，噪声放大 ≤ √2 倍）。

    - x 投影方向 = ``atan2(R10, R00)``，在标准 ZYX 分解（pitch ∈ [-90°,
      90°]）下恒等于 yaw，与 roll 无关；
    - z 投影方向 = ``atan2(R12, R02)`` 还混入 ``atan2(-sinφ, sinθcosφ)``
      一项，需用 ``atan2(-R21, -R20·R22)``（代入 sinφ = R21/cosθ 后 cosθ
      相消）扣除。该项的信号正比于 |cosθ|，机身接近竖直（躺倒）时落入噪声
      底，此时置 0——该角度本来就与 yaw 在数据上不可分（坐标奇异），置 0
      在无 roll 时精确、有 roll 时退化为连续的有限误差。

    已知边界：躯干恰好竖直（|pitch| 接近 90° 且 |cosθ| 低于四元数噪声）
    时航向不可观测，odom 的 x/y 指向存在坐标奇异，输出的水平位置会有有限
    抖动；离开该姿态后立即恢复精确。

    :param quat: 基座在世界系的姿态四元数 ``(x, y, z, w)``。
    :return: 去除 yaw 后的四元数 ``(x, y, z, w)``（单位）。
    """
    qx, qy, qz, qw = quaternion_normalize(quat)
    rot0, rot1, rot2 = quaternion_to_matrix((qx, qy, qz, qw))
    # 列 0 = base x 轴、列 2 = base z 轴在世界系的分量；rot2[0] = -sinθ。
    mag_x = math.hypot(rot0[0], rot1[0])
    mag_z = math.hypot(rot0[2], rot1[2])
    if mag_x >= mag_z:
        # x 投影：|cosθ| 较长，方向即 yaw，完全不掺 roll。
        heading = math.atan2(rot1[0], rot0[0])
    else:
        # z 投影：先取其方向，再扣除随 roll 混入的 atan2(-sinφ, sinθcosφ)。
        heading = math.atan2(rot1[2], rot0[2])
        numerator = -rot2[1]  # -cosθ·sinφ
        denominator = (-rot2[0]) * rot2[2]  # sinθ·cosθ·cosφ
        if math.hypot(numerator, denominator) > 2e-3:
            heading -= math.atan2(numerator, denominator)
    return quaternion_normalize(
        quaternion_multiply(
            quaternion_from_axis_angle((0.0, 0.0, 1.0), -heading), (qx, qy, qz, qw)
        )
    )


def quaternion_to_matrix(quat: Quaternion) -> Tuple[Vector3, Vector3, Vector3]:
    """四元数转 3x3 旋转矩阵（按行返回），用于测试与外部校验。"""
    x, y, z, w = quaternion_normalize(quat)
    return (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ),
        (
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ),
        (
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ),
    )


def compute_head_camera_pose(
    head_pitch: float,
    head_yaw: float,
    chain: HeadCameraChain | None = None,
    base_tilt: Quaternion | None = None,
) -> HeadCameraPose:
    """由头部关节角与基座倾斜计算深度相机在 odom 系的位姿。

    :param head_pitch: ``head_pitch`` 关节角（rad，绕 base 的 y 轴）。
    :param head_yaw: ``head_yaw`` 关节角（rad，绕 ``head_yaw_link`` 的 z 轴）。
    :param chain: 安装链几何常数；``None`` 时使用 XML 默认值。
    :param base_tilt: 基座相对 odom 的**去 yaw 倾斜**四元数 ``(x, y, z, w)``
        （来自 :func:`yaw_stripped_orientation`）；``None`` 视为直立（单位四元数）。
    :return: 光学系（x 右 / y 下 / z 前）在 odom 系下的位置与姿态。

    倾斜只旋转“相机相对 base 原点”的链向量（含三个装配偏移），``base_height``
    高度偏置沿 odom 的 +z 叠加、不参与旋转——odom 原点在地面、x/y 跟随 base。
    """
    geometry = chain if chain is not None else HeadCameraChain()
    tilt = base_tilt if base_tilt is not None else (0.0, 0.0, 0.0, 1.0)

    quat_pitch = quaternion_from_axis_angle((0.0, 1.0, 0.0), head_pitch)
    quat_yaw = quaternion_from_axis_angle((0.0, 0.0, 1.0), head_yaw)
    # R_base_yaw_link = Ry(pitch)·Rz(yaw)：先 yaw 再 pitch，与关节链一致。
    quat_head = quaternion_multiply(quat_pitch, quat_yaw)

    # 平移链：t_pitch 固定；t_yaw 只经过 pitch 旋转；t_cam 经过 pitch+yaw 旋转。
    yaw_rotated = quaternion_rotate(quat_pitch, geometry.head_yaw_offset)
    cam_rotated = quaternion_rotate(quat_head, geometry.camera_offset)
    position_in_base = (
        geometry.head_pitch_offset[0] + yaw_rotated[0] + cam_rotated[0],
        geometry.head_pitch_offset[1] + yaw_rotated[1] + cam_rotated[1],
        geometry.head_pitch_offset[2] + yaw_rotated[2] + cam_rotated[2],
    )
    # 基座倾斜旋转整条链向量；base_height 是 odom 系的地面高度偏置，最后再加。
    tilted = quaternion_rotate(tilt, position_in_base)
    position = (tilted[0], tilted[1], tilted[2] + geometry.base_height)

    orientation = quaternion_normalize(
        quaternion_multiply(
            tilt,
            quaternion_multiply(quat_head, _OPTICAL_IN_YAW_LINK),
        )
    )
    return HeadCameraPose(position=position, orientation=orientation)
