"""Policy IMU 数据对象。

PolicyImu 存放 policy 直接需要的语义输入（姿态、角速度、线速度），
与具体 IMU 厂商无关。转换逻辑由 rl_manager_node 负责。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PolicyImu:
    """厂商无关的 IMU 语义输入。所有值均为弧度制/国际单位制。

    由 rl_manager_node 从具体厂商消息转换而来，
    adapter 仅读取此结构，不关心底层 IMU 型号。
    """

    euler_rad: list[float]
    """[roll, pitch, yaw] 姿态欧拉角，单位 rad。"""

    orientation_quat: list[float]
    """[w, x, y, z] 姿态四元数，由厂商硬件直接提供。"""

    angular_velocity: list[float]
    """[wx, wy, wz] body frame 角速度，单位 rad/s。"""

    linear_velocity: list[float]
    """[ve, vn, vu] ENU 坐标系导航解算线速度，单位 m/s。"""
