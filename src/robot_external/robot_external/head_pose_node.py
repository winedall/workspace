"""发布头部深度相机位姿（``/head_pose``）的节点。

话题语义
--------
``/head_pose``（``geometry_msgs/Pose``）给出**头部深度相机在 odom 坐标系
下的位姿**：位置单位 m，姿态为四元数 ``(x, y, z, w)``，坐标轴采用 ROS
光学系约定 x 右 / y 下 / z 前，与图像话题的 ``depth_cam_optical_frame``
一致。

odom 与 base（``torso_link``）在 x、y 方向重合、odom 位于地面上，因此
高度用常数偏置近似（``base_height``，默认 0.83 m）。**基座倾斜来自 IMU**：
订阅 ``/robot/imu/nav_all`` 取基座姿态四元数，去除航向 yaw 后参与计算
（odom 航向跟随 base，只有相对重力的 roll/pitch 改变输出）——机器人摔倒
时躯干倾斜接近 ±90°，``/head_pose`` 的位置与姿态随之大幅变化，而不是
保持在站立值附近（见 ``head_pose.py`` 的说明）。

输入
----
- ``/robot/control/hardware_states``（``robot_interfaces/JointStates``，
  SensorDataQoS）：``head_pitch``、``head_yaw`` 实际关节角；
- ``/robot/imu/nav_all``（``yesense_interface/NavAll``，depth 10）：基座
  姿态四元数 ``quat(q0=w, q1=x, q2=y, q3=z)``（真机 yesense 与仿真同语义）。

每收到一帧关节状态即计算并发布一次，发布频率与状态流一致（真机/仿真均
为 200 Hz）。未收到关节状态前不发布；未收到 IMU 前按直立（单位倾斜）计
算，IMU 缺席只降级不阻塞。
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import JointStates

try:
    from yesense_interface.msg import NavAll
except ImportError:  # 缺少 IMU 接口包时仍可启动，仅失去基座姿态修正。
    NavAll = None

from robot_external.head_pose import (
    HeadCameraChain,
    compute_head_camera_pose,
    yaw_stripped_orientation,
)

# 与 robot_control / 仿真节点的 /robot/control/hardware_states 发布端 QoS 对齐
# （SensorDataQoS：BEST_EFFORT + KEEP_LAST(1)）。
_STATES_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

# /robot/imu/nav_all 发布端（yesense 驱动与仿真）均用默认 QoS：RELIABLE +
# KEEP_LAST(10)，与 robot_rl_manager 的订阅一致。
_IMU_QOS = QoSProfile(depth=10)


class HeadPoseNode(Node):
    """把头部关节角与基座倾斜换算成相机位姿并发布 ``/head_pose``。"""

    def __init__(self) -> None:
        super().__init__("head_pose_node")

        self._states_topic: str = str(
            self.declare_parameter("states_topic", "/robot/control/hardware_states").value
        )
        self._pose_topic: str = str(self.declare_parameter("pose_topic", "/head_pose").value)
        self._imu_topic: str = str(
            self.declare_parameter("imu_topic", "/robot/imu/nav_all").value
        )
        self._use_base_attitude: bool = bool(
            self.declare_parameter("use_base_attitude", True).value
        )
        self._head_pitch_joint: str = str(
            self.declare_parameter("head_pitch_joint", "head_pitch").value
        )
        self._head_yaw_joint: str = str(
            self.declare_parameter("head_yaw_joint", "head_yaw").value
        )

        # 安装链几何常数：默认值取自 MuJoCo XML（Z01_std_football_player），
        # 真机装配或其它机型可通过 YAML 覆盖。
        self._chain = HeadCameraChain(
            base_height=float(self.declare_parameter("base_height", 0.83).value),
            head_pitch_offset=self._vector_parameter(
                "head_pitch_offset", (0.00155, 0.0, 0.3155)
            ),
            head_yaw_offset=self._vector_parameter("head_yaw_offset", (0.0, 0.0, 0.0795)),
            camera_offset=self._vector_parameter("camera_offset", (0.069, 0.01, 0.068)),
        )

        # 最新的头部关节角（rad）；未收到状态前不发布。
        self._head_pitch = 0.0
        self._head_yaw = 0.0
        self._states_received = False
        self._missing_joint_warned = False

        # 最新基座倾斜（去 yaw 四元数）；未收到 IMU 前为单位四元数（直立）。
        self._base_tilt: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
        self._imu_received = False
        self._imu_invalid_warned = False

        # 复用同一个 Pose 对象，避免 200 Hz 回调上反复分配内存。
        self._pose_msg = Pose()

        self._pose_pub = self.create_publisher(Pose, self._pose_topic, 10)
        self._states_sub = self.create_subscription(
            JointStates,
            self._states_topic,
            self._handle_hardware_states,
            _STATES_QOS,
        )
        self._imu_sub = None
        if NavAll is not None and self._use_base_attitude:
            self._imu_sub = self.create_subscription(
                NavAll, self._imu_topic, self._handle_nav_all, _IMU_QOS
            )
        elif self._use_base_attitude:
            self.get_logger().warn(
                "yesense_interface not available; %s falls back to upright base attitude"
                % self._imu_topic
            )

        self.get_logger().info(
            "head_pose node ready: %s + %s -> %s, base_attitude=%s, base_height=%.4f m, "
            "head_pitch_offset=%s, head_yaw_offset=%s, camera_offset=%s"
            % (
                self._states_topic,
                self._imu_topic if self._imu_sub is not None else "(disabled)",
                self._pose_topic,
                self._use_base_attitude,
                self._chain.base_height,
                tuple(round(v, 5) for v in self._chain.head_pitch_offset),
                tuple(round(v, 5) for v in self._chain.head_yaw_offset),
                tuple(round(v, 5) for v in self._chain.camera_offset),
            )
        )

    def _vector_parameter(self, name: str, default: Sequence[float]) -> Tuple[float, float, float]:
        """读取长度必须为 3 的浮点数组参数，非法配置直接报错拒绝启动。"""
        value = self.declare_parameter(name, list(default)).value
        if value is None or len(value) != 3:
            raise ValueError(f"parameter {name} must be a 3-element array, got {value!r}")
        return (float(value[0]), float(value[1]), float(value[2]))

    def _handle_nav_all(self, msg) -> None:
        """缓存 IMU 基座姿态：去 yaw 后作为 ``base_tilt`` 参与下次换算。

        NavAll.quat 顺序为 ``(q0=w, q1=x, q2=y, q3=z)``（真机 yesense 与
        仿真 SimNodeBase FillImu 同语义）。零四元数视为无效数据，保持上
        一次倾斜并只告警一次，避免 200 Hz 刷屏。
        """
        quat = (float(msg.quat.q1), float(msg.quat.q2), float(msg.quat.q3), float(msg.quat.q0))
        if math.sqrt(sum(component * component for component in quat)) < 1e-6:
            if not self._imu_invalid_warned:
                self._imu_invalid_warned = True
                self.get_logger().warn("invalid zero quaternion on %s; keeping last base tilt" % self._imu_topic)
            return
        self._base_tilt = yaw_stripped_orientation(quat)
        self._imu_received = True
        self._imu_invalid_warned = False

    def _handle_hardware_states(self, msg: JointStates) -> None:
        """缓存头部关节角，并立即换算发布一帧相机位姿。"""
        self._update_head_joints(msg)
        if not self._states_received:
            return
        pose = compute_head_camera_pose(self._head_pitch, self._head_yaw, self._chain, self._base_tilt)

        self._pose_msg.position.x = pose.position[0]
        self._pose_msg.position.y = pose.position[1]
        self._pose_msg.position.z = pose.position[2]
        self._pose_msg.orientation.x = pose.orientation[0]
        self._pose_msg.orientation.y = pose.orientation[1]
        self._pose_msg.orientation.z = pose.orientation[2]
        self._pose_msg.orientation.w = pose.orientation[3]
        self._pose_pub.publish(self._pose_msg)

    def _update_head_joints(self, msg: JointStates) -> None:
        """从状态帧中取出头部两个关节；缺失时保持上一次的角度。"""
        found_pitch: Optional[float] = None
        found_yaw: Optional[float] = None
        for state in msg.states:
            if state.joint_name == self._head_pitch_joint:
                found_pitch = float(state.position)
            elif state.joint_name == self._head_yaw_joint:
                found_yaw = float(state.position)

        if found_pitch is not None:
            self._head_pitch = found_pitch
        if found_yaw is not None:
            self._head_yaw = found_yaw

        if found_pitch is None or found_yaw is None:
            # 仅在首次缺失时告警一次，避免 200 Hz 刷屏。
            if not self._missing_joint_warned:
                self._missing_joint_warned = True
                self.get_logger().warn(
                    "%s missing joint(s): %s"
                    % (
                        self._states_topic,
                        ", ".join(
                            name
                            for name, value in (
                                (self._head_pitch_joint, found_pitch),
                                (self._head_yaw_joint, found_yaw),
                            )
                            if value is None
                        ),
                    )
                )
        else:
            self._missing_joint_warned = False

        self._states_received = True


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = HeadPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
