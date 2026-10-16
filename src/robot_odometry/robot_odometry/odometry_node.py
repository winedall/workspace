"""本体感觉里程计 ROS 2 节点。

数据流：

    /robot/control/hardware_states ─┐
    /robot/imu/nav_all            ─┤→ adapter 拼观测 → 历史窗口 → ONNX → Δ(dx,dy)
                                   │                                        │
             50Hz 定时器（本节点自驱）┘                                        ▼
                                                       积分 p(t+1)=p(t)+Δ
                                                                            │
                                        /robot/odometry/odom (nav_msgs/Odometry)
                                        /robot/odometry/delta (Vector3Stamped)
                                        CSV 对比记录（可选）

节点本身不关心观测由哪些字段拼成：那由 adapter 决定（见 ``adapters/``），
用哪个模型由注册表 ``config/odometry_models.yaml`` 决定，两者都通过
``model_name`` 参数选择。

触发方式：本节点**自驱 50 Hz 定时器**，频率取自模型的 ``control_frequency_hz``
（由注册表加载时校验，必须与模型训练步长一致）。模型对输入的时间语义有硬要求
——历史窗口 50 帧的物理时长等于 ``50 / 频率`` 秒，换频率就换了窗口含义，
因此这个频率不是可以随手调的参数。

本节点因此**不依赖任何其它节点的话题**来驱动：joint states 与 IMU 只提供数据，
谁在什么频率上发布都不影响推理节奏。控制流中断超过 ``reset_gap_sec`` 时自动
重置模型与 adapter 状态，避免跨越中断把历史窗口拼错。

行走门控：只有 ``RobotState == STATE_RL_ACTIVE`` 时才做估计并积分；其它状态
下一律输出 0（位置与增量均为 0），并按 ``idle_publish_rate_hz`` 周期维持零值
输出，保证下游看到的是一个明确的零而不是上一次的残留值。

仿真真值：仿真节点把 MuJoCo 的基座位置填进 ``NavAll.pos``
（longitude/latitude/altitude 即 x/y/z），本节点用它与估计值逐帧对比记录。
"""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from robot_interfaces.msg import JointStates, RobotState
from std_msgs.msg import Empty
from yesense_interface.msg import NavAll

from .adapters.base import OdometryContext
from .csv_recorder import CsvRecorder, resolve_output_dir
from .geometry import quaternion_to_yaw, wrap_angle
from .joint_order import ODOM_STEP_DT
from .model_registry import OdometryModelRegistry, load_adapter_class
from .online_odometry import OnlineOdometry
from .onnx_backend import OnnxOdometryBackend

DEFAULT_MODELS_CONFIG_NAME = "config/odometry_models.yaml"


def _default_models_config_path() -> str:
    """返回默认模型注册表路径。

    优先取安装目录（share/robot_odometry/config/），包尚未安装时回退到源码树，
    这样未 colcon build 也能跑单测。
    """
    try:
        share = Path(get_package_share_directory("robot_odometry"))
        return str(share / DEFAULT_MODELS_CONFIG_NAME)
    except Exception:  # noqa: BLE001 - 未安装时 ament 抛 PackageNotFoundError
        return str(
            Path(__file__).resolve().parents[1] / DEFAULT_MODELS_CONFIG_NAME
        )


class RobotOdometryNode(Node):
    """订阅本体感觉与策略动作，发布增量与累计里程计位姿。"""

    def __init__(self) -> None:
        super().__init__("robot_odometry_node")

        self._models_config = str(
            self.declare_parameter("models_config", "").value
            or _default_models_config_path()
        )
        self._requested_model_name = str(
            self.declare_parameter("model_name", "").value
        )
        self._intra_op_num_threads = int(
            self.declare_parameter("onnx_intra_op_num_threads", 1).value
        )
        self._inter_op_num_threads = int(
            self.declare_parameter("onnx_inter_op_num_threads", 1).value
        )

        self._joint_states_topic = str(
            self.declare_parameter(
                "joint_states_topic", "control/hardware_states"
            ).value
        )
        self._imu_topic = str(
            self.declare_parameter("imu_topic", "imu/nav_all").value
        )
        self._state_topic = str(
            self.declare_parameter("state_topic", "state").value
        )
        self._odom_topic = str(
            self.declare_parameter("odom_topic", "odometry/odom").value
        )
        self._delta_topic = str(
            self.declare_parameter("delta_topic", "odometry/delta").value
        )
        self._reset_topic = str(
            self.declare_parameter("reset_topic", "odometry/reset").value
        )
        self._odom_frame_id = str(
            self.declare_parameter("odom_frame_id", "odom").value
        )
        self._base_frame_id = str(
            self.declare_parameter("base_frame_id", "base_link").value
        )

        # 输入新鲜度阈值：关节状态 / IMU 超过该时长未更新则跳过本帧推理。
        self._input_timeout_sec = float(
            self.declare_parameter("input_timeout_sec", 0.1).value
        )
        # 控制流中断阈值：两次推理间隔超过该时长视为断流，重置模型状态。
        self._reset_gap_sec = float(
            self.declare_parameter("reset_gap_sec", 0.1).value
        )
        # 非行走状态下维持零值输出的频率。
        self._idle_publish_rate_hz = float(
            self.declare_parameter("idle_publish_rate_hz", 10.0).value
        )
        # 单步位移合理性上限（米）。模型在越界输入下自回归会放大，
        # 超过该阈值视为发散：丢弃本步、重置模型并重新从复制窗口起步。
        self._max_step_delta_m = float(
            self.declare_parameter("max_step_delta_m", 0.2).value
        )

        self._csv_enabled = bool(
            self.declare_parameter("csv_enabled", True).value
        )
        self._csv_dir = str(self.declare_parameter("csv_dir", "").value)

        # 模型注册表：由 model_name 选出 ONNX 图、每模型配置和输入 adapter。
        self._registry = OdometryModelRegistry(self._models_config)
        self._model_name = self._registry.resolve_model_name(
            self._requested_model_name
        )
        self._model_info = self._registry.get(self._model_name)

        adapter_class = load_adapter_class(self._model_info.adapter_class_path)
        self._adapter = adapter_class()

        # 配置声明的观测维度必须与 adapter 实际产出的一致，否则是配置与
        # adapter 版本错配（例如跨 schema 复制了配置文件）。
        if self._adapter.obs_dim != self._model_info.obs_dim:
            raise ValueError(
                f"模型 '{self._model_name}' 配置的 obs_dim="
                f"{self._model_info.obs_dim} 与 adapter "
                f"{type(self._adapter).__name__} 实际产出的 "
                f"{self._adapter.obs_dim} 不一致"
            )

        self._backend = OnnxOdometryBackend(
            self._model_info.model_path,
            obs_dim=self._model_info.obs_dim,
            history_len=self._model_info.history_len,
            intra_op_num_threads=self._intra_op_num_threads,
            inter_op_num_threads=self._inter_op_num_threads,
        )
        self._odometry = OnlineOdometry(
            self._backend,
            history_len=self._model_info.history_len,
            obs_dim=self._model_info.obs_dim,
            odom_dim=self._model_info.odom_dim,
        )

        self.get_logger().info(
            f"里程计模型已加载: {self._model_name} "
            f"(obs_dim={self._model_info.obs_dim}, "
            f"history_len={self._model_info.history_len}, "
            f"{self._model_info.control_frequency_hz:g}Hz, "
            f"adapter={type(self._adapter).__name__})\n"
            f"  注册表: {self._models_config}\n"
            f"  模型图: {self._model_info.model_path}\n"
            f"  观测布局: {self._model_info.obs_layout}"
        )

        # 最新输入缓存。
        self._joint_states: Optional[Dict[str, Tuple[float, float]]] = None
        self._joint_states_time_sec: Optional[float] = None
        self._base_quat: Optional[Tuple[float, float, float, float]] = None
        self._base_ang_vel: Optional[Tuple[float, float, float]] = None
        self._base_acc: Optional[Tuple[float, float, float]] = None
        self._imu_time_sec: Optional[float] = None
        # 仿真真值：NavAll.pos 在仿真里被填成 MuJoCo 基座位置 (x, y, z)。
        self._gt_abs: Optional[Tuple[float, float, float]] = None

        # 行走门控与里程计累计状态。
        self._walking = False
        self._position_xy = np.zeros(2, dtype=np.float64)
        self._yaw_origin: Optional[float] = None
        self._last_step_time_sec: Optional[float] = None
        self._steps = 0
        self._skipped = 0
        self._diverged = 0
        self._last_warn_time_sec = 0.0

        # 真值对比状态。
        self._gt_origin: Optional[np.ndarray] = None
        self._gt_previous: Optional[np.ndarray] = None
        self._node_start_time_sec: Optional[float] = None
        self._recorder: Optional[CsvRecorder] = None
        self._init_recorder()

        # 高频控制输入使用 KeepLast + depth=1 + BEST_EFFORT，与上游发布端一致。
        _high_freq_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self._joint_states_sub = self.create_subscription(
            JointStates,
            self._joint_states_topic,
            self._handle_joint_states,
            _high_freq_qos,
        )
        self._imu_sub = self.create_subscription(
            NavAll, self._imu_topic, self._handle_nav_all, 10
        )
        self._state_sub = self.create_subscription(
            RobotState, self._state_topic, self._handle_robot_state, 10
        )
        self._reset_sub = self.create_subscription(
            Empty, self._reset_topic, self._handle_reset, 10
        )

        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 10)
        self._delta_pub = self.create_publisher(
            Vector3Stamped, self._delta_topic, 10
        )

        # 推理定时器：频率取自模型配置，是节点唯一的推理触发源。
        # 不依赖任何上游话题来驱动，因此对上游发布频率不敏感。
        inference_period = 1.0 / max(self._model_info.control_frequency_hz, 1e-6)
        self._inference_timer = self.create_timer(
            inference_period, self._handle_inference_timer
        )

        # 非行走状态下的零值心跳，保证下游看到的是明确的 0 而非残留值。
        idle_period = 1.0 / max(self._idle_publish_rate_hz, 0.1)
        self._idle_timer = self.create_timer(idle_period, self._handle_idle_timer)

    # ------------------------------------------------------------------
    # 输入回调
    # ------------------------------------------------------------------

    def _handle_joint_states(self, msg: JointStates) -> None:
        """缓存按关节名索引的硬件反馈。"""
        states: Dict[str, Tuple[float, float]] = {}
        for state in msg.states:
            states[state.joint_name] = (state.position, state.velocity)
        self._joint_states = states
        self._joint_states_time_sec = self._now_sec()

    def _handle_nav_all(self, msg: NavAll) -> None:
        """缓存 body 系角速度、姿态四元数与仿真真值位置。

        NavAll 的 gyro 单位为 deg/s，此处统一转成 rad/s，与训练侧
        ``imu_ang_vel``（rad/s）保持一致。

        ``pos.longitude/latitude/altitude`` 在仿真里被填成基座世界系位置
        (x, y, z)，真机上是经纬高，因此只用于记录，不参与推理。
        """
        self._base_quat = (
            float(msg.quat.q0),
            float(msg.quat.q1),
            float(msg.quat.q2),
            float(msg.quat.q3),
        )
        self._base_ang_vel = (
            math.radians(float(msg.gyro.x)),
            math.radians(float(msg.gyro.y)),
            math.radians(float(msg.gyro.z)),
        )
        # NavAll.acc 是 body 系线加速度（m/s^2），只有 91 维模型会用到。
        # 仿真与真机的 yesense 驱动都会填该字段。
        self._base_acc = (
            float(msg.acc.x),
            float(msg.acc.y),
            float(msg.acc.z),
        )
        self._gt_abs = (
            float(msg.pos.longitude),
            float(msg.pos.latitude),
            float(msg.pos.altitude),
        )
        self._imu_time_sec = self._now_sec()

    def _handle_robot_state(self, msg: RobotState) -> None:
        """跟随系统状态切换行走门控。"""
        walking = msg.state == RobotState.STATE_RL_ACTIVE
        if walking == self._walking:
            return
        self._walking = walking
        if walking:
            self._start_run()
        else:
            self._stop_run()

    def _handle_reset(self, _msg: Empty) -> None:
        """手动重置：清空模型状态并把累计位姿归零。"""
        self._reset_state(reason="收到 /reset 请求")
        self._publish_odometry(np.zeros(2, dtype=np.float32), self._stamp())

    # ------------------------------------------------------------------
    # 推理主流程
    # ------------------------------------------------------------------

    def _handle_inference_timer(self) -> None:
        """按模型标称频率跑一次里程计推理并发布结果。"""
        if not self._walking:
            # 非行走状态不做估计，零值由空闲心跳维持。
            return

        now_sec = self._now_sec()

        if not self._inputs_ready(now_sec):
            return

        assert self._joint_states is not None
        assert self._base_quat is not None
        assert self._base_ang_vel is not None

        # 控制流中断判定：中断后历史窗口已不连续，必须重置模型状态。
        if (
            self._last_step_time_sec is not None
            and now_sec - self._last_step_time_sec > self._reset_gap_sec
        ):
            gap_ms = (now_sec - self._last_step_time_sec) * 1000.0
            self._reset_state(reason=f"控制流中断 {gap_ms:.0f}ms")

        try:
            observation = self._adapter.build_observation(self._make_context(now_sec))
            delta = self._odometry.step(observation)
        except (KeyError, ValueError) as exc:
            self._warn_throttled(f"里程计输入无效: {exc}")
            return

        step_norm = float(np.linalg.norm(delta))
        if step_norm > self._max_step_delta_m:
            # 越界输入会让自回归发散。丢弃本步并重置，避免污染后续窗口。
            self._diverged += 1
            self._warn_throttled(
                f"单步位移 {step_norm:.4f}m 超过上限 "
                f"{self._max_step_delta_m:.3f}m（累计 {self._diverged} 次），"
                "判定发散并重置里程计状态。"
            )
            self._reset_state(reason="单步位移越界")
            return

        self._position_xy += delta.astype(np.float64)
        self._last_step_time_sec = now_sec
        self._steps += 1

        self._publish_odometry(delta, self._stamp())
        self._record_row(delta, now_sec)

    def _make_context(self, now_sec: float) -> OdometryContext:
        """把当前缓存打包成 adapter 需要的输入快照。"""
        return OdometryContext(
            joint_states=self._joint_states,
            base_quat=self._base_quat,
            base_ang_vel=self._base_ang_vel,
            base_acc=self._base_acc,
            now_sec=now_sec,
        )

    def _inputs_ready(self, now_sec: float) -> bool:
        """检查关节状态、IMU 与模型关节是否齐备且新鲜。"""
        if self._joint_states is None or self._joint_states_time_sec is None:
            self._warn_throttled("尚未收到硬件关节状态，跳过里程计推理。")
            return False
        if self._base_quat is None or self._base_ang_vel is None or self._imu_time_sec is None:
            self._warn_throttled("尚未收到 IMU 数据，跳过里程计推理。")
            return False

        # 姿态模长异常说明 AHRS 尚未收敛或数据无效：训练数据都是单位四元数，
        # 直接送入会让输入偏离分布，因此先归一化，偏差过大则跳过本帧。
        norm = math.sqrt(sum(value * value for value in self._base_quat))
        if not 0.5 < norm < 1.5:
            self._warn_throttled(
                f"IMU 姿态四元数模长异常 ({norm:.3f})，跳过里程计推理。"
            )
            return False
        if abs(norm - 1.0) > 1e-3:
            self._base_quat = tuple(value / norm for value in self._base_quat)

        if now_sec - self._joint_states_time_sec > self._input_timeout_sec:
            self._skipped += 1
            self._warn_throttled(
                f"硬件关节状态超时（累计跳过 {self._skipped} 帧），"
                "跳过里程计推理。"
            )
            return False
        if now_sec - self._imu_time_sec > self._input_timeout_sec:
            self._skipped += 1
            self._warn_throttled(
                f"IMU 数据超时（累计跳过 {self._skipped} 帧），"
                "跳过里程计推理。"
            )
            return False
        if self._yaw_origin is None:
            self._yaw_origin = quaternion_to_yaw(self._base_quat)

        # 其余输入是否齐备由 adapter 声明，节点不写死任何 schema 细节。
        missing = self._adapter.missing_inputs(self._make_context(now_sec))
        if missing:
            self._warn_throttled(
                f"缺少模型所需输入 {sorted(missing)}，跳过里程计推理。"
            )
            return False
        return True

    # ------------------------------------------------------------------
    # 行走门控与运行周期
    # ------------------------------------------------------------------

    def _start_run(self) -> None:
        """进入行走状态：重置模型与原点，开始新一轮估计。"""
        self._reset_state(reason="进入行走状态")
        self.get_logger().info("进入 RL_ACTIVE，里程计开始估计。")

    def _stop_run(self) -> None:
        """离开行走状态：停估计、清状态，并输出一次零值。"""
        # 先取本轮结果再重置，否则日志里只剩 0。
        steps = self._steps
        distance = float(np.linalg.norm(self._position_xy))
        self._reset_state(reason="离开行走状态")
        self.get_logger().info(
            f"离开 RL_ACTIVE，里程计输出归零（本轮累计 {steps} 帧、"
            f"{distance:.3f} m）。"
        )
        self._publish_odometry(np.zeros(2, dtype=np.float32), self._stamp())

    def _handle_idle_timer(self) -> None:
        """非行走状态下维持零值输出。"""
        if self._walking:
            return
        self._publish_odometry(np.zeros(2, dtype=np.float32), self._stamp())

    def _reset_state(self, reason: str) -> None:
        """重置自回归状态、adapter 帧间状态与累计位姿。"""
        self._odometry.reset()
        # adapter 持有 joint_pos_previous：不一起清掉的话，新一轮的首帧会
        # 沿用上一轮的关节位置，与训练侧的「首帧自引用」约定不符。
        self._adapter.reset()
        self._last_step_time_sec = None
        self._position_xy = np.zeros(2, dtype=np.float64)
        self._yaw_origin = None
        self._gt_origin = None
        self._gt_previous = None
        self._steps = 0
        self.get_logger().info(f"里程计状态已重置（{reason}）。")

    def _current_yaw_relative(self) -> float:
        """返回相对本次运行起点的偏航角。"""
        assert self._base_quat is not None
        yaw = quaternion_to_yaw(self._base_quat)
        origin = self._yaw_origin if self._yaw_origin is not None else 0.0
        return wrap_angle(yaw - origin)

    # ------------------------------------------------------------------
    # 发布与记录
    # ------------------------------------------------------------------

    def _publish_odometry(self, delta: np.ndarray, stamp) -> None:
        """同时发布单步增量与累计位姿。"""
        self._publish_delta(delta, stamp)
        self._publish_odom(delta, stamp, self._safe_yaw_relative())

    def _safe_yaw_relative(self) -> float:
        """在 IMU 尚未就绪时返回 0，避免发布时断言失败。"""
        if self._base_quat is None:
            return 0.0
        return self._current_yaw_relative()

    def _publish_delta(self, delta: np.ndarray, stamp) -> None:
        """发布本步世界系位移增量，单位米。"""
        message = Vector3Stamped()
        message.header.stamp = stamp
        message.header.frame_id = self._odom_frame_id
        message.vector.x = float(delta[0])
        message.vector.y = float(delta[1])
        message.vector.z = 0.0
        self._delta_pub.publish(message)

    def _publish_odom(self, delta: np.ndarray, stamp, yaw: float) -> None:
        """发布累计里程计位姿与速度。

        位姿为 2D 积分结果（z 恒为 0，仅保留偏航），姿态取自 IMU 相对本次
        运行起点的偏航角——模型本身不预测姿态。

        covariance 未做估计，保持全 0；如需下游按方差加权，应在此填入标定值。
        """
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self._odom_frame_id
        message.child_frame_id = self._base_frame_id

        message.pose.pose.position.x = float(self._position_xy[0])
        message.pose.pose.position.y = float(self._position_xy[1])
        message.pose.pose.position.z = 0.0
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # 位移增量按模型标称步长换算成世界系线速度。
        message.twist.twist.linear.x = float(delta[0]) / ODOM_STEP_DT
        message.twist.twist.linear.y = float(delta[1]) / ODOM_STEP_DT
        if self._base_ang_vel is not None:
            message.twist.twist.angular.z = float(self._base_ang_vel[2])

        self._odom_pub.publish(message)

    def _init_recorder(self) -> None:
        """按配置打开 CSV 记录文件。"""
        if not self._csv_enabled:
            self.get_logger().info("里程计 CSV 记录已关闭（csv_enabled=false）。")
            return
        output_dir = resolve_output_dir(self._csv_dir)
        run_id = self._run_id()
        self._recorder = CsvRecorder(output_dir / f"odometry_{run_id}.csv")
        self.get_logger().info(f"里程计对比记录: {self._recorder.path}")

    @staticmethod
    def _run_id() -> str:
        """返回本次运行的标识：优先复用 run.sh 的 RUN_ID。"""
        import os

        run_id = os.environ.get("ZL_ROBOT_RUN_ID", "")
        if run_id:
            return run_id
        return time.strftime("%Y%m%d_%H%M%S")

    def _record_row(self, delta: np.ndarray, now_sec: float) -> None:
        """写入一帧「真值 vs 估计」对比记录。

        真值相对列与估计值共用同一个原点（本次运行起点），可直接相减求误差。
        仿真真值缺失时整行跳过，避免写出误导性的零。
        """
        if self._recorder is None or self._gt_abs is None:
            return

        if self._node_start_time_sec is None:
            self._node_start_time_sec = now_sec

        gt_abs = np.asarray(self._gt_abs, dtype=np.float64)
        if self._gt_origin is None:
            self._gt_origin = gt_abs.copy()
            self._gt_previous = gt_abs.copy()

        gt_relative = gt_abs[:2] - self._gt_origin[:2]
        assert self._gt_previous is not None
        gt_delta = gt_abs[:2] - self._gt_previous[:2]
        self._gt_previous = gt_abs.copy()

        self._recorder.write_row(
            time_sec=now_sec - self._node_start_time_sec,
            step=self._steps,
            gt_xy=(float(gt_relative[0]), float(gt_relative[1])),
            gt_delta_xy=(float(gt_delta[0]), float(gt_delta[1])),
            est_xy=(float(self._position_xy[0]), float(self._position_xy[1])),
            est_delta_xy=(float(delta[0]), float(delta[1])),
            gt_abs_xyz=(
                float(gt_abs[0]),
                float(gt_abs[1]),
                float(gt_abs[2]),
            ),
        )

    # ------------------------------------------------------------------
    # 工具
    # ------------------------------------------------------------------

    def _stamp(self):
        """返回当前 ROS 时间戳消息。"""
        return self.get_clock().now().to_msg()

    def _now_sec(self) -> float:
        """返回当前 ROS 时钟的秒数。"""
        return self.get_clock().now().nanoseconds * 1e-9

    def _warn_throttled(self, text: str) -> None:
        """限制告警频率，避免超时/缺输入时刷屏。"""
        now_sec = time.monotonic()
        if now_sec - self._last_warn_time_sec < 2.0:
            return
        self._last_warn_time_sec = now_sec
        self.get_logger().warn(text)

    def destroy_node(self) -> bool:
        """关闭记录文件后再销毁节点。"""
        if self._recorder is not None:
            self._recorder.close()
            self._recorder = None
        return super().destroy_node()


def main(args=None) -> None:
    """节点入口。"""
    rclpy.init(args=args)
    node = RobotOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
