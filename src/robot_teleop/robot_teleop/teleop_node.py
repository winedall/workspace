"""把遥控输入归一化为 RobotIntent 的 rclpy 节点。"""

from __future__ import annotations

from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import RobotIntent, RobotState
from sensor_msgs.msg import Joy

from robot_teleop.intent_builder import IntentBuilder, IntentSpec
from robot_teleop.teleop_joy import JoyTeleopConfig, TeleopJoyMapper


class RobotTeleopNode(Node):
    """将 Joy 或未来其他外部输入转换为 RobotIntent。

    本节点只发布 input/intent，不发布 JointCommands，也不访问真实硬件。
    """

    def __init__(self) -> None:
        super().__init__("robot_teleop_node")

        # 基础参数：来源名、发布频率、运动速度上限、意图超时和 Joy 使能开关。
        self._source_name = str(self.declare_parameter("source_name", "teleop").value)
        self._publish_frequency_hz = float(self.declare_parameter("publish_frequency_hz", 20.0).value)
        self._max_linear_x_mps = float(self.declare_parameter("max_linear_x_mps", 1.0).value)
        self._max_linear_y_mps = float(self.declare_parameter("max_linear_y_mps", 0.5).value)
        self._max_linear_x_high_mps = float(self.declare_parameter("max_linear_x_high_mps", 3.0).value)
        self._max_linear_y_high_mps = float(self.declare_parameter("max_linear_y_high_mps", 1.0).value)
        self._max_angular_z_radps = float(self.declare_parameter("max_angular_z_radps", 1.5).value)
        self._enable_joy = bool(self.declare_parameter("enable_joy", True).value)
        self._joy_deadzone = float(self.declare_parameter("joy_deadzone", 0.08).value)
        self._dpad_threshold = float(self.declare_parameter("dpad_threshold", 0.5).value)
        self._robot_type = str(self.declare_parameter("robot_type", "humanoid").value)
        self._joy_timeout_sec = float(self.declare_parameter("joy_timeout_sec", 0.3).value)

        # IntentBuilder 统一填充 RobotIntent 公共字段。
        self._builder = IntentBuilder(
            source_name=self._source_name,
            priority=0,
        )
        # Joy 解析器只输出 IntentSpec，不直接发布 ROS topic。
        self._joy_mapper = TeleopJoyMapper(
            JoyTeleopConfig(
                max_linear_x_mps=self._max_linear_x_mps,
                max_linear_y_mps=self._max_linear_y_mps,
                max_linear_x_high_mps=self._max_linear_x_high_mps,
                max_linear_y_high_mps=self._max_linear_y_high_mps,
                max_angular_z_radps=self._max_angular_z_radps,
                deadzone=self._joy_deadzone,
                dpad_threshold=self._dpad_threshold,
            ),
            robot_type=self._robot_type,
        )
        # 一次性事件队列：status/model/config 只在按键事件发生时发布一次。
        self._pending_events: deque[IntentSpec] = deque()
        # 连续运动缓存：motion + walk/stop 由 timer 按 publish_frequency_hz 周期发布。
        self._latest_motion: Optional[IntentSpec] = None

        # 唯一输出 topic：所有外部输入归一化为 RobotIntent。
        self._intent_pub = self.create_publisher(RobotIntent, "input/intent", 10)
        self._joy_sub = None
        # Joy 订阅默认关闭，避免未接手柄时产生无意义输入链路。
        if self._enable_joy:
            self._joy_sub = self.create_subscription(Joy, "joy", self._handle_joy, 10)

        self._last_joy_stamp = self.get_clock().now()
        self._ever_received_joy = False

        # 系统状态缓存，用于判断超时机制是否应当激活。
        self._system_state = RobotState.STATE_BOOTING
        self._state_sub = self.create_subscription(
            RobotState, "state", self._handle_state, 10)

        # 定时发布用于统一节流：事件优先，若无事件则发布最新 motion。
        period_sec = 1.0 / max(self._publish_frequency_hz, 1.0)
        self._timer = self.create_timer(period_sec, self._handle_timer)

    def _handle_state(self, msg: RobotState) -> None:
        """缓存系统状态，供超时机制判断是否应当激活。"""
        self._system_state = msg.state

    def _handle_joy(self, msg: Joy) -> None:
        """处理一帧 Joy 输入。"""
        self._ever_received_joy = True
        self._last_joy_stamp = self.get_clock().now()
        spec = self._joy_mapper.map_joy(msg)
        if spec is None:
            return
        if spec.intent_type == "motion":
            # motion 是连续控制，只保留最新一帧，避免队列堆积造成延迟。
            self._latest_motion = spec
        else:
            if spec.intent_type == "status" and spec.command != "walk":
                # ready/damp/reinit 会关闭 motion 授权，同时清掉旧的连续运动缓存。
                self._latest_motion = None
            # status/model/config 是边沿事件，进入队列后由 timer 发布一次。
            self._pending_events.append(spec)

    def _handle_timer(self) -> None:
        """按固定频率发布 RobotIntent，并在遥控器信号丢失时发布零速度停止指令。"""
        if self._enable_joy and self._ever_received_joy:
            now = self.get_clock().now()
            elapsed = (now - self._last_joy_stamp).nanoseconds / 1e9
            if elapsed > self._joy_timeout_sec and self._system_state == RobotState.STATE_RL_ACTIVE:
                self._latest_motion = IntentSpec(intent_type="motion", command="walk")

        spec = None
        # 一次性事件优先于连续 motion，避免按键命令被摇杆 stop 覆盖。
        if self._pending_events:
            spec = self._pending_events.popleft()
        elif self._latest_motion is not None:
            spec = self._latest_motion
        if spec is None:
            return

        intent = self._builder.build(spec)
        # 使用 ROS clock 填写时间戳，支持 use_sim_time。
        intent.header.stamp = self.get_clock().now().to_msg()
        self._intent_pub.publish(intent)


def main(args: Optional[list[str]] = None) -> None:
    """运行 robot_teleop_node。"""
    rclpy.init(args=args)
    node = RobotTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
