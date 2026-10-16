import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from robot_interfaces.msg import HeadControl, JointCommand, JointCommands, JointStates


# 头部关节弧度限位（与 robot_joints.yaml 一致）。
_HEAD_YAW_MIN_RAD = -3.14
_HEAD_YAW_MAX_RAD = 3.14
_HEAD_PITCH_MIN_RAD = -3.14
_HEAD_PITCH_MAX_RAD = 3.14

class RobotExternalNode(Node):
    """外部控制接口节点。

    职责：
      - 订阅 external/head_control 接收头部控制指令（角度制）。
      - 订阅 control/hardware_states 缓存全量关节位置。
      - 将头部指令转换为弧度，做相对/绝对运算和限幅，然后插值平滑发布。
      - 发布到 external/desired_commands 供 router 合并。
    """

    def __init__(self):
        super().__init__("robot_external_node")

        # 插值步数参数（200Hz 下默认 100 帧 ≈ 0.5s）。
        self.declare_parameter("interpolation_steps", 100)
        self._interpolation_steps: int = max(
            1,
            int(self.get_parameter("interpolation_steps").value),
        )

        # 仅缓存头部关节位置 {关节名: 位置弧度值}。
        self._joint_states: dict[str, float] = {}

        # 插值状态。
        self._interpolating = False
        self._interp_step = 0
        self._interp_steps = 0
        self._interp_start_yaw = 0.0
        self._interp_start_pitch = 0.0
        self._interp_target_yaw = 0.0
        self._interp_target_pitch = 0.0

        # 订阅头部控制指令（角度制）。
        self._head_control_sub = self.create_subscription(
            HeadControl,
            "external/head_control",
            self._handle_head_control,
            10,
        )

        # 订阅硬件状态，只缓存头部关节位置。
        self._joint_states_sub = self.create_subscription(
            JointStates,
            "control/hardware_states",
            self._handle_hardware_states,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            ),
        )

        # 发布外部关节命令（弧度制）。
        self._external_pub = self.create_publisher(
            JointCommands,
            "external/desired_commands",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            ),
        )

        # 插值定时器（5ms = 200Hz，与控制链频率对齐）。默认取消，
        # 仅在收到头部控制命令后 reset，插值完成后再次 cancel。
        self._interp_timer = self.create_timer(0.005, self._publish_interpolated)
        self._interp_timer.cancel()

    def _handle_hardware_states(self, msg: JointStates) -> None:
        """缓存头部关节位置（弧度）。"""
        for state in msg.states:
            if state.joint_name in ("head_yaw", "head_pitch"):
                self._joint_states[state.joint_name] = state.position

    def _handle_head_control(self, msg: HeadControl) -> None:
        """处理头部控制指令。

        1. 将角度制转为弧度制。
        2. 相对/绝对转换。
        3. 限幅到关节活动范围。
        4. 启动插值。
        """
        # 角度 → 弧度转换。
        target_yaw_rad = math.radians(msg.yaw)
        target_pitch_rad = math.radians(msg.pitch)

        # 获取当前头部位置（弧度）。
        current_yaw = self._joint_states.get("head_yaw", 0.0)
        current_pitch = self._joint_states.get("head_pitch", 0.0)

        # 相对/绝对转换。
        if msg.is_relative:
            target_yaw_rad = current_yaw + target_yaw_rad
            target_pitch_rad = current_pitch + target_pitch_rad

        # 限幅到关节活动范围。
        target_yaw_rad = max(_HEAD_YAW_MIN_RAD, min(_HEAD_YAW_MAX_RAD, target_yaw_rad))
        target_pitch_rad = max(_HEAD_PITCH_MIN_RAD, min(_HEAD_PITCH_MAX_RAD, target_pitch_rad))

        # 保存插值起点和终点。
        self._interp_start_yaw = current_yaw
        self._interp_start_pitch = current_pitch
        self._interp_target_yaw = target_yaw_rad
        self._interp_target_pitch = target_pitch_rad
        self._interp_step = 0
        self._interp_steps = self._interpolation_steps
        self._interpolating = True
        self._interp_timer.reset()

        self.get_logger().info(
            f"head control: yaw={msg.yaw}° -> {math.degrees(target_yaw_rad):.1f}°, "
            f"pitch={msg.pitch}° -> {math.degrees(target_pitch_rad):.1f}°, "
            f"relative={msg.is_relative}"
        )

    def _publish_interpolated(self) -> None:
        """定时器回调：按插值进度发布 JointCommands。"""
        if not self._interpolating:
            return

        if self._interp_step >= self._interp_steps:
            self._interpolating = False
            self._interp_timer.cancel()
            return

        # 线性插值。
        t = (self._interp_step + 1) / self._interp_steps
        yaw = self._interp_start_yaw + (self._interp_target_yaw - self._interp_start_yaw) * t
        pitch = self._interp_start_pitch + (self._interp_target_pitch - self._interp_start_pitch) * t

        # 构造 JointCommands 消息。
        msg = JointCommands()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "external"
        msg.stage = "desired"

        cmd_yaw = JointCommand()
        cmd_yaw.joint_name = "head_yaw"
        cmd_yaw.position = yaw
        cmd_yaw.kp = 20.0
        cmd_yaw.kd = 2.0
        cmd_yaw.enable = True
        msg.commands.append(cmd_yaw)

        cmd_pitch = JointCommand()
        cmd_pitch.joint_name = "head_pitch"
        cmd_pitch.position = pitch
        cmd_pitch.kp = 20.0
        cmd_pitch.kd = 2.0
        cmd_pitch.enable = True
        msg.commands.append(cmd_pitch)

        self._external_pub.publish(msg)
        self._interp_step += 1
        if self._interp_step >= self._interp_steps:
            self._interpolating = False
            self._interp_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = RobotExternalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
