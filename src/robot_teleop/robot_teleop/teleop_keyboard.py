"""把键盘输入归一化为 RobotIntent 的 rclpy 节点。

使用 pynput 实现全局按键监听，不抢占终端，不产生乱码。

使用方法：
  ros2 run robot_teleop robot_keyboard_node
  或通过 ros2 launch 自动启动。

按键映射（与 F710 功能对应）：
  b  → status/ready         起立
  r  → status/walk          启用行走
  y  → status/damp          阻尼/坐下
  x  → status/reinit        重新初始化
  /  → model/finish_loop    结束循环
  1  → model/switch_model/t4_hello       打招呼
  2  → model/switch_model/t4_clap        鼓掌
  3  → model/switch_model/t4_squat_up    蹲起
  4  → model/switch_model/t4_squat_down  下蹲
  5  → model/switch_model/t4_heart       比心
  6  → model/switch_model/t4_salute      敬礼
  7  → model/switch_model/t4_handshake   握手
  8  → model/switch_model/t4_high_five   击掌
  9  → model/switch_model/t4_long_jump   长跳
  0  → model/switch_model/t4_standby     待机
  k  → model/switch_model/t4_kick        踢球
  m  → 切换足球模式（显示/隐藏球门与球）
  p  → 足球放置模式（方向键移动、回车确认）
  o  → 随机球速度
"""

from __future__ import annotations

import math
import random
from threading import Lock
from typing import Optional

import rclpy
from geometry_msgs.msg import Point, Vector3
from pynput import keyboard
from rclpy.node import Node
from robot_interfaces.msg import RobotIntent
from std_msgs.msg import Bool

from robot_teleop.intent_builder import IntentBuilder, IntentSpec


class RobotKeyboardNode(Node):
    """使用 pynput 全局监听键盘，转换为 RobotIntent 发布到 input/intent。

    按键事件在 pynput 回调线程中入队，ROS timer 在主线程中出队并发布，
    无需修改终端属性，不影响终端正常使用。
    """

    _KEY_MAP: dict[str, IntentSpec] = {
        "b": IntentSpec(intent_type="status", command="ready"),
        "r": IntentSpec(intent_type="status", command="walk"),
        "y": IntentSpec(intent_type="status", command="damp"),
        "x": IntentSpec(intent_type="status", command="reinit"),
        "/": IntentSpec(intent_type="model", command="finish_loop"),
        "1": IntentSpec(intent_type="model", command="switch_model", action="t4_hello"),
        "2": IntentSpec(intent_type="model", command="switch_model", action="t4_clap"),
        "3": IntentSpec(intent_type="model", command="switch_model", action="t4_squat_up"),
        "4": IntentSpec(intent_type="model", command="switch_model", action="t4_squat_down"),
        "5": IntentSpec(intent_type="model", command="switch_model", action="t4_heart"),
        "6": IntentSpec(intent_type="model", command="switch_model", action="t4_salute"),
        "7": IntentSpec(intent_type="model", command="switch_model", action="t4_handshake"),
        "8": IntentSpec(intent_type="model", command="switch_model", action="t4_high_five"),
        "9": IntentSpec(intent_type="model", command="switch_model", action="t4_long_jump"),
        "0": IntentSpec(intent_type="model", command="switch_model", action="t4_standby"),
        "k": IntentSpec(intent_type="model", command="switch_model", action="t4_kick"),
    }

    def __init__(self) -> None:
        super().__init__("robot_keyboard_node")

        self._source_name = str(
            self.declare_parameter("source_name", "keyboard").value
        )
        self._publish_frequency_hz = float(
            self.declare_parameter("publish_frequency_hz", 20.0).value
        )

        # 声明速度参数（若 launch 中未传入，则使用默认标准速度值）
        self._max_linear_x_mps = float(self.declare_parameter("max_linear_x_mps", 1.0).value)
        self._max_linear_y_mps = float(self.declare_parameter("max_linear_y_mps", 0.5).value)
        self._max_angular_z_radps = float(self.declare_parameter("max_angular_z_radps", 1.5).value)

        self._builder = IntentBuilder(
            source_name=self._source_name,
            priority=80,
        )
        self._intent_pub = self.create_publisher(
            RobotIntent, "input/intent", 10
        )

        # 足球控制：p 放置（含预览点）、o 随机球速、m 足球模式。
        self._ball_pos_pub = self.create_publisher(
            Point, "/robot/teleop/set_ball_position", 10
        )
        self._ball_vel_pub = self.create_publisher(
            Vector3, "/robot/teleop/set_ball_velocity", 10
        )
        self._ball_preview_pub = self.create_publisher(
            Point, "/robot/teleop/ball_preview", 10
        )
        self._soccer_mode_pub = self.create_publisher(
            Bool, "/robot/sim/soccer_mode", 10
        )

        self._pending: Optional[IntentSpec] = None
        self._pressed_keys = set()     # 当前被按下的运动键集合
        self._motion_enabled = False   # 是否启用了行走授权（按下 r 后为 True，按下 b/y/x 后为 False）
        self._ball_placement_mode = False
        self._ball_pos = Point(x=0.0, y=0.0, z=0.11)
        self._soccer_mode = False
        self._lock = Lock()

        # 启动 pynput 全局监听（后台线程，不阻塞）。
        self._listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        self._listener.daemon = True
        self._listener.start()

        self.get_logger().info("全局键盘监听已启动（WASD/QE 运动、/结束循环、k 踢球、m 足球模式、p 足球放置、o 随机球速）。")

        period_sec = 1.0 / max(self._publish_frequency_hz, 1.0)
        self._timer = self.create_timer(period_sec, self._handle_timer)

    def _on_press(self, key: keyboard.Key | keyboard.KeyCode | None) -> None:
        """pynput 按键按下回调（在监听线程中执行）。"""
        # 处理方向键/回车（仅足球放置模式生效）
        with self._lock:
            if key == keyboard.Key.up:
                if self._ball_placement_mode:
                    self._ball_pos.y += 0.1
                    self._ball_preview_pub.publish(self._ball_pos)
                    self.get_logger().info(f"ball pos: ({self._ball_pos.x:.2f}, {self._ball_pos.y:.2f})")
                return
            if key == keyboard.Key.down:
                if self._ball_placement_mode:
                    self._ball_pos.y -= 0.1
                    self._ball_preview_pub.publish(self._ball_pos)
                    self.get_logger().info(f"ball pos: ({self._ball_pos.x:.2f}, {self._ball_pos.y:.2f})")
                return
            if key == keyboard.Key.left:
                if self._ball_placement_mode:
                    self._ball_pos.x -= 0.1
                    self._ball_preview_pub.publish(self._ball_pos)
                    self.get_logger().info(f"ball pos: ({self._ball_pos.x:.2f}, {self._ball_pos.y:.2f})")
                return
            if key == keyboard.Key.right:
                if self._ball_placement_mode:
                    self._ball_pos.x += 0.1
                    self._ball_preview_pub.publish(self._ball_pos)
                    self.get_logger().info(f"ball pos: ({self._ball_pos.x:.2f}, {self._ball_pos.y:.2f})")
                return
            if key == keyboard.Key.enter:
                if self._ball_placement_mode:
                    self._ball_placement_mode = False
                    self._ball_pos_pub.publish(self._ball_pos)
                    self._ball_preview_pub.publish(Point(x=0.0, y=0.0, z=-1.0))
                    self.get_logger().info(f"ball placed at ({self._ball_pos.x:.2f}, {self._ball_pos.y:.2f})")
                return

        try:
            if hasattr(key, "char") and key.char is not None:
                k = key.char.lower()
            else:
                return
        except (AttributeError, ValueError):
            return

        with self._lock:
            # p 键切换足球放置模式
            if k == "p":
                self._ball_placement_mode = not self._ball_placement_mode
                if self._ball_placement_mode:
                    self._ball_preview_pub.publish(self._ball_pos)
                    self.get_logger().info("足球放置模式已开启（方向键移动，回车确认）")
                else:
                    self._ball_preview_pub.publish(Point(x=0.0, y=0.0, z=-1.0))
                    self.get_logger().info("足球放置模式已关闭")
                return
            # o 键：随机球速度（xy 方向，速度 0~0.3）
            if k == "o":
                speed = random.uniform(0.0, 0.3)
                angle = random.uniform(0.0, 2.0 * math.pi)
                vel = Vector3()
                vel.x = speed * math.cos(angle)
                vel.y = speed * math.sin(angle)
                self._ball_vel_pub.publish(vel)
                self.get_logger().info(f"ball velocity set: ({vel.x:.3f}, {vel.y:.3f})")
                return
            # m 键切换足球模式（显示/隐藏球门与球，独立于踢球模型）
            if k == "m":
                self._soccer_mode = not self._soccer_mode
                self._soccer_mode_pub.publish(Bool(data=self._soccer_mode))
                if self._soccer_mode:
                    self.get_logger().info("足球模式已开启：显示球门和球")
                else:
                    self.get_logger().info("足球模式已关闭：隐藏球门和球")
                return
            # 如果是运动按键，加入按下集合
            if k in {"w", "s", "a", "d", "q", "e"}:
                self._pressed_keys.add(k)
            # 如果是事件/指令按键，存入单次 pending，并维护运动授权状态
            elif k in self._KEY_MAP:
                spec = self._KEY_MAP[k]
                self._pending = spec
                if spec.intent_type == "status":
                    if spec.command == "walk":
                        self._motion_enabled = True
                    elif spec.command in {"ready", "damp", "reinit"}:
                        self._motion_enabled = False

    def _on_release(self, key: keyboard.Key | keyboard.KeyCode | None) -> None:
        """pynput 按键释放回调（在监听线程中执行）。"""
        try:
            if hasattr(key, "char") and key.char is not None:
                k = key.char.lower()
            else:
                return
        except (AttributeError, ValueError):
            return

        with self._lock:
            # 运动键松开时，从集合中移除
            if k in self._pressed_keys:
                self._pressed_keys.discard(k)

    def _handle_timer(self) -> None:
        """ROS timer 回调（在主线程中执行）。"""
        spec: Optional[IntentSpec] = None
        with self._lock:
            if self._pending is not None:
                # 1. 瞬时事件优先发布
                spec = self._pending
                self._pending = None
            elif self._motion_enabled:
                # 2. 如果当前处于行走状态，无事件时周期发布连续控制速度
                vx = (1.0 if "w" in self._pressed_keys else 0.0) - (1.0 if "s" in self._pressed_keys else 0.0)
                vy = (1.0 if "a" in self._pressed_keys else 0.0) - (1.0 if "d" in self._pressed_keys else 0.0)
                az = (1.0 if "q" in self._pressed_keys else 0.0) - (1.0 if "e" in self._pressed_keys else 0.0)

                spec = IntentSpec(
                    intent_type="motion",
                    command="walk",
                    linear_x_mps=vx * self._max_linear_x_mps,
                    linear_y_mps=vy * self._max_linear_y_mps,
                    angular_z_radps=az * self._max_angular_z_radps
                )

        if spec is not None:
            intent = self._builder.build(spec)
            intent.header.stamp = self.get_clock().now().to_msg()
            self._intent_pub.publish(intent)

    def destroy_node(self) -> bool:
        if hasattr(self, "_listener") and self._listener.is_alive():
            self._listener.stop()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    """运行 robot_keyboard_node。"""
    rclpy.init(args=args)
    node = RobotKeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
