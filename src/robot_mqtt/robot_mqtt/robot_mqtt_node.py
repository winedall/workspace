from __future__ import annotations

import json
import time
import uuid

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from rosidl_runtime_py import message_to_ordereddict, set_message_fields
from rosidl_runtime_py.utilities import get_message

from robot_mqtt.config import BridgeConfig


class RobotMqttNode(Node):
    """MQTT 与 ROS2 的双向桥接节点。

    节点本身不写死业务 topic；所有桥接关系由 robot_bringup/config/mqtt_params.yaml
    提供，便于不同部署环境调整 broker 和 topic 映射。
    """

    def __init__(self) -> None:
        super().__init__("robot_mqtt")
        self._declare_parameters()

        # ROS 参数只传配置文件路径；桥接映射仍保留在 YAML 中，避免 ROS 参数类型限制。
        config_file = self.get_parameter("config_file").value
        self._config = BridgeConfig.from_yaml_file(config_file)

        # 保存 publisher/subscription 引用，防止 Python 对象被垃圾回收。
        self._ros_publishers = {}
        self._ros_subscriptions = []
        self._mqtt_connected = False
        self._mqtt_stopping = False
        self._last_disconnected_publish_warning_time = 0.0

        # 每次启动追加 UUID，避免多机器人或异常重启时 MQTT client_id 冲突。
        client_id = f"{self._config.mqtt_client_id}-{uuid.uuid4().hex}"
        self._mqtt_client = mqtt.Client(client_id=client_id)
        self._mqtt_client.on_connect = self._on_mqtt_connect
        self._mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self._mqtt_client.on_message = self._on_mqtt_message

        self._setup_ros_to_mqtt()
        self._setup_mqtt_to_ros()
        self._connect_mqtt()

        self.get_logger().info(
            "Robot MQTT bridge ready: "
            f"ROS->MQTT {len(self._config.ros_to_mqtt)}, "
            f"MQTT->ROS {len(self._config.mqtt_to_ros)}"
        )

    def _declare_parameters(self) -> None:
        """声明 bringup 传入的配置文件参数。"""

        self.declare_parameter("config_file", "")

    def _connect_mqtt(self) -> None:
        """异步连接 MQTT broker，并启动 paho-mqtt 后台网络循环。

        connect_async 配合 loop_start 后，启动时 broker 不可达也不会阻塞节点；
        paho 会按 reconnect_delay_set 设置的退避间隔在后台持续重连。
        """

        try:
            self._mqtt_client.reconnect_delay_set(min_delay=1, max_delay=30)
            self._mqtt_client.connect_async(self._config.mqtt_broker, self._config.mqtt_port, 5)
            self._mqtt_client.loop_start()
            self.get_logger().info(
                f"Connecting to MQTT at {self._config.mqtt_broker}:{self._config.mqtt_port}"
            )
        except Exception as exc:
            self.get_logger().error(f"MQTT connection failed: {exc}")

    def _setup_ros_to_mqtt(self) -> None:
        """根据配置创建 ROS 订阅：收到 ROS 消息后转成 JSON 发到 MQTT。"""

        for bridge in self._config.ros_to_mqtt.values():
            try:
                msg_class = get_message(bridge.message_type)
                sub = self.create_subscription(
                    msg_class,
                    bridge.ros_topic,
                    lambda msg, topic=bridge.mqtt_topic: self._publish_mqtt(topic, msg),
                    10,
                )
                self._ros_subscriptions.append(sub)
                self.get_logger().info(
                    f"ROS->MQTT: {bridge.ros_topic} ({bridge.message_type}) -> {bridge.mqtt_topic}"
                )
            except Exception as exc:
                self.get_logger().error(f"ROS->MQTT setup error for {bridge.ros_topic}: {exc}")

    def _setup_mqtt_to_ros(self) -> None:
        """根据配置创建 ROS publisher：收到 MQTT JSON 后还原成 ROS 消息发布。"""

        for bridge in self._config.mqtt_to_ros.values():
            try:
                msg_class = get_message(bridge.message_type)
                self._ros_publishers[bridge.ros_topic] = self.create_publisher(
                    msg_class, bridge.ros_topic, 10
                )
                self.get_logger().info(
                    f"MQTT->ROS: {bridge.mqtt_topic} -> {bridge.ros_topic} ({bridge.message_type})"
                )
            except Exception as exc:
                self.get_logger().error(f"MQTT->ROS setup error for {bridge.mqtt_topic}: {exc}")

    def _publish_mqtt(self, mqtt_topic: str, ros_msg) -> None:
        """把任意 ROS 消息转换为 JSON，并发布到目标 MQTT topic。"""

        try:
            if not self._mqtt_connected:
                self._warn_disconnected_publish(mqtt_topic)
                return

            payload = json.dumps(message_to_ordereddict(ros_msg), default=str)
            self._mqtt_client.publish(mqtt_topic, payload=payload, qos=self._config.mqtt_qos)
        except Exception as exc:
            self.get_logger().error(f"ROS->MQTT publish error for {mqtt_topic}: {exc}")

    def _warn_disconnected_publish(self, mqtt_topic: str) -> None:
        """MQTT 未连接时丢弃周期状态消息，并对日志做简单限频。"""

        now = time.monotonic()
        if now - self._last_disconnected_publish_warning_time < 5.0:
            return

        self._last_disconnected_publish_warning_time = now
        self.get_logger().warning(
            f"MQTT is disconnected; dropping ROS->MQTT message for {mqtt_topic}"
        )

    def _on_mqtt_connect(self, client, userdata, flags, rc) -> None:
        """MQTT 连接建立后订阅所有 MQTT->ROS 方向的 topic。"""

        if rc != 0:
            self._mqtt_connected = False
            self.get_logger().error(f"MQTT connection failed with code {rc}")
            return

        self._mqtt_connected = True
        self.get_logger().info("MQTT connected, subscribing to configured topics")
        for mqtt_topic in self._config.mqtt_to_ros:
            client.subscribe(mqtt_topic, qos=self._config.mqtt_qos)

    def _on_mqtt_disconnect(self, client, userdata, rc) -> None:
        """记录 MQTT 断线状态；非主动退出时由 paho 后台继续重连。"""

        self._mqtt_connected = False
        if self._mqtt_stopping:
            return

        self.get_logger().warning(
            f"MQTT disconnected unexpectedly with code {rc}; reconnecting in background"
        )

    def _on_mqtt_message(self, client, userdata, msg) -> None:
        """处理 MQTT 入站消息，并按 topic 映射发布到 ROS。"""

        bridge = self._config.mqtt_to_ros.get(msg.topic)
        if bridge is None:
            return

        try:
            ros_msg = self._json_to_ros_message(bridge.message_type, msg.payload.decode())
            self._ros_publishers[bridge.ros_topic].publish(ros_msg)
        except Exception as exc:
            self.get_logger().error(f"MQTT->ROS publish error for {msg.topic}: {exc}")

    @staticmethod
    def _json_to_ros_message(message_type: str, payload: str):
        """把 JSON payload 填充到配置指定的 ROS 消息类型中。"""

        msg_class = get_message(message_type)
        ros_msg = msg_class()
        set_message_fields(ros_msg, json.loads(payload))
        return ros_msg

    def shutdown(self) -> None:
        """节点退出时停止 MQTT 网络循环并断开连接。"""

        self.get_logger().info("Shutting down Robot MQTT bridge")
        self._mqtt_stopping = True
        self._mqtt_client.disconnect()
        self._mqtt_client.loop_stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RobotMqttNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 构造节点失败时 node 仍为 None，避免旧版代码中 finally 访问未定义变量的问题。
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
