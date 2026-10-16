from __future__ import annotations

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import AudioRequest


class AudioClient:
    def __init__(self, node, topic: str = "audio/request") -> None:
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )
        self._publisher = node.create_publisher(AudioRequest, topic, qos)

    def play_tip(self, key: str, priority: int = 100, interrupt: bool = False) -> None:
        self._publish(AudioRequest.KIND_TIP, key, priority, interrupt)

    def play_file(self, name: str, priority: int = 100, interrupt: bool = False) -> None:
        self._publish(AudioRequest.KIND_FILE, name, priority, interrupt)

    def speak_text(self, text: str, priority: int = 100, interrupt: bool = False) -> None:
        self._publish(AudioRequest.KIND_TEXT, text, priority, interrupt)

    def _publish(self, kind: int, name: str, priority: int, interrupt: bool) -> None:
        msg = AudioRequest()
        msg.kind = kind
        msg.name = str(name)
        msg.priority = int(priority)
        msg.interrupt = bool(interrupt)
        self._publisher.publish(msg)
