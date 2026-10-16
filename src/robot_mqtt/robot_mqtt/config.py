from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class TopicBridge:
    """单条桥接规则：描述一个 ROS topic 和一个 MQTT topic 的映射关系。"""

    ros_topic: str
    mqtt_topic: str
    message_type: str


@dataclass(frozen=True)
class BridgeConfig:
    """robot_mqtt 的完整运行配置。

    ros_to_mqtt 使用 ROS topic 作为索引，方便 ROS 订阅回调定位目标 MQTT topic。
    mqtt_to_ros 使用 MQTT topic 作为索引，方便 MQTT 消息回调定位目标 ROS publisher。
    """

    mqtt_broker: str
    mqtt_port: int
    mqtt_client_id: str
    mqtt_qos: int
    ros_to_mqtt: dict[str, TopicBridge]
    mqtt_to_ros: dict[str, TopicBridge]

    @classmethod
    def from_parameter_mapping(cls, mapping: dict[str, Any]) -> "BridgeConfig":
        """从 ROS 参数 YAML 的完整字典中提取 robot_mqtt 参数段。"""

        params = mapping.get("robot_mqtt", {}).get("ros__parameters", {})
        return cls.from_ros_parameters(params)

    @classmethod
    def from_yaml_file(cls, path: str | Path) -> "BridgeConfig":
        """从 bringup 传入的 YAML 文件路径读取桥接配置。"""

        with Path(path).open("r", encoding="utf-8") as stream:
            mapping = yaml.safe_load(stream) or {}
        return cls.from_parameter_mapping(mapping)

    @classmethod
    def from_ros_parameters(cls, params: dict[str, Any]) -> "BridgeConfig":
        """把 YAML 中的列表形式映射规范化为运行时查找更方便的字典。"""

        ros_to_mqtt = {
            item["ros_topic"]: TopicBridge(
                ros_topic=item["ros_topic"],
                mqtt_topic=item["mqtt_topic"],
                message_type=item["type"],
            )
            for item in params.get("ros_to_mqtt", [])
        }
        mqtt_to_ros = {
            item["mqtt_topic"]: TopicBridge(
                ros_topic=item["ros_topic"],
                mqtt_topic=item["mqtt_topic"],
                message_type=item["type"],
            )
            for item in params.get("mqtt_to_ros", [])
        }
        return cls(
            mqtt_broker=str(params.get("mqtt_broker", "localhost")),
            mqtt_port=int(params.get("mqtt_port", 1883)),
            mqtt_client_id=str(params.get("mqtt_client_id", "robot_mqtt")),
            mqtt_qos=int(params.get("mqtt_qos", 1)),
            ros_to_mqtt=ros_to_mqtt,
            mqtt_to_ros=mqtt_to_ros,
        )
