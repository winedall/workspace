#!/usr/bin/env python3
import os
import threading
from robot_ota.server.config import _find_workspace_root
import rclpy
from rclpy.node import Node
import uvicorn


class OTAServerNode(Node):
    def __init__(self):
        super().__init__("robot_ota_node")

        _workspace = _find_workspace_root()
        _default_data_root = os.path.join(_workspace, "data") if _workspace else "./data"

        self.declare_parameter("data_root", _default_data_root)
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8000)
        os.environ["DATA_ROOT"] = self.get_parameter("data_root").value

        host = self.get_parameter("host").value
        port = self.get_parameter("port").value
        self.get_logger().info(f"OTA Server starting on {host}:{port}, data_root={os.environ['DATA_ROOT']}")

        self._server_thread = threading.Thread(
            target=uvicorn.run,
            kwargs={
                "app": "robot_ota.server.app:app",
                "host": host,
                "port": port,
                "reload": False,
                "log_level": "info",
            },
            daemon=True,
        )
        self._server_thread.start()


def main(args=None):
    rclpy.init(args=args)
    node = OTAServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
