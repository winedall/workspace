import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config", "ota_params.yaml",
    )
    port = LaunchConfiguration("port")
    return LaunchDescription([
        DeclareLaunchArgument(
            "port",
            default_value="8000",
            description="OTA 服务监听端口",
        ),
        Node(
            package="robot_ota",
            executable="robot_ota_node",
            name="robot_ota_node",
            output="screen",
            parameters=[config, {"port": ParameterValue(port, value_type=int)}],
        ),
    ])
