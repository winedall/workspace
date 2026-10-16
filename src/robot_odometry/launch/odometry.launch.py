"""单独启动里程计节点的 launch 文件。

bringup 尚未包含本节点。需要合入顶层 bringup 时，可用
IncludeLaunchDescription 引入本文件，或把 config/odometry.yaml 的参数
并入 bringup 的参数文件后直接起可执行文件。

单独调试：

    ros2 launch robot_odometry odometry.launch.py
    ros2 launch robot_odometry odometry.launch.py model_name:=z01_std_odom_no_acc
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_node(context: LaunchContext) -> list:
    """按是否显式传入 model_name 组装参数列表。

    参数文件与关键字参数会合并，但空字符串同样会覆盖参数文件里的取值，
    因此只在显式提供时才追加覆盖项。
    """
    params_file = LaunchConfiguration("params_file").perform(context)
    model_name = LaunchConfiguration("model_name").perform(context)

    parameters: list = [params_file]
    if model_name:
        parameters.append({"model_name": model_name})

    return [
        Node(
            package="robot_odometry",
            executable="robot_odometry_node",
            name="robot_odometry_node",
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """构造里程计节点的启动描述。"""
    default_params = os.path.join(
        get_package_share_directory("robot_odometry"),
        "config",
        "odometry.yaml",
    )

    arguments = [
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="节点参数文件路径。",
        ),
        DeclareLaunchArgument(
            "model_name",
            default_value="",
            description=(
                "使用的里程计模型名，必须在 config/odometry_models.yaml 中注册；"
                "留空则取注册表的 default_model_name。"
            ),
        ),
    ]

    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_node)])
