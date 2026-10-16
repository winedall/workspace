from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = get_package_share_directory("robot_injector") + "/config/injector_params.yaml"

    mode = LaunchConfiguration("mode")
    order = LaunchConfiguration("order")
    joint_name = LaunchConfiguration("joint_name")
    count = LaunchConfiguration("count")

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="", description="system_fault / joint_warning / both"),
        DeclareLaunchArgument("order", default_value="warning_first", description="warning_first / fault_first"),
        DeclareLaunchArgument("joint_name", default_value="waist_yaw", description="joint name for joint_warning"),
        DeclareLaunchArgument("count", default_value="3", description="publish ticks; <=0 means continuous"),
        Node(
            package="robot_injector",
            executable="fault_injector_node",
            name="fault_injector_node",
            output="screen",
            parameters=[config, {
                "mode": mode,
                "order": order,
                "joint_name": joint_name,
                "count": count,
            }],
        ),
    ])
