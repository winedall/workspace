"""Launch robot_driver in config mode (admin services only, no control loop)."""

from __future__ import annotations

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

ROS_NODE_LOG_ARGS = ["--ros-args", "--disable-external-lib-logs"]


def _bringup_config_path(file_name: str) -> str:
    return os.path.join(
        get_package_share_directory("robot_bringup"), "config", file_name
    )


def generate_launch_description() -> LaunchDescription:
    # 系统配置
    arg_use_sim_time = LaunchConfiguration("arg_use_sim_time")
    arg_robot_type = LaunchConfiguration("arg_robot_type")
    arg_config_root = LaunchConfiguration("arg_config_root")
    arg_hardware_profile = LaunchConfiguration("arg_hardware_profile")

    # 配置文件路径：使用 config_root 拼接绝对路径
    arg_file_robot_joints = LaunchConfiguration("arg_file_robot_joints")
    arg_file_hardware = LaunchConfiguration("arg_file_hardware")
    expr_robot_joints_file = PythonExpression(
        [
            "'robot_joints_' + ('mix' if '",
            arg_hardware_profile,
            "'.startswith('mix') else '",
            arg_hardware_profile,
            "') + '.yaml' if 'humanoid' == '",
            arg_robot_type,
            "' else 'robot_joints.yaml'",
        ]
    )
    expr_hardware_file_name = PythonExpression(
        [
            "'hw_' + '",
            arg_hardware_profile,
            "' + '.yaml'",
        ]
    )

    return LaunchDescription(
        [
            GroupAction(
                [
                    PushRosNamespace("robot"),
                    # 系统参数
                    DeclareLaunchArgument("arg_use_sim_time", default_value="false"),
                    DeclareLaunchArgument("arg_robot_type", default_value="humanoid"),
                    DeclareLaunchArgument(
                        "arg_config_root",
                        default_value=PathJoinSubstitution(
                            [FindPackageShare("robot_bringup"), "config"]
                        ),
                    ),
                    DeclareLaunchArgument("arg_hardware_profile", default_value="mix_canfd"),
                    DeclareLaunchArgument(
                        "arg_file_robot_joints",
                        default_value=PathJoinSubstitution(
                            [arg_config_root, arg_robot_type, expr_robot_joints_file]
                        ),
                    ),
                    DeclareLaunchArgument(
                        "arg_file_hardware",
                        default_value=PathJoinSubstitution(
                            [
                                arg_config_root,
                                arg_robot_type,
                                expr_hardware_file_name,
                            ]
                        ),
                    ),
                    # 仅启动 robot_driver 节点，config_mode=true
                    # - 不启动 EtherCAT worker 线程，不执行使能序列
                    # - 只注册 admin srv 服务，供单独的电机配置使用
                    # - 禁止 /control/hardware_commands 订阅，避免运动命令下发
                    # - 不发布 JointStates/DriverStatus 定时消息
                    Node(
                        package="robot_driver",
                        executable="robot_driver_node",
                        name="robot_driver_node",
                        prefix="taskset -c 3",
                        output="screen",
                        arguments=ROS_NODE_LOG_ARGS,
                        parameters=[
                            _bringup_config_path("driver_params.yaml"),
                            {
                                "use_sim_time": arg_use_sim_time,
                                "robot_type": arg_robot_type,
                            },
                            {
                                "robot_joints_config": arg_file_robot_joints,
                                "hardware_config": arg_file_hardware,
                                "config_mode": True,  # 关键：配置模式
                            },
                        ],
                    ),
                ]
            ),
        ]
    )
