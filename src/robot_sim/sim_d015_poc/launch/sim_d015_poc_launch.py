import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    logs_dir = os.path.join(os.getcwd(), "logs", "mj_sim")
    os.makedirs(logs_dir, exist_ok=True)

    return LaunchDescription([
        Node(
            package="sim_d015_poc",
            executable="sim_d015_poc",
            name="sim_d015_poc",
            # 显式命名空间：本节点的话题名已经是绝对的 "/robot/..."（topic_prefix 拼接），
            # 所以 namespace 只影响节点自身的 FQN。必须显式给的原因见 config YAML 顶部的说明。
            namespace="robot",
            output="screen",
            parameters=[
            # 参数0：本仿真包的 config YAML（发布频率与可调常量）
            PathJoinSubstitution([
                FindPackageShare('sim_d015_poc'),
                'config/sim_d015_poc.yaml'
            ]),
            {
                "xml_path": PathJoinSubstitution([
                    FindPackageShare("mj_sim"),
                    "description/D015_poc/xml/D015_poc.xml",
                ]),
                "record_folder": logs_dir,
            }],
        )
    ])
