from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

# ── RoboCup 多机器人仿真使用说明 ────────────────────────────────────────
# 本 launch 只启动**仿真进程**：一个 MuJoCo 窗口 + 6 台机器人（robot_01..robot_06），
# 每台一套 /robot_0X/* 话题，外加 /sim/* 全局话题与 /reset_* 服务。
#
# 6 套控制栈（robot_control / robot_rl_manager）不在本 launch 里启动，
# 需要用话题 remap 各自启动（ROS 2 的 remap 是整名匹配，必须逐话题写）；
# 完整 remap 列表见包 README。
#
#   ros2 launch sim_z01_std_robocup sim_z01_std_robocup_launch.py
#
# 只用其中一台做单机联调时，可以只起这套仿真 + 一套带 remap 的控制栈。

ROS_NODE_LOG_ARGS = ["--ros-args", "--disable-external-lib-logs"]

def generate_launch_description():
    # 仿真记录文件放到当前工作目录的 logs/mj_sim（与其它 sim 包一致）。
    current_dir = os.getcwd()
    logs_dir = os.path.join(current_dir, 'logs', 'mj_sim')
    os.makedirs(logs_dir, exist_ok=True)

    sim_node = Node(
        package='sim_z01_std_robocup',
        executable='sim_z01_std_robocup',
        name='sim_z01_std_robocup',
        # 显式命名空间：本节点的话题名已经是绝对的 "/robot/..."（topic_prefix 拼接），
        # 所以 namespace 只影响节点自身的 FQN。必须显式给的原因见 config YAML 顶部的说明。
        namespace='robot',
        output='screen',
        arguments=ROS_NODE_LOG_ARGS,
        parameters=[
            # 参数0：本仿真包的 config YAML（发布频率、出生点规则、相机等）
            PathJoinSubstitution([
                FindPackageShare('sim_z01_std_robocup'),
                'config/sim_z01_std_robocup.yaml'
            ]),
            # 参数1：模型路径（随 mj_sim 安装，资产来自 robot_assets/Z01_std_robocup）
            {
                'xml_path': PathJoinSubstitution([
                    FindPackageShare('mj_sim'),
                    'description/Z01_std_robocup/xml/Z01_std_robocup.xml'
                ]),
                'record_folder': logs_dir
            }
        ]
    )

    return LaunchDescription([sim_node])
