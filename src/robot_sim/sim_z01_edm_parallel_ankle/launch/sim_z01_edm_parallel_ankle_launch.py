from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

# ── 并联腿仿真使用说明 ──────────────────────────────────────────────────
# 本仿真节点发布并联空间的踝关节状态（active motor positions）。
# 通过 robot_bringup 启动时需传入 ankle_env_parallel:=true，告知 robot_control
# 环境处于并联空间，避免对已为并联的数据再做串并联转换：
#
#   ros2 launch robot_bringup zl_robot.launch.py \
#       enable_mujoco_sim:=true \
#       ankle_env_parallel:=true
#
# 独立启动时仅运行仿真渲染，不涉及 robot_control 的踝关节转换。

ROS_NODE_LOG_ARGS = ["--ros-args", "--disable-external-lib-logs"]

def generate_launch_description():
    # 独立启动 sim_z01_edm_parallel_ankle 时，将仿真记录文件放到当前工作目录的 logs/mj_sim。
    # 组合启动时 robot_bringup 只是 include 本 launch，不会启动真实硬件驱动。
    current_dir = os.getcwd()
    logs_dir = os.path.join(current_dir, 'logs', 'mj_sim')
    os.makedirs(logs_dir, exist_ok=True)
    # 这里只传运行时路径。仿真控制默认 enable_control=true，关节数量由
    # MuJoCo XML 和 z01_edm_parallel_ankle.h/kJointsNum 决定，不再通过独立 YAML 配置。
    sim_t4_node = Node(
        package='sim_z01_edm_parallel_ankle',
        executable='sim_z01_edm_parallel_ankle',
        name='sim_z01_edm_parallel_ankle',
        # 显式命名空间：本节点的话题名已经是绝对的 "/robot/..."（topic_prefix 拼接），
        # 所以 namespace 只影响节点自身的 FQN。必须显式给的原因见 config YAML 顶部的说明。
        namespace='robot',
        output='screen',
        arguments=ROS_NODE_LOG_ARGS,
        parameters=[
            # 参数0：本仿真包的 config YAML（发布频率与可调常量）
            PathJoinSubstitution([
                FindPackageShare('sim_z01_edm_parallel_ankle'),
                'config/sim_z01_edm_parallel_ankle.yaml'
            ]),
            # 参数1: XML路径（硬编码默认值）
            {
                'xml_path': PathJoinSubstitution([
                    FindPackageShare('mj_sim'),
                    'description/Z01_edm_parallel_ankle/xml/Z01_edm_parallel_ankle.xml'
                ]),
                'record_folder': logs_dir
            }
        ]
    )

    return LaunchDescription([sim_t4_node])
