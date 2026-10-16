"""以安全默认值启动完整机器人控制系统。"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

ROS_NODE_LOG_ARGS = ["--ros-args", "--disable-external-lib-logs"]


def _bringup_config_path(file_name: str) -> str:
    """返回已安装的 robot_bringup YAML 参数文件路径。"""
    return os.path.join(get_package_share_directory("robot_bringup"), "config", file_name)


def _build_launch_context() -> dict:
    """集中创建 launch 参数、派生表达式、路径、条件和公共节点参数。

    public launch 参数名也使用 arg_ 前缀；expr_、file_、dir_ 和 cond_
    只用于内部派生值，避免把内部计算结果暴露为外部接口。
    """
    # 参数配置。
    arg_use_sim_time = LaunchConfiguration("arg_use_sim_time")
    arg_dir_recorder = LaunchConfiguration("arg_dir_recorder")
    arg_recorder_mode = LaunchConfiguration("arg_recorder_mode")
    arg_robot_type = LaunchConfiguration("arg_robot_type")
    arg_config_root = LaunchConfiguration("arg_config_root")
    arg_hardware_profile = LaunchConfiguration("arg_hardware_profile")
    arg_runtime_platform = LaunchConfiguration("arg_runtime_platform")
    arg_parallel_ankle = LaunchConfiguration("arg_parallel_ankle")

    # 文件配置。
    arg_file_robot_joints = LaunchConfiguration("arg_file_robot_joints")
    arg_file_hardware = LaunchConfiguration("arg_file_hardware")
    arg_file_rl_models = LaunchConfiguration("arg_file_rl_models")
    arg_file_action_audio = LaunchConfiguration("arg_file_action_audio")
    arg_file_prepare_profiles = LaunchConfiguration("arg_file_prepare_profiles")
    arg_file_domain_bridge = LaunchConfiguration("arg_file_domain_bridge")
    arg_file_cyclonedds = LaunchConfiguration("arg_file_cyclonedds")

    # 可选子系统和设备驱动。
    arg_enable_teleop = LaunchConfiguration("arg_enable_teleop")
    arg_enable_keyboard = LaunchConfiguration("arg_enable_keyboard")
    arg_enable_audio = LaunchConfiguration("arg_enable_audio")
    arg_enable_external = LaunchConfiguration("arg_enable_external")
    arg_enable_head_pose = LaunchConfiguration("arg_enable_head_pose")
    arg_enable_recorder = LaunchConfiguration("arg_enable_recorder")
    arg_enable_battery = LaunchConfiguration("arg_enable_battery")
    arg_enable_monitor = LaunchConfiguration("arg_enable_monitor")
    arg_enable_mqtt = LaunchConfiguration("arg_enable_mqtt")
    arg_enable_domain_bridge = LaunchConfiguration("arg_enable_domain_bridge")
    arg_enable_joy_node = LaunchConfiguration("arg_enable_joy_node")
    arg_enable_mujoco_sim = LaunchConfiguration("arg_enable_mujoco_sim")
    arg_enable_camera = LaunchConfiguration("arg_enable_camera")
    arg_enable_odometry = LaunchConfiguration("arg_enable_odometry")

    # 派生表达式。
    # 人形真机固定使用并联踝环境；仿真可通过 parallel_ankle 选择串联或并联。
    expr_parallel_ankle = PythonExpression(
        [
            "'true' if 'humanoid' == '",
            arg_robot_type,
            "' and ('false' == '",
            arg_enable_mujoco_sim,
            "' or 'true' == '",
            arg_parallel_ankle,
            "') else 'false'",
        ]
    )
    # MuJoCo 仿真按机器人形态选择不同 package 和 launch 文件名。
    # 仿真电机供应商：encos → sim_z01_edm_*（Encos 驱动器）；tienkung → sim_z01_std_*（TienKung 驱动器）。
    arg_sim_motor_vendor = LaunchConfiguration("arg_sim_motor_vendor")
    # 足球仿真（tienkung 并联踝 + 球场/球门场景），由 arg_enable_football 单独开启。
    arg_enable_football = LaunchConfiguration("arg_enable_football")
    expr_sim_package = PythonExpression(
        [
            "'sim_d040_poc' if 'dog' == '",
            arg_robot_type,
            "' else 'sim_z01_std_football_player' if 'true' == '",
            arg_enable_football,
            "' else 'sim_z01_std_parallel_ankle' if ('tienkung' == '",
            arg_sim_motor_vendor,
            "' and 'true' == '",
            arg_parallel_ankle,
            "') else 'sim_z01_std_add_head' if 'tienkung' == '",
            arg_sim_motor_vendor,
            "' else 'sim_z01_edm_parallel_ankle' if 'true' == '",
            arg_parallel_ankle,
            "' else 'sim_z01_edm_add_head'",
        ]
    )
    expr_sim_launch_file = PythonExpression(
        [
            "'sim_d040_poc_launch.py' if 'dog' == '",
            arg_robot_type,
            "' else 'sim_z01_std_football_player_launch.py' if 'true' == '",
            arg_enable_football,
            "' else 'sim_z01_std_parallel_ankle_launch.py' if ('tienkung' == '",
            arg_sim_motor_vendor,
            "' and 'true' == '",
            arg_parallel_ankle,
            "') else 'sim_z01_std_add_head_launch.py' if 'tienkung' == '",
            arg_sim_motor_vendor,
            "' else 'sim_z01_edm_parallel_ankle_launch.py' if 'true' == '",
            arg_parallel_ankle,
            "' else 'sim_z01_edm_add_head_launch.py'",
        ]
    )
    # 人形仿真使用 robot_joints_sim.yaml；人形真机按硬件 profile 选择关节配置。
    # 四足当前仍使用标准 robot_joints.yaml。
    expr_robot_joints_file = PythonExpression(
        [
            "'robot_joints_sim.yaml' if 'true' == '",
            arg_enable_mujoco_sim,
            "' and 'humanoid' == '",
            arg_robot_type,
            "' else 'robot_joints_' + ('mix' if '",
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
    expr_default_model = PythonExpression(
        # ["'t4_walk' if 'humanoid' == '", arg_robot_type, "' else 'd4_flat'"]
        ["'z01_pa_walk' if 'humanoid' == '", arg_robot_type, "' else 'd4_flat'"]
        # ["'z01_std_walk' if 'humanoid' == '", arg_robot_type, "' else 'd4_flat'"]
    )
    # 内部路径和 include launch 文件。
    file_launch_realsense = PathJoinSubstitution(
        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
    )
    file_launch_imu = PathJoinSubstitution(
        [FindPackageShare("yesense_std_ros2"), "launch", "yesense_node.launch.py"]
    )
    file_launch_mujoco = PathJoinSubstitution(
        [FindPackageShare(expr_sim_package), "launch", expr_sim_launch_file]
    )
    file_launch_odometry = PathJoinSubstitution(
        [FindPackageShare("robot_odometry"), "launch", "odometry.launch.py"]
    )
    dir_actions = PathJoinSubstitution([arg_config_root, "data", "actions"])
    file_tips_manifest = PathJoinSubstitution([arg_config_root, "data", "tips.yaml"])

    # 条件对象集中创建，节点分组函数只负责描述动作。
    cond_enable_camera = IfCondition(arg_enable_camera)
    cond_real_robot = UnlessCondition(arg_enable_mujoco_sim)
    cond_mujoco_sim = IfCondition(arg_enable_mujoco_sim)
    cond_enable_joy_node = IfCondition(arg_enable_joy_node)
    cond_enable_teleop = IfCondition(arg_enable_teleop)
    cond_enable_keyboard = IfCondition(arg_enable_keyboard)
    cond_enable_external = IfCondition(arg_enable_external)
    cond_enable_head_pose = IfCondition(arg_enable_head_pose)
    cond_enable_audio = IfCondition(arg_enable_audio)
    cond_enable_recorder = IfCondition(arg_enable_recorder)
    cond_enable_battery = IfCondition(arg_enable_battery)
    cond_enable_monitor = IfCondition(arg_enable_monitor)
    cond_enable_mqtt = IfCondition(arg_enable_mqtt)
    cond_enable_domain_bridge = IfCondition(arg_enable_domain_bridge)
    cond_enable_odometry = IfCondition(arg_enable_odometry)

    # 公开 launch 参数名已经规范化，但 ROS 节点参数名保持不变，以兼容节点实现。
    params_common = {
        "use_sim_time": arg_use_sim_time,
        "enable_mujoco_sim": arg_enable_mujoco_sim,
        "ankle_env_parallel": ParameterValue(expr_parallel_ankle, value_type=bool),
        "robot_type": arg_robot_type,
    }

    return locals()


def _declare_launch_arguments(ctx: dict) -> list:
    """声明公开 launch 参数接口及其默认值。"""
    return [
        # 基础运行设置。
        DeclareLaunchArgument("arg_use_sim_time", default_value="false"),
        DeclareLaunchArgument("arg_robot_type", default_value="humanoid"),
        DeclareLaunchArgument("arg_parallel_ankle", default_value="true"),
        DeclareLaunchArgument("arg_sim_motor_vendor", default_value="tienkung"),
        DeclareLaunchArgument("arg_enable_football", default_value="true"),
        DeclareLaunchArgument("arg_config_root", default_value=""),
        DeclareLaunchArgument("arg_hardware_profile", default_value="mix_canfd"),
        DeclareLaunchArgument("arg_runtime_platform", default_value=""),
        DeclareLaunchArgument("arg_dir_recorder", default_value="logs"),
        DeclareLaunchArgument("arg_recorder_mode", default_value="event_window"),
        # 配置文件参数。默认路径直接写在这里，避免只用一次的默认值进入 paths 字典。
        DeclareLaunchArgument(
            "arg_file_robot_joints",
            default_value=PathJoinSubstitution(
                [ctx["arg_config_root"], ctx["arg_robot_type"], ctx["expr_robot_joints_file"]]
            ),
        ),
        DeclareLaunchArgument(
            "arg_file_hardware",
            default_value=PathJoinSubstitution(
                [ctx["arg_config_root"], ctx["arg_robot_type"], ctx["expr_hardware_file_name"]]
            ),
        ),
        DeclareLaunchArgument(
            "arg_file_rl_models",
            default_value=PathJoinSubstitution(
                [ctx["arg_config_root"], ctx["arg_robot_type"], "rl_models.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "arg_file_action_audio",
            default_value=PathJoinSubstitution(
                [ctx["arg_config_root"], ctx["arg_robot_type"], "action_audio.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "arg_file_prepare_profiles",
            default_value=PathJoinSubstitution(
                [ctx["arg_config_root"], ctx["arg_robot_type"], "default_prepare.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "arg_file_domain_bridge",
            default_value=PathJoinSubstitution(
                [ctx["arg_config_root"], "domain_bridge_robot.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "arg_file_cyclonedds",
            default_value=PathJoinSubstitution(
                [ctx["arg_config_root"], "cyclonedds_bridge_whitelist.xml"]
            ),
        ),
        # 可选子系统。
        DeclareLaunchArgument("arg_enable_teleop", default_value="true"),
        DeclareLaunchArgument("arg_enable_keyboard", default_value="false"),
        DeclareLaunchArgument("arg_enable_joy_node", default_value="true"),
        DeclareLaunchArgument("arg_enable_audio", default_value="true"),
        DeclareLaunchArgument("arg_enable_recorder", default_value="true"),
        DeclareLaunchArgument("arg_enable_battery", default_value="false"),
        DeclareLaunchArgument("arg_enable_monitor", default_value="true"),
        DeclareLaunchArgument("arg_enable_mqtt", default_value="false"),
        DeclareLaunchArgument("arg_enable_camera", default_value="false"),
        DeclareLaunchArgument("arg_enable_external", default_value="true"),
        DeclareLaunchArgument("arg_enable_head_pose", default_value="true"),
        DeclareLaunchArgument("arg_enable_mujoco_sim", default_value="false"),
        DeclareLaunchArgument("arg_enable_domain_bridge", default_value="true"),
        DeclareLaunchArgument("arg_enable_odometry", default_value="true"),
    ]


def _sensor_and_simulation_launches(ctx: dict) -> list:
    """返回相机、IMU 和仿真相关的 include launch。"""
    return [
        # RealSense 相机启动时需要等待 10 秒，因此在其他节点启动前先启动相机。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ctx["file_launch_realsense"]),
            condition=ctx["cond_enable_camera"],
        ),
        # 仿真模式下由仿真节点提供 driver/joint_states 和 nav_all，因此不启动真实 IMU 驱动。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ctx["file_launch_imu"]),
            condition=ctx["cond_real_robot"],
        ),
        # MuJoCo 仿真启动后替代 robot_driver_node 发布机器人状态。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ctx["file_launch_mujoco"]),
            condition=ctx["cond_mujoco_sim"],
        ),
    ]


def _control_pipeline_nodes(ctx: dict) -> list:
    """返回状态、路由、控制、驱动和 RL 组成的控制闭环节点。"""
    arg_config_root = ctx["arg_config_root"]
    arg_file_robot_joints = ctx["arg_file_robot_joints"]
    arg_file_hardware = ctx["arg_file_hardware"]
    arg_file_rl_models = ctx["arg_file_rl_models"]
    arg_file_action_audio = ctx["arg_file_action_audio"]
    arg_runtime_platform = ctx["arg_runtime_platform"]
    arg_file_prepare_profiles = ctx["arg_file_prepare_profiles"]
    params_common = ctx["params_common"]

    return [
        # 真实/空跑驱动入口；仿真模式下不启动，避免和仿真节点同时发布 driver/joint_states。
        Node(
            package="robot_driver",
            executable="robot_driver_node",
            name="robot_driver_node",
            # prefix=['gdb -ex run -ex bt --args'],
            prefix="taskset -c 3",
            output="screen",
            arguments=ROS_NODE_LOG_ARGS,
            condition=ctx["cond_real_robot"],
            parameters=[
                _bringup_config_path("driver_params.yaml"),
                params_common,
                {
                    "robot_joints_config": arg_file_robot_joints,
                    "hardware_config": arg_file_hardware,
                    "config_mode": False,  # NEW: 显式声明运控模式，不进入配置模式
                },
            ],
        ),
        # 系统状态机节点，只发布 RobotState，不直接下发关节命令。
        Node(
            package="robot_state",
            executable="robot_state_node",
            name="robot_state_node",
            output="screen",
            arguments=ROS_NODE_LOG_ARGS,
            respawn=True,
            respawn_delay=1.0,
            parameters=[
                _bringup_config_path("state_params.yaml"),
                params_common,
                {"robot_joints_config": arg_file_robot_joints},
            ],
        ),
        # 唯一命令路由节点，负责 RobotIntent 和候选 JointCommands 的准入。
        Node(
            package="robot_router",
            executable="robot_router_node",
            name="robot_router_node",
            output="screen",
            arguments=ROS_NODE_LOG_ARGS,
            respawn=True,
            respawn_delay=1.0,
            parameters=[
                _bringup_config_path("router_params.yaml"),
                params_common,
                {"robot_joints_config": arg_file_robot_joints},
            ],
        ),
        # 控制安全处理节点，负责限幅、跳变处理和最终硬件命令输出。
        Node(
            package="robot_control",
            executable="robot_control_node",
            name="robot_control_node",
            output="screen",
            arguments=ROS_NODE_LOG_ARGS,
            respawn=True,
            respawn_delay=1.0,
            parameters=[
                _bringup_config_path("control_params.yaml"),
                params_common,
                {"robot_joints_config": arg_file_robot_joints},
            ],
        ),
        # RL 管理节点只接收统一状态和意图，输出 desired_commands，不直接访问硬件。
        Node(
            package="robot_rl_manager",
            executable="robot_rl_manager_node",
            name="robot_rl_manager_node",
            output="screen",
            arguments=ROS_NODE_LOG_ARGS,
            respawn=True,
            respawn_delay=1.0,
            parameters=[
                _bringup_config_path("rl_manager_params.yaml"),
                params_common,
                {
                    "config_root": arg_config_root,
                    "robot_joints_config": arg_file_robot_joints,
                    "rl_models_config": arg_file_rl_models,
                    "action_audio_config": arg_file_action_audio,
                    "runtime_platform": arg_runtime_platform,
                    "default_model_name": ctx["expr_default_model"],
                    "prepare_profiles_config": arg_file_prepare_profiles,
                    "samples_root": PathJoinSubstitution(
                        [arg_config_root, "data", "samples"]
                    ),
                },
            ],
        ),
    ]


def _operator_input_nodes(ctx: dict) -> list:
    """返回操作员输入源，并将输入归一化到 RobotIntent 类链路。"""
    params_common = ctx["params_common"]

    return [
        # joy_node 会打开本机输入设备，因此保留独立开关，便于无手柄环境启动。
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            arguments=ROS_NODE_LOG_ARGS,
            condition=ctx["cond_enable_joy_node"],
            respawn=True,
            respawn_delay=1.0,
        ),
        # 外部遥控输入归一化为 RobotIntent，不发布 JointCommands。
        Node(
            package="robot_teleop",
            executable="robot_teleop_node",
            name="robot_teleop_node",
            output="screen",
            arguments=ROS_NODE_LOG_ARGS,
            condition=ctx["cond_enable_teleop"],
            respawn=True,
            respawn_delay=1.0,
            parameters=[
                _bringup_config_path("teleop_params.yaml"),
                params_common,
            ],
        ),
        # 键盘遥控输入归一化为 RobotIntent，与手柄节点并存；通过 enable_keyboard:=true 启动。
        Node(
            package="robot_teleop",
            executable="robot_keyboard_node",
            name="robot_keyboard_node",
            output="screen",
            arguments=ROS_NODE_LOG_ARGS,
            condition=ctx["cond_enable_keyboard"],
            respawn=True,
            respawn_delay=1.0,
            parameters=[
                _bringup_config_path("teleop_params.yaml"),
                params_common,
            ],
        ),
        # 外部控制接口节点，处理头部控制等外部指令的插值和转换。
        Node(
            package="robot_external",
            executable="robot_external_node",
            name="robot_external_node",
            output="screen",
            arguments=ROS_NODE_LOG_ARGS,
            condition=ctx["cond_enable_external"],
            parameters=[
                _bringup_config_path("external_params.yaml"),
                params_common,
            ],
        ),
        # 头部深度相机位姿解算节点：由头部关节角 + IMU 基座倾斜换算
        # /robot/head_pose（camera in odom，geometry_msgs/Pose）。
        # 与 robot_external_node 同用 external_params.yaml 中的 head_pose_node 段。
        Node(
            package="robot_external",
            executable="head_pose_node",
            name="head_pose_node",
            output="screen",
            arguments=ROS_NODE_LOG_ARGS,
            condition=ctx["cond_enable_head_pose"],
            parameters=[
                _bringup_config_path("external_params.yaml"),
                params_common,
            ],
        ),
    ]


def _application_service_nodes(ctx: dict) -> list:
    """返回音频、记录、健康监控、电池和 MQTT 等非控制服务节点。"""
    arg_dir_recorder = ctx["arg_dir_recorder"]
    arg_recorder_mode = ctx["arg_recorder_mode"]
    arg_file_robot_joints = ctx["arg_file_robot_joints"]
    params_common = ctx["params_common"]

    return [
        # 音频提示节点，默认启用；无音频设备或调试时可通过 enable_audio:=false 关闭。
        Node(
            package="robot_audio",
            executable="robot_audio_node",
            name="robot_audio_node",
            output="screen",
            arguments=ROS_NODE_LOG_ARGS,
            condition=ctx["cond_enable_audio"],
            parameters=[
                _bringup_config_path("audio_params.yaml"),
                {
                    "actions_dir": ctx["dir_actions"],
                    "tips_manifest": ctx["file_tips_manifest"],
                },
                params_common,
            ],
        ),
        # 旁路式实时性记录节点，默认不启动；通过 enable_recorder:=true 启用。
        Node(
            package="robot_recorder",
            executable="robot_recorder_node",
            name="robot_recorder_node",
            output="screen",
            arguments=ROS_NODE_LOG_ARGS,
            condition=ctx["cond_enable_recorder"],
            parameters=[
                _bringup_config_path("recorder_params.yaml"),
                params_common,
                {
                    "output_dir": arg_dir_recorder,
                    "recording_mode": arg_recorder_mode,
                    "robot_joints_config": arg_file_robot_joints,
                },
            ],
        ),
        # 电池信息节点，通过 SocketCAN 连接 BMS，发布 battery/info。
        # 无电池设备时可通过 enable_battery:=false 关闭。
        Node(
            package="robot_battery",
            executable="robot_battery_node",
            name="robot_battery_node",
            output="screen",
            arguments=ROS_NODE_LOG_ARGS,
            condition=ctx["cond_enable_battery"],
            parameters=[
                _bringup_config_path("battery_params.yaml"),
                params_common,
            ],
        ),
        # 系统监控节点，采集 CPU/内存/磁盘/温度信息，订阅电池状态，
        # 超阈值时通过 audio request topic 触发提示音播报。
        Node(
            package="robot_monitor",
            executable="robot_monitor_node",
            name="robot_monitor_node",
            output="screen",
            arguments=ROS_NODE_LOG_ARGS,
            condition=ctx["cond_enable_monitor"],
            parameters=[
                _bringup_config_path("monitor_params.yaml"),
                params_common,
            ],
        ),
        # MQTT 桥接节点，默认关闭；启用后把云端意图归一化为 RobotIntent，并上报核心状态。
        Node(
            package="robot_mqtt",
            executable="robot_mqtt_node",
            name="robot_mqtt",
            output="screen",
            arguments=ROS_NODE_LOG_ARGS,
            condition=ctx["cond_enable_mqtt"],
            parameters=[
                {"config_file": _bringup_config_path("mqtt_params.yaml")},
                params_common,
            ],
        ),
    ]


def _odometry_launches(ctx: dict) -> list:
    """返回本体感觉里程计 include launch。"""
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ctx["file_launch_odometry"]),
            condition=ctx["cond_enable_odometry"],
        ),
    ]


def _external_bridge_nodes(ctx: dict) -> list:
    """返回对外暴露白名单 ROS 话题的桥接动作。"""
    return [
        # domain_bridge 作为普通 ROS 节点启动，使用独立 DDS 环境暴露白名单话题。
        Node(
            package="domain_bridge",
            executable="domain_bridge",
            name="domain_bridge",
            output="screen",
            arguments=[ctx["arg_file_domain_bridge"]],
            condition=ctx["cond_enable_domain_bridge"],
            additional_env={
                "ROS_LOCALHOST_ONLY": "0",
                "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                "CYCLONEDDS_URI": ["file://", ctx["arg_file_cyclonedds"]],
            },
        ),
    ]

def generate_launch_description() -> LaunchDescription:
    """组装 launch 参数、派生值、路径和分组动作。"""
    ctx = _build_launch_context()

    return LaunchDescription(
        [
            *_declare_launch_arguments(ctx),
            GroupAction(
                [
                    PushRosNamespace("robot"),
                    *_sensor_and_simulation_launches(ctx),
                    *_control_pipeline_nodes(ctx),
                    *_operator_input_nodes(ctx),
                    *_odometry_launches(ctx),
                    *_application_service_nodes(ctx),
                    *_external_bridge_nodes(ctx),
                ]
            ),
        ]
    )
