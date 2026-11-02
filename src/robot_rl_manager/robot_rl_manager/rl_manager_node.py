"""RL 模型管理与 JointCommands 发布节点。"""

from __future__ import annotations

import copy
import importlib
import math
import time
from pathlib import Path
from typing import Optional

import numpy as np
import yaml
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Point, Twist, Vector3
from robot_interfaces.msg import (
    FollowControl,
    JointCommands,
    JointStates,
    RlModelInfo,
    RobotIntent,
    RobotState,
    RlManagerStatus,
)
from robot_interfaces.srv import ListRlModels, LoadRlModel, SwitchRlModel
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String

from robot_audio import audio_tips
from robot_audio.audio_client import AudioClient
from robot_rl_manager.joint_command_builder import load_joint_configs
from robot_rl_manager.convert.utils import find_repo_root
from robot_rl_manager.model_registry import ModelRegistry, RegistryRlModelInfo
from robot_rl_manager.policy.context import PolicyContext
from robot_rl_manager.policy.factory import create_policy_adapter, create_policy_backend
from robot_rl_manager.policy.imu import PolicyImu
from robot_rl_manager.policy.runtime import PolicyRuntime
from robot_rl_manager.replay.capture import LatestReplaySequenceRecorder

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

try:
    from yesense_interface.msg import NavAll
except ImportError:
    NavAll = None


_CONFIG_VOICE_MAP: dict[tuple[str, str], str] = {
    ("getup", "getup"):  audio_tips.GETUP_CONFIG,
    ("getup", "stand"):  audio_tips.GETUP_STAND,
    ("getup", "lie"):    audio_tips.GETUP_LIE,
    ("getup", "face"):   audio_tips.GETUP_FACE,
    ("speed", "speed"):  audio_tips.SPEED_CONFIG,
    ("speed", "std"):    audio_tips.SPEED_STD,
    ("speed", "high"):   audio_tips.SPEED_HIGH,
    ("nav",   "nav"):    audio_tips.NAV_CONFIG,
    ("nav",   "auto"):   audio_tips.NAV_AUTO,
    ("nav",   "manual"): audio_tips.NAV_MANUAL,
    ("nav",   "follow"): audio_tips.NAV_FOLLOW,
    ("mode",  "mode"):   audio_tips.MODE_CONFIG,
    ("mode",  "rl"):     audio_tips.MODE_RL,
    ("mode",  "mix"):    audio_tips.MODE_MIX,
    ("mode",  "spec"):   audio_tips.MODE_SPEC,
    ("mode",  "app"):    audio_tips.MODE_APP,
}

# 起身配置 profile 名称 → 起身模型名称映射
_GETUP_MODEL_MAP: dict[str, str] = {
    "lie": "t4_lay_up",
    "face": "t4_face_up",
}


class OutputSmoother:
    """模型切换时对 JointCommands 做线性衰减平滑。

    切换瞬间 alpha=1（完全保持旧模型末帧输出），
    N 帧内线性衰减到 alpha=0（完全使用新模型原始输出）。
    position/kp/kd 同时平滑，保证 PD 增益也平滑过渡。
    """

    def __init__(self, transition_frames: int = 10) -> None:
        self._transition_frames = transition_frames
        self._frame = 0
        self._old_positions: list[float] | None = None
        self._old_kps: list[float] | None = None
        self._old_kds: list[float] | None = None
        self._active = False

    def start_transition(self, commands: JointCommands) -> None:
        self._old_positions = [float(c.position) for c in commands.commands]
        self._old_kps = [float(c.kp) for c in commands.commands]
        self._old_kds = [float(c.kd) for c in commands.commands]
        self._frame = 0
        self._active = True

    def filter(self, commands: JointCommands) -> None:
        if not self._active or self._old_positions is None:
            return
        alpha = max(0.0, 1.0 - self._frame / self._transition_frames)
        for i, cmd in enumerate(commands.commands):
            if i < len(self._old_positions):
                cmd.position = alpha * self._old_positions[i] + (1.0 - alpha) * cmd.position
                cmd.kp       = alpha * self._old_kps[i]   + (1.0 - alpha) * cmd.kp
                cmd.kd       = alpha * self._old_kds[i]   + (1.0 - alpha) * cmd.kd
        self._frame += 1
        if self._frame >= self._transition_frames:
            self._active = False
            self._old_positions = None
            self._old_kps = None
            self._old_kds = None


class CommandInterpolator:
    """把 50Hz 推理命令在 200Hz 发布周期内做线性插值，消除零阶保持位置阶跃。

    背景：推理 50Hz 产生一次命令，发布定时器 200Hz 原样重发（零阶保持），
    于是策略相邻两帧的位置差会作为瞬时阶跃直达电机（踝关节尤其明显），
    诱发“命令突跳→力矩塌零→机体倾斜”。本类在发布周期内把上一帧实际发布
    位置线性逼近到最新目标位置，将单帧阶跃摊平成多帧斜坡。

    只对 position 插值；kp/kd/enable/type/velocity 等语义字段一律沿用最新
    目标帧，不改变控制语义。若中途来了新目标，则以“当前已发布位置”为新起点
    重新规划，保证输出连续。
    """

    def __init__(self, total_frames: int) -> None:
        self._total = max(1, int(total_frames))
        self._start: list[float] | None = None
        self._target: list[float] | None = None
        self._frame = 0
        self._published: list[float] | None = None

    def set_target(self, positions: list[float]) -> None:
        """收到新的推理目标：以最近一次实际发布位置为起点规划插值段。"""
        self._start = list(self._published) if self._published is not None else list(positions)
        self._target = list(positions)
        self._frame = 0

    def next_positions(self) -> list[float] | None:
        """返回当前发布帧应输出的位置；尚未设置目标时返回 None。"""
        if self._start is None or self._target is None:
            return None
        if len(self._start) != len(self._target):
            return list(self._target)
        self._frame += 1
        alpha = self._frame / self._total
        if alpha > 1.0:
            alpha = 1.0
        out = [s + (t - s) * alpha for s, t in zip(self._start, self._target)]
        self._published = out
        return out

    def mark_published(self, positions: list[float]) -> None:
        """零阶路径（如 TYPE_PREPARE）下同步基线，保证下一段插值从实际发布位置起步。"""
        self._published = list(positions)

    def reset(self) -> None:
        """清空插值状态（模型切换/停止等场景可调用）。"""
        self._start = None
        self._target = None
        self._frame = 0
        self._published = None


class RobotRlManagerNode(Node):
    """管理 RL 模型元数据，并周期发布 policy backend 的期望关节命令。"""

    def __init__(self) -> None:
        super().__init__("robot_rl_manager_node")

        # 声明 ROS 参数。路径为空时由各加载函数从工作区根目录向上查找 config/。
        self._robot_joints_config = self.declare_parameter("robot_joints_config", "").value
        self._rl_models_config = self.declare_parameter("rl_models_config", "").value
        self._prepare_profiles_config = self.declare_parameter("prepare_profiles_config", "").value
        self._default_prepare_profile = str(self.declare_parameter("default_prepare_profile", "stand").value)
        self._runtime_platform = str(self.declare_parameter("runtime_platform", "").value)
        self._config_root = str(self.declare_parameter("config_root", "").value)
        self._samples_root = str(self.declare_parameter("samples_root", "").value)
        self._enable_model_input_dump = bool(self.declare_parameter("enable_model_input_dump", False).value)
        self._model_input_dump_mode = str(self.declare_parameter("model_input_dump_mode", "always").value).strip().lower()
        self._model_input_dump_max_seconds = float(self.declare_parameter("model_input_dump_max_seconds", 0.0).value)
        self._model_input_dump_max_frames = int(self.declare_parameter("model_input_dump_max_frames", 0).value)
        self._model_input_dump_overwrite_latest = bool(self.declare_parameter("model_input_dump_overwrite_latest", True).value)
        self._inference_frequency_hz = float(self.declare_parameter("inference_frequency_hz", 50.0).value)
        self._command_publish_frequency_hz = float(self.declare_parameter("command_publish_frequency_hz", 200.0).value)
        self._status_publish_frequency_hz = float(self.declare_parameter("status_publish_frequency_hz", 1.0).value)
        # A1：命令发布插值（消除 50Hz→200Hz 零阶保持位置阶跃）。默认开启；
        # command_interpolation_frames<=0 时按 发布频率/推理频率 自动取整（200/50=4）。
        self._command_interpolation_enabled = bool(self.declare_parameter("enable_command_interpolation", True).value)
        self._command_interpolation_frames = int(self.declare_parameter("command_interpolation_frames", 0).value)
        self._onnx_runtime_options = {
            "intra_op_num_threads": int(self.declare_parameter("onnx_intra_op_num_threads", 2).value),
            "inter_op_num_threads": int(self.declare_parameter("onnx_inter_op_num_threads", 1).value),
            "execution_mode": str(self.declare_parameter("onnx_execution_mode", "sequential").value),
        }
        self._torch_runtime_options = {
            "intra_op_num_threads": int(self.declare_parameter("torch_intra_op_num_threads", 2).value),
            "inter_op_num_threads": int(self.declare_parameter("torch_inter_op_num_threads", 1).value),
        }
        self._rknn_runtime_options = {
            "core_mask": int(self.declare_parameter("rknn_core_mask", 7).value),
        }
        self._default_model_name = str(self.declare_parameter("default_model_name", "walk").value)
        self._robot_type = str(self.declare_parameter("robot_type", "humanoid").value)
        self._nav_all_timeout_sec = float(self.declare_parameter("nav_all_timeout_sec", 0.5).value)
        self._cmd_vel_timeout_sec = float(self.declare_parameter("cmd_vel_timeout_sec", 0.3).value)
        self._nav_mode = str(self.declare_parameter("default_nav_mode", "manual").value)
        self._follow_control_name = str(self.declare_parameter("follow_control_name", "human_follow/follow_control").value)
        self._action_audio_config = str(self.declare_parameter("action_audio_config", "").value)

        # 加载关节逻辑配置和模型元数据。这里不读取 hw_encos.yaml，也不访问真实硬件。
        self._joint_configs = load_joint_configs(str(self._robot_joints_config))
        self._model_registry = ModelRegistry(
            str(self._rl_models_config),
            default_model_name=self._default_model_name,
            prepare_profiles_config=str(self._prepare_profiles_config),
            robot_type=self._robot_type,
            runtime_platform=self._runtime_platform,
        )
        # 启动阶段校验模型元数据和 joint_profiles，最终关节映射由 adapter/builder 兜底。
        self._model_registry.validate_active_model(
            len(self._joint_configs),
            [joint.name for joint in self._joint_configs],
        )
        # PolicyRuntime 统一编排 backend、adapter 和 JointCommands 构造。
        # backend 可替换为 mock/ONNX/Torch，adapter 负责 walk/stand 等模型语义。
        self._policy_runtime = self._create_policy_runtime(self._model_registry.active_model)
        self._policy_runtime.load(self._model_registry.active_model)
        self._model_loaded = self._policy_runtime.is_ready()
        self._requested_model = ""
        self._active_sequence_name = ""
        self._active_sequence_models: tuple[str, ...] = ()
        self._active_sequence_index = 0
        self._active_sequence_return_home = True
        self._inference_error = False
        self._last_error_message = ""
        self._last_inference_latency_ms = 0.0
        self._latest_commands: Optional[JointCommands] = None
        self._command_sequence = 0
        self._command_trace_id = 0
        self._run_profile_first_frame_published = False
        self._prepare_frame_published = False
        self._active_prepare_profile_name = self._default_prepare_profile
        self._last_state_value = RobotState.STATE_BOOTING
        self._latest_joint_states: Optional[JointStates] = None
        self._latest_policy_imu: Optional[PolicyImu] = None
        self._last_policy_imu_time_sec: Optional[float] = None
        self._latest_depth: Optional[np.ndarray] = None
        self._latest_target_point: Optional[Point] = None
        self._latest_target_destination: Optional[Point] = None
        self._latest_intent = RobotIntent()
        self._latest_cmd_vel = Twist()
        self._latest_external_cmd_vel = Twist()
        self._external_cmd_vel_time_sec: Optional[float] = None
        self._stop_active = False
        self._smoother = OutputSmoother()
        # A1：命令发布插值器（线性插值 50Hz 目标到 200Hz 发布周期）。
        if self._command_interpolation_frames > 0:
            _interp_frames = self._command_interpolation_frames
        else:
            _interp_frames = int(round(self._command_publish_frequency_hz / max(self._inference_frequency_hz, 1.0)))
        self._command_interpolator = CommandInterpolator(max(1, _interp_frames))
        self._interp_publish_msg: Optional[JointCommands] = None
        self._latest_state = RobotState()
        self._latest_state.state = RobotState.STATE_BOOTING
        self._action_audio_enabled = False
        self._action_audio_bindings: dict[str, list[dict[str, object]]] = {}
        self._action_audio_timers = []
        self._model_input_recorder = LatestReplaySequenceRecorder(
            samples_root=self._resolve_samples_root(),
            mode=self._model_input_dump_mode,
            max_seconds=self._model_input_dump_max_seconds,
            max_frames=self._model_input_dump_max_frames,
            overwrite_latest=self._model_input_dump_overwrite_latest,
        )

        # 桥接工具，用于 depth Image → numpy 转换。
        self._bridge = CvBridge()

        # 高频控制 topic 的 QoS：KeepLast + depth=1 + VOLATILE + BEST_EFFORT。
        _high_freq_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        _cmd_vel_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        # 订阅底层关节反馈，供 policy 形成观测输入。
        self._joint_states_sub = self.create_subscription(JointStates, "control/hardware_states", self._handle_joint_states, _high_freq_qos)
        # 订阅全局导航指令，供 policy 形成观测输入。
        self._nav_all_sub = self.create_subscription(NavAll, "imu/nav_all", self._handle_nav_all, 10)
        # 订阅第三方导航/遥控器发送的 cmd_vel 速度命令，在 auto 模式下用作模型速度输入。
        self._cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self._handle_external_cmd_vel, _cmd_vel_qos)
        # 订阅 command_router 当前激活意图。注意此处按用户要求使用该 topic 名称。
        self._active_intent_sub = self.create_subscription(RobotIntent, "router/active_intent", self._handle_active_intent, 10)
        # 订阅系统状态。READY/DAMP/RL_STANDBY/RL_ACTIVE 分别对应 prepare/damping/run 首帧/模型推理输出，state_manager 是唯一状态权威。
        self._state_sub = self.create_subscription(RobotState, "state", self._handle_robot_state, 10)
        # 订阅深度相机深度图，作为模型观测输入之一。
        self._depth_sub = self.create_subscription(Image, "camera/depth/image_rect_raw", self._handle_depth_image, 10)
        # 订阅踢球感知：球与球门在基座系下的位置（米）。
        self._target_point_sub = self.create_subscription(Point, "perception/target_point", self._handle_target_point, _high_freq_qos)
        self._target_destination_sub = self.create_subscription(Point, "perception/target_destination", self._handle_target_destination, _high_freq_qos)
        # 发布统一音频请求，由 robot_audio 解析资源路径并处理队列/中断。
        self._audio = AudioClient(self)
        # 只发布 RL 候选命令，不直接发布 /control/hardware_commands。
        self._commands_pub = self.create_publisher(JointCommands, "rl_manager/desired_commands", _high_freq_qos)
        # 周期发布 RL 管理状态，供 state_manager 判断模型是否 ready。
        self._status_pub = self.create_publisher(RlManagerStatus, "rl_manager/status", 10)
        # 模型加载服务只管理元数据，不加载真实推理引擎。
        self._load_model_srv = self.create_service(LoadRlModel, "rl_manager/load_model", self._handle_load_model)
        # 切换模型服务，供 state_manager 调用。
        self._switch_model_srv = self.create_service(SwitchRlModel, "rl_manager/switch_model", self._handle_switch_model)
        # 查询当前平台实际注册的模型列表。
        self._list_models_srv = self.create_service(ListRlModels, "rl_manager/list_models", self._handle_list_models)
        # 跟随控制话题。进入 follow 时 ENTER_TRACK_MODE，离开 follow 时 EXIT_TRACK_MODE。
        self._follow_control_pub = self.create_publisher(FollowControl, self._follow_control_name, 10)
        # 订阅结束 loop 通知，供 policy adapter 调用。
        self._finish_loop_sub = self.create_subscription(Empty, "rl_manager/finish_loop", self._handle_finish_loop, 10)
        # 订阅 motion_stop 通知，切回默认运动模型并强制零速度，或恢复。
        self._motion_stop_sub = self.create_subscription(String, "rl_manager/motion_stop", self._handle_motion_stop, 10)

        # 三个周期分离：推理 50Hz 左右、命令发布 200Hz 左右、状态发布 1Hz 左右。
        # 使用独立的 MutuallyExclusiveCallbackGroup 确保各定时器互不阻塞。
        # 200Hz 高频发布组 — 独占线程，不被推理/订阅阻塞。
        self._inference_timer = self.create_timer(1.0 / max(self._inference_frequency_hz, 1.0), self._handle_inference_timer)
        self._command_publish_timer = self.create_timer(1.0 / max(self._command_publish_frequency_hz, 1.0), self._handle_command_publish_timer)
        self._status_timer = self.create_timer(1.0 / max(self._status_publish_frequency_hz, 0.1), self._handle_status_timer)
        self._load_action_audio_config(self._action_audio_config)

    def _handle_robot_state(self, msg: RobotState) -> None:
        """缓存 state_manager 发布的系统状态。"""
        if msg.state != self._last_state_value:
            # 进入 DO_READY 时，根据当前起身配置同步目标模型。
            if msg.state == RobotState.STATE_DO_READY:
                self._sync_model_for_prepare_profile()
            # 从 RL_ACTIVE 进入其他状态时，重置运行配置
            if msg.state != RobotState.STATE_RL_ACTIVE:
                self._run_profile_first_frame_published = False
            # 从 DO_READY 进入其他状态时，重置准备配置
            if msg.state != RobotState.STATE_DO_READY:
                self._prepare_frame_published = False
            # 进入 IDLE 时自动重置为默认运动模型，防止一次性动作模型残留。
            if msg.state == RobotState.STATE_IDLE and self._model_registry.active_model.name != self._default_model_name:
                self.get_logger().info(f"entered IDLE, resetting model to default: {self._default_model_name}")
                self._switch_to_model(self._default_model_name)
            self._last_state_value = msg.state
        self._latest_state = msg

    def _target_model_for_prepare_profile(self) -> str:
        """由当前起身配置派生准备阶段应使用的模型。"""
        return _GETUP_MODEL_MAP.get(
            self._active_prepare_profile_name,
            self._default_model_name,
        )

    def _sync_model_for_prepare_profile(self) -> bool:
        """确保当前模型与起身配置一致。成功切换返回 True。"""
        target_model = self._target_model_for_prepare_profile()
        if self._model_registry.active_model.name == target_model:
            return False
        self.get_logger().info(
            f"prepare profile '{self._active_prepare_profile_name}' "
            f"selecting model '{target_model}'"
        )
        if self._switch_to_model(target_model):
            self._smoother = OutputSmoother()
            self._publish_status_now()
            return True
        return False

    def _handle_joint_states(self, msg: JointStates) -> None:
        """缓存最近一帧关节状态。"""
        self._latest_joint_states = msg

    def _handle_external_cmd_vel(self, msg: Twist) -> None:
        """缓存第三方 /cmd_vel 话题的 Twist 消息及时间戳。"""
        self._latest_external_cmd_vel = msg
        self._external_cmd_vel_time_sec = self.get_clock().now().nanoseconds * 1e-9

    def _handle_nav_all(self, msg) -> None:
        """缓存厂家 NavAll 转换后的 policy IMU 语义输入。"""
        self._latest_policy_imu = PolicyImu(
            euler_rad=[
                math.radians(float(msg.euler.roll)),
                math.radians(float(msg.euler.pitch)),
                math.radians(float(msg.euler.yaw)),
            ],
            orientation_quat=[
                float(msg.quat.q0),
                float(msg.quat.q1),
                float(msg.quat.q2),
                float(msg.quat.q3),
            ],
            angular_velocity=[
                math.radians(float(msg.gyro.x)),
                math.radians(float(msg.gyro.y)),
                math.radians(float(msg.gyro.z)),
            ],
            linear_velocity=[
                float(msg.vel.vel_e),
                float(msg.vel.vel_n),
                float(msg.vel.vel_u),
            ],
        )
        self._last_policy_imu_time_sec = self.get_clock().now().nanoseconds * 1e-9

    def _handle_depth_image(self, msg: Image) -> None:
        """缓存预处理后的深度图（归一化至 [0,1] 的 float32 数组）。"""
        try:
            depth_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            # depth_normalized = np.clip(depth_image.astype(np.float32) / 1000.0 / 3.0, 0, 1)
            # self._latest_depth = depth_normalized
            self._latest_depth = depth_image
        except Exception as exc:
            self.get_logger().warn(f"failed to process depth image: {exc}")

    def _handle_target_point(self, msg: Point) -> None:
        """缓存球在基座系下的位置（米），供踢球模型观测输入。"""
        self._latest_target_point = msg

    def _handle_target_destination(self, msg: Point) -> None:
        """缓存球门在基座系下的位置（米），供踢球模型观测输入。"""
        self._latest_target_destination = msg

    def _handle_active_intent(self, msg: RobotIntent) -> None:
        """缓存 command_router 选出的当前激活意图，提取速度命令供模型运控。"""
        self._latest_intent = msg

        self._announce_config_intent(msg)
        self._handle_getup_switch_intent(msg)
        self._handle_nav_switch_intent(msg)
        self._handle_cmd_vel_intent(msg)
        self._handle_model_switch_intent(msg)
        self._handle_finish_loop_intent(msg)
        self._handle_motion_stop_intent(msg)
    
    def _announce_config_intent(self, msg: RobotIntent) -> None:
        """根据 config 意图的 command + action 播报对应语音提示。"""
        if msg.intent_type != "config":
            return
        text = _CONFIG_VOICE_MAP.get((msg.command.strip(), msg.action.strip()))
        if text is None:
            return
        self.get_logger().info(f"config intent announced: {text}")
        self._audio.play_tip(text, priority=100)

    def _handle_getup_switch_intent(self, msg: RobotIntent) -> None:
        """config/getup 只更新当前准备姿态选择，不直接决定是否发布。"""
        if msg.intent_type != "config" or msg.command != "getup":
            return
        action = msg.action.strip()
        if action in {"", "getup"}:
            return
        if action not in self._model_registry.active_model.prepare_profiles:
            self._inference_error = True
            self._last_error_message = f"unknown prepare profile: {action}"
            self.get_logger().warn(self._last_error_message)
            return
        if action != self._active_prepare_profile_name:
            self.get_logger().info(f"getup switched to: {action}")
        else:
            self.get_logger().info(f"getup selected: {action}")
        self._active_prepare_profile_name = action
        self._inference_error = False
        self._last_error_message = ""
        if self._latest_state.state == RobotState.STATE_DO_READY:
            self._sync_model_for_prepare_profile()

    def _handle_nav_switch_intent(self, msg: RobotIntent) -> None:
        """处理 config/nav 触发式模式切换。"""
        if msg.intent_type != "config" or msg.command != "nav":
            return
        action = msg.action.strip()
        if action not in ("auto", "manual", "follow"):
            return
        previous_mode = self._nav_mode
        if action == previous_mode:
            return
        self._nav_mode = action
        self.get_logger().info(f"nav mode switched to: {self._nav_mode}")
        if action == "follow":
            self._publish_follow_control_command(FollowControl.ENTER_TRACK_MODE)
        elif previous_mode == "follow":
            self._publish_follow_control_command(FollowControl.EXIT_TRACK_MODE)

    def _publish_follow_control_command(self, command: int) -> None:
        """发布跟随控制命令，避免跨 DDS domain 桥接 service。"""
        msg = FollowControl()
        msg.command = int(command)
        self._follow_control_pub.publish(msg)

    def _handle_cmd_vel_intent(self, msg: RobotIntent) -> None:
        """从非 config/nav 意图中提取速度命令（manual 模式下使用此值）。"""
        self._latest_cmd_vel = Twist(
            linear=Vector3(x=msg.linear_x_mps, y=msg.linear_y_mps),
            angular=Vector3(z=msg.angular_z_radps),
        )

    def _handle_model_switch_intent(self, msg: RobotIntent) -> None:
        """从 RobotIntent 中提取 model 切换指令。"""
        if msg.intent_type != "model" or msg.command != "switch_model":
            return
        model_name = msg.action.strip()
        if not model_name:
            return
        if model_name == self._model_registry.active_model.name:
            return
        if self._stop_active:
            return
        if self._latest_state.state != RobotState.STATE_RL_ACTIVE:
            return
        rules = self._model_registry.switch_rules
        default_model = self._default_model_name

        if model_name in rules.sequences:
            if self._model_registry.active_model.name != default_model:
                return
            self._start_model_sequence(model_name)
            return

        if model_name in rules.pair_map:
            required = rules.pair_map[model_name]
            if self._model_registry.active_model.name != required:
                return
        elif model_name != default_model and self._model_registry.active_model.name != default_model:
            return
        self._clear_model_sequence()
        self.get_logger().info(f"intent: switching to model {model_name}")
        self._switch_to_model(model_name)

    def _start_model_sequence(self, sequence_name: str) -> None:
        """启动命名模型序列，先切到第一段模型。"""
        sequence = self._model_registry.switch_rules.sequences[sequence_name]
        first_model = sequence.models[0]
        self.get_logger().info(
            f"intent: starting model sequence {sequence_name} with {first_model}"
        )
        if self._switch_to_model(first_model):
            self._active_sequence_name = sequence_name
            self._active_sequence_models = sequence.models
            self._active_sequence_index = 0
            self._active_sequence_return_home = sequence.return_home

    def _clear_model_sequence(self) -> None:
        """清除当前模型序列编排状态。"""
        self._active_sequence_name = ""
        self._active_sequence_models = ()
        self._active_sequence_index = 0
        self._active_sequence_return_home = True

    def _handle_finish_loop_intent(self, msg: RobotIntent) -> None:
        """处理 model/finish_loop 意图，通知当前 adapter 结束运动循环。"""
        if msg.intent_type != "model" or msg.command != "finish_loop":
            return
        self._handle_finish_loop(Empty())

    def _handle_motion_stop_intent(self, msg: RobotIntent) -> None:
        """处理 model/motion_stop 意图，控制运动停止/恢复并播报语音。"""
        if msg.intent_type != "model" or msg.command != "motion_stop":
            return
        action = msg.action.strip().lower()
        if action not in ("stop", "resume"):
            return
        self._handle_motion_stop(String(data=action))
        key = audio_tips.MOTION_STOPPED if action == "stop" else audio_tips.MOTION_RESUMED
        self._audio.play_tip(key, priority=100)

    def _switch_to_model(self, model_name: str) -> bool:
        """执行模型切换的核心逻辑。成功返回 True，失败返回 False。"""
        try:
            previous_model_name = self._model_registry.active_model.name
            # 切换前：保存旧模型最后一帧命令，启动平滑过渡
            if self._latest_commands is not None:
                self._smoother.start_transition(self._latest_commands)

            target_model = self._model_registry.get(model_name)
            self._model_registry.validate_model(target_model.name, len(self._joint_configs), [joint.name for joint in self._joint_configs])
            new_runtime = self._create_policy_runtime(target_model)
            new_runtime.load(target_model)

            if previous_model_name != model_name:
                self._finalize_model_input_sequence(reason="model_switch")

            self._policy_runtime.close()
            self._policy_runtime = new_runtime
            self._model_registry.switch(model_name)
            self._model_loaded = self._policy_runtime.is_ready()
            self._last_policy_imu_time_sec = self.get_clock().now().nanoseconds * 1e-9
            self._inference_error = False
            self._last_error_message = ""
            self._latest_commands = None
            self._run_profile_first_frame_published = False
            self._prepare_frame_published = False
            self._requested_model = ""
            self.get_logger().info(f"switched to model: {model_name}")
            self._schedule_action_audio(model_name)
            return True
        except (KeyError, ValueError, RuntimeError) as exc:
            self._inference_error = True
            self._last_error_message = str(exc)
            self.get_logger().warn(f"failed to switch to model {model_name}: {exc}")
            return False

    def _load_action_audio_config(self, config_path: str) -> None:
        """加载动作音频编排配置。配置为空或文件不存在时保持禁用。"""
        self._action_audio_enabled = False
        self._action_audio_bindings = {}
        if not config_path:
            return
        path = Path(str(config_path)).expanduser()
        if not path.exists():
            self.get_logger().warn(f"action audio config not found: {path}")
            return
        try:
            with path.open("r", encoding="utf-8") as stream:
                root = yaml.safe_load(stream) or {}
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().warn(f"failed to load action audio config {path}: {exc}")
            return

        config = root.get("action_audio", root)
        if not bool(config.get("enabled", True)):
            return
        bindings_data = config.get("bindings", {}) or {}
        bindings: dict[str, list[dict[str, object]]] = {}
        for action_name, entries in bindings_data.items():
            if not isinstance(entries, list):
                entries = [entries]
            parsed_entries = []
            for entry in entries:
                if not isinstance(entry, dict):
                    continue
                file_name = str(entry.get("file", "")).strip()
                if not file_name:
                    continue
                parsed_entries.append(
                    {
                        "file": file_name,
                        "delay_sec": max(0.0, float(entry.get("delay_sec", 0.0))),
                        "priority": int(entry.get("priority", 100)),
                        "interrupt": bool(entry.get("interrupt", False)),
                    }
                )
            if parsed_entries:
                bindings[str(action_name)] = parsed_entries
        self._action_audio_bindings = bindings
        self._action_audio_enabled = bool(bindings)

    def _schedule_action_audio(self, model_name: str) -> None:
        """为已成功切换的动作模型安排预录音频播放。"""
        if not self._action_audio_enabled:
            return
        if not hasattr(self, "_action_audio_timers"):
            self._action_audio_timers = []
        entries = self._action_audio_bindings.get(model_name, [])
        for entry in entries:
            delay_sec = float(entry["delay_sec"])

            def _publish(entry=entry, timer_ref=None):
                self._audio.play_file(
                    str(entry["file"]),
                    priority=int(entry["priority"]),
                    interrupt=bool(entry["interrupt"]),
                )
                timer = timer_ref[0] if timer_ref else None
                if timer is not None:
                    timer.cancel()

            if delay_sec <= 0.0:
                _publish(entry)
                continue

            timer_ref = [None]
            timer_ref[0] = self.create_timer(delay_sec, lambda entry=entry, timer_ref=timer_ref: _publish(entry, timer_ref))
            self._action_audio_timers.append(timer_ref[0])

    @staticmethod
    def _resolve_service_path(path_text: str) -> str:
        """解析 load_model 服务传入的路径，支持相对路径。

        依次尝试：robot_rl_manager share 目录 → 包源码目录 → 当前工作目录。
        """
        path = Path(path_text)
        if path.is_absolute():
            return str(path)
        candidates = []
        if get_package_share_directory is not None:
            try:
                share_dir = Path(get_package_share_directory("robot_rl_manager"))
                candidates.append((share_dir / path).resolve())
            except Exception:
                pass
        try:
            import robot_rl_manager as _pkg
            pkg_dir = Path(_pkg.__file__).resolve().parent
            candidates.append((pkg_dir / path).resolve())
        except Exception:
            pass
        for candidate in candidates:
            if candidate.exists():
                return str(candidate)
        return str(path.resolve())

    def _handle_load_model(
        self,
        request: LoadRlModel.Request,
        response: LoadRlModel.Response,
    ) -> LoadRlModel.Response:
        """处理模型加载服务请求。"""
        try:
            model_path = self._resolve_service_path(request.model_path)
            config_path = self._resolve_service_path(request.config_path)
            adapter_path = self._resolve_service_path(request.adapter_path)
            self._model_registry.add_model(
                request.model_name,
                model_path,
                request.model_type,
                config_path=config_path,
                adapter_path=adapter_path,
            )
            response.success = True
            response.message = f"model loaded: {request.model_name}"
        except (RuntimeError, ValueError, FileNotFoundError) as exc:
            # 服务回调不抛异常，统一通过 response 返回错误说明。
            response.success = False
            response.message = str(exc)
        return response

    def _handle_switch_model(
        self,
        request: SwitchRlModel.Request,
        response: SwitchRlModel.Response,
    ) -> SwitchRlModel.Response:
        """处理模型切换服务请求。"""
        self._requested_model = request.model_name
        success = self._switch_to_model(request.model_name)
        if success:
            response.success = True
            response.message = f"active model: {request.model_name}"
        else:
            response.success = False
            response.message = self._last_error_message
        return response

    def _handle_list_models(
        self,
        request: ListRlModels.Request,
        response: ListRlModels.Response,
    ) -> ListRlModels.Response:
        """返回当前平台实际注册的 RL 模型信息。"""
        del request
        response.success = True
        response.message = "ok"
        response.runtime_platform = self._model_registry.runtime_platform
        response.default_model_name = self._default_model_name
        response.active_model_name = self._model_registry.active_model.name
        response.models = [
            self._build_model_info_msg(self._model_registry.get(model_name))
            for model_name in self._model_registry.names()
        ]
        return response

    @staticmethod
    def _build_model_info_msg(model_info: RegistryRlModelInfo) -> RlModelInfo:
        """把 registry 元数据转换成对外服务消息。"""
        msg = RlModelInfo()
        msg.name = model_info.name
        msg.alias = model_info.alias
        msg.category = model_info.category
        msg.backend = model_info.model_type
        msg.duration_sec = float(model_info.duration_sec)
        msg.has_duration = bool(model_info.has_duration)
        return msg

    def _handle_motion_stop(self, msg: String) -> None:
        """处理 motion_stop 话题，停止或恢复当前模型运动。"""
        cmd = msg.data.strip().lower()
        if cmd == "stop":
            if self._stop_active:
                return
            self._stop_active = True
            default_model = self._default_model_name
            self.get_logger().info(
                f"motion_stop: forcing zero velocity on default model {default_model}"
            )
            # 从一次性动作停止时回到默认模型；已经运行默认模型时不 reload，
            # 避免清空历史观测和重新注入 reset input。
            if self._model_registry.active_model.name != default_model:
                self._switch_to_model(default_model)
        elif cmd == "resume":
            if not self._stop_active:
                return
            self.get_logger().info("motion_stop: resuming normal operation")
            self._stop_active = False
        else:
            self.get_logger().warn(f"motion_stop: unknown command '{cmd}'")

    def _handle_finish_loop(self, msg: Empty) -> None:
        """处理 finish_loop 话题，通知当前 adapter 结束运动循环。"""
        del msg
        self.get_logger().info("finish_loop received, notifying adapter")
        try:
            self._policy_runtime.adapter.finish_loop()
        except Exception as exc:
            self.get_logger().warn(f"finish_loop failed: {exc}")

    def _handle_inference_timer(self) -> None:
        """按模型推理周期生成最新 RL 候选关节命令缓存。"""
        # FAULT 默认不发布命令；只有人工 damp 意图允许生成阻尼命令。
        if not self._base_command_conditions_ok():
            self._latest_commands = None
            return

        profile_commands = self._profile_commands_for_current_state()
        if profile_commands is not None:
            self._set_latest_commands(profile_commands)
            return

        # 非 RL_ACTIVE 时不运行模型推理。
        if not self._should_run_inference():
            self._latest_commands = None
            return
        if not self._policy_runtime.is_ready():
            self._latest_commands = None
            return
        assert self._latest_joint_states is not None

        # policy backend 输出 action 向量，builder 负责转为 JointCommands。
        try:
            start_time = time.perf_counter()
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            commands = self._policy_runtime.infer_commands(
                cmd_vel=self._resolve_cmd_vel(),
                joint_states=self._latest_joint_states,
                policy_imu=self._policy_imu(),
                depth_image=self._latest_depth,
                target_point=self._latest_target_point,
                target_destination=self._latest_target_destination,
                now_sec=now_sec,
                model_input_callback=self._record_model_input_sequence,
            )
            self._last_inference_latency_ms = (time.perf_counter() - start_time) * 1000.0
        except (RuntimeError, ValueError) as exc:
            # adapter/backend 解码错误会被拒绝，避免错误命令进入 command_router。
            self.get_logger().warn(f"policy output rejected: {exc}")
            self._latest_commands = None
            self._inference_error = True
            self._last_error_message = str(exc)
            self._publish_status_now()
            return

        self._set_latest_commands(commands)

        # 一次性模型参考运动播放完毕 → 自动切回默认运动模型。
        if not self._inference_error and self._policy_runtime.is_motion_end():
            self._handle_model_motion_end()

    def _resolve_samples_root(self) -> Path:
        """解析样本根目录。

        优先使用 launch 注入的 ``samples_root`` 参数；若未提供，则回退到仓库根目录
        下的 ``config/data/samples``。
        """
        samples_root_text = str(self._samples_root).strip()
        if samples_root_text:
            return Path(samples_root_text).expanduser().resolve()

        config_root_text = str(self._config_root).strip()
        if config_root_text:
            return (Path(config_root_text).expanduser().resolve() / "data" / "samples").resolve()

        repo_root = find_repo_root(Path(__file__))
        return (repo_root / "config" / "data" / "samples").resolve()

    def _record_model_input_sequence(self, model_info: RegistryRlModelInfo, model_input, now_sec: float) -> None:
        """按模型名把当前推理输入追加到序列录制缓冲区。

        Args:
            model_info: 当前活动模型元数据，用于生成模型目录名。
            model_input: adapter 组装好的模型输入字典。
            now_sec: 当前推理时间戳，单位秒。
        """
        if not self._enable_model_input_dump:
            return
        try:
            sample_path = self._model_input_recorder.append(model_info, model_input, now_sec)
        except Exception as exc:
            self.get_logger().warn(f"failed to record model input sequence for {model_info.name}: {exc}")
            return

        if sample_path is not None:
            self.get_logger().info(f"model input sequence dumped: {sample_path}")

    def _finalize_model_input_sequence(self, reason: str) -> None:
        """收口当前模型的序列录制并清空状态。"""
        if not self._enable_model_input_dump:
            return
        try:
            sample_path = self._model_input_recorder.finalize(reason=reason)
            self._model_input_recorder.reset()
        except Exception as exc:
            self.get_logger().warn(f"failed to finalize model input sequence: {exc}")
            return
        if sample_path is not None:
            self.get_logger().info(f"model input sequence finalized ({reason}): {sample_path}")

    def _handle_model_motion_end(self) -> None:
        """处理当前模型 motion_end，优先推进 active sequence，否则回默认模型。"""
        if self._active_sequence_name and self._active_sequence_models:
            next_index = self._active_sequence_index + 1
            if next_index < len(self._active_sequence_models):
                next_model = self._active_sequence_models[next_index]
                self.get_logger().info(
                    f"sequence {self._active_sequence_name}: "
                    f"switching to next model {next_model}"
                )
                if self._switch_to_model(next_model):
                    self._active_sequence_index = next_index
                return

            sequence_name = self._active_sequence_name
            return_home = self._active_sequence_return_home
            self._clear_model_sequence()
            if return_home and self._model_registry.active_model.name != self._default_model_name:
                self.get_logger().info(
                    f"sequence {sequence_name} ended, auto-switching to {self._default_model_name}"
                )
                self._switch_to_model(self._default_model_name)
            return

        default_model = self._default_model_name
        if self._model_registry.active_model.name != default_model:
            self.get_logger().info(
                f"model {self._model_registry.active_model.name} motion ended, "
                f"auto-switching to {default_model}"
            )
            self._switch_to_model(default_model)

    def _handle_command_publish_timer(self) -> None:
        """按指令发布周期发布最近一次有效 RL 候选命令。

        A1：TYPE_RUN 命令在发布周期内做线性插值（消除 50Hz 零阶保持阶跃）；
        其余类型（如 TYPE_PREPARE）保持原有零阶发布语义，由 robot_control 侧
        的准备插值负责平滑。
        """
        if not self._should_publish_commands() or self._latest_commands is None:
            return
        self._command_sequence = int(self._command_sequence) + 1

        interpolate = (
            self._command_interpolation_enabled
            and self._interp_publish_msg is not None
            and self._latest_commands.type == JointCommands.TYPE_RUN
        )
        if interpolate:
            positions = self._command_interpolator.next_positions()
            if positions is None:
                interpolate = False
            else:
                for i, cmd in enumerate(self._interp_publish_msg.commands):
                    if i < len(positions):
                        cmd.position = positions[i]

        if interpolate:
            out = self._interp_publish_msg
        else:
            out = self._latest_commands
            # 零阶路径同步插值基线，保证下一段插值从实际发布位置起步。
            if self._command_interpolation_enabled:
                self._command_interpolator.mark_published([float(c.position) for c in out.commands])
        out.sequence = self._command_sequence
        out.header.stamp = self.get_clock().now().to_msg()
        self._commands_pub.publish(out)

        # 判断命令是否只允许发布一次
        if self._latest_commands.type == JointCommands.TYPE_PREPARE:
            self._latest_commands = None
            self._interp_publish_msg = None

    def _handle_status_timer(self) -> None:
        """按低频周期发布 RL 管理状态。"""
        self._publish_status_now()

    def _publish_status_now(self) -> None:
        """立即发布一次 RL 管理状态，用于推理错误等事件快速通知 state_manager。"""
        status = self._make_status_msg()
        status.header.stamp = self.get_clock().now().to_msg()
        self._status_pub.publish(status)

    def _should_run_inference(self) -> bool:
        """判断当前是否允许运行模型推理。"""
        if not self._base_command_conditions_ok():
            return False
        if self._latest_state.state != RobotState.STATE_RL_ACTIVE:
            return False
        if self._missing_policy_inputs():
            return False
        return True

    def _should_publish_commands(self) -> bool:
        """判断当前是否允许发布 RL 候选运动命令。"""
        if self._latest_state.state == RobotState.STATE_RL_ACTIVE and self._missing_policy_inputs():
            return False
        base_ok = self._base_command_conditions_ok()
        no_error = not self._inference_error
        if not base_ok or not no_error:
            return False
        return True

    def _base_command_conditions_ok(self) -> bool:
        """判断 RL manager 是否可发布任意候选关节命令。"""
        if self._latest_joint_states is None:
            return False
        if self._fault_manual_damp_requested():
            return True
        if self._latest_state.fault_active:
            return False
        if self._latest_state.state in {
            RobotState.STATE_FAULT,
        }:
            return False
        if self._latest_state.state not in {
            RobotState.STATE_DO_READY,
            RobotState.STATE_READY,
            RobotState.STATE_DAMP,
            RobotState.STATE_IDLE,
            RobotState.STATE_RL_STANDBY,
            RobotState.STATE_RL_ACTIVE,
        }:
            return False
        return True

    def _profile_commands_for_current_state(self) -> Optional[JointCommands]:
        """根据当前状态/意图生成无需推理的 profile 命令。"""
        try:
            if self._fault_manual_damp_requested():
                return self._policy_runtime.build_damping_commands()
            # IDLE 状态下，只发布 zero moment 命令。
            if self._latest_state.state == RobotState.STATE_IDLE:
                return self._policy_runtime.build_zero_moment_commands()
            # DAMP 状态下，只发布 damping 命令。
            if self._latest_state.state == RobotState.STATE_DAMP:
                return self._policy_runtime.build_damping_commands()
            # DO_READY 状态下，只发布 prepare 命令。
            if self._latest_state.state == RobotState.STATE_DO_READY:
                if not self._prepare_frame_published:
                    self._prepare_frame_published = True
                    return self._policy_runtime.build_prepare_commands(self._active_prepare_profile_name)
                return self._policy_runtime.build_prepare_commands(self._active_prepare_profile_name, command_type=JointCommands.TYPE_RUN)
            # READY 状态下，只发布 run 命令。
            if self._latest_state.state == RobotState.STATE_READY:
                return self._policy_runtime.build_prepare_commands(self._active_prepare_profile_name, command_type=JointCommands.TYPE_RUN)
            # RL_STANDBY 状态下，只发布 run 首帧命令。
            if self._latest_state.state == RobotState.STATE_RL_STANDBY:
                self._run_profile_first_frame_published = True
                return self._policy_runtime.build_model_profile_commands("run")
            # RL_ACTIVE 状态下，只发布 run 首帧命令。
            if (self._latest_state.state == RobotState.STATE_RL_ACTIVE and not self._run_profile_first_frame_published):
                self._run_profile_first_frame_published = True
                return self._policy_runtime.build_model_profile_commands("run")
        except (RuntimeError, ValueError) as exc:
            self.get_logger().warn(f"profile command rejected: {exc}")
            self._inference_error = True
            self._last_error_message = str(exc)
            return None
        return None

    def _fault_manual_damp_requested(self) -> bool:
        """判断当前是否请求手动 damp 命令。"""
        return (
            self._latest_state.state == RobotState.STATE_FAULT
            and self._latest_state.fault_active
            and self._latest_intent.intent_type == "status"
            and self._latest_intent.command == "damp"
        )

    def _set_latest_commands(self, commands: JointCommands) -> None:
        """统一填充时间戳、超时和错误状态后缓存候选命令。"""
        self._inference_error = False
        self._last_error_message = ""
        self._command_trace_id += 1
        commands.trace_id = self._command_trace_id
        commands.parent_sequence = 0
        commands.parent_source = ""
        commands.stage = "desired"
        commands.header.stamp = self.get_clock().now().to_msg()
        commands.ankle_in_series = self._model_registry.active_model.ankle_in_series
        self._smoother.filter(commands)
        self._latest_commands = commands
        # A1：为新目标准备发布副本并规划插值段（只对 TYPE_RUN 生效，见发布定时器）。
        if self._command_interpolation_enabled:
            self._interp_publish_msg = copy.deepcopy(commands)
            self._command_interpolator.set_target([float(c.position) for c in commands.commands])

    def _make_status_msg(self) -> RlManagerStatus:
        """构造 RlManagerStatus，便于单元测试和节点发布复用。"""
        active_model = self._model_registry.active_model
        status = RlManagerStatus()
        status.online = True
        status.model_loaded = self._model_loaded
        missing_inputs = sorted(self._missing_policy_inputs())
        status.model_ready = self._policy_runtime.is_ready()
        status.inference_active = self._should_run_inference() and not self._inference_error
        status.inference_error = self._inference_error
        status.active_model = active_model.name
        status.requested_model = self._requested_model
        status.model_type = active_model.model_type
        status.policy_backend = active_model.policy_backend
        status.ankle_in_series = active_model.ankle_in_series
        status.inference_frequency_hz = float(self._inference_frequency_hz)
        status.command_publish_frequency_hz = float(self._command_publish_frequency_hz)
        status.status_publish_frequency_hz = float(self._status_publish_frequency_hz)
        status.last_inference_latency_ms = float(self._last_inference_latency_ms)
        status.imu_ready = self._is_policy_imu_fresh()
        imu_age = self._policy_imu_age_sec()
        status.imu_age_sec = float(imu_age) if imu_age is not None else -1.0
        status.missing_inputs = missing_inputs
        status.error_message = self._last_error_message
        return status

    def _create_policy_runtime(self, model_info: RegistryRlModelInfo) -> PolicyRuntime:
        """根据模型配置创建 backend、adapter 和 runtime。"""
        adapter = create_policy_adapter(model_info.policy_adapter)
        backend = create_policy_backend(model_info.policy_backend)
        self._configure_policy_backend_runtime_options(model_info, backend, adapter)
        return PolicyRuntime(self._joint_configs, backend, adapter)

    def _configure_policy_backend_runtime_options(self, model_info, backend, adapter) -> None:
        """把全局 runtime 默认值和 adapter 覆盖传给支持该接口的 backend。

        按模型类型选择对应后端配置：onnx/pt/rknn 分别使用独立参数。
        """
        configure = getattr(backend, "configure_runtime_options", None)
        if configure is None:
            return

        model_type = str(getattr(model_info, "model_type", "")).lower()
        if model_type == "pt":
            global_options = dict(self._torch_runtime_options)
            backend_name = "torch"
        elif model_type == "rknn":
            global_options = dict(self._rknn_runtime_options)
            backend_name = "rknn"
        else:
            global_options = dict(self._onnx_runtime_options)
            backend_name = "onnx"
        adapter_options = self._adapter_onnx_runtime_options(adapter) if model_type != "rknn" else {}
        if adapter_options:
            global_options.update(adapter_options)
        configure(global_options)

        source = "adapter" if adapter_options else "global"
        if model_type == "rknn":
            self.get_logger().info(
                f"{backend_name} runtime options for "
                f"{model_info.name}: core_mask={global_options.get('core_mask')} source={source}"
            )
        else:
            self.get_logger().info(
                f"{backend_name} runtime options for "
                f"{model_info.name}: intra_op={global_options.get('intra_op_num_threads')} "
                f"inter_op={global_options.get('inter_op_num_threads')} "
                f"execution_mode={global_options.get('execution_mode')} source={source}"
            )

    @staticmethod
    def _adapter_onnx_runtime_options(adapter) -> dict:
        """读取 adapter 文件中可选的 ONNX_RUNTIME_OPTIONS 覆盖。"""
        options = getattr(adapter, "ONNX_RUNTIME_OPTIONS", None)
        if options is None:
            options = getattr(type(adapter), "ONNX_RUNTIME_OPTIONS", None)
        if options is None:
            module = importlib.import_module(type(adapter).__module__)
            options = getattr(module, "ONNX_RUNTIME_OPTIONS", None)
        if options is None:
            return {}
        if not isinstance(options, dict):
            raise ValueError("ONNX_RUNTIME_OPTIONS must be a dict")
        return {str(key): value for key, value in options.items()}

    def _is_external_cmd_vel_fresh(self) -> bool:
        """判断外部 /cmd_vel 是否在超时阈值内。"""
        if self._external_cmd_vel_time_sec is None:
            return False
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        return (now_sec - self._external_cmd_vel_time_sec) <= self._cmd_vel_timeout_sec

    @staticmethod
    def _clamp_cmd_vel(cmd: Twist) -> Twist:
        """将速度指令钳位到行走模型安全范围内，防止外部指令超限。"""
        cmd.linear.x = max(-1.5, min(5.0, cmd.linear.x))
        cmd.linear.y = max(-1.0, min(1.0, cmd.linear.y))
        cmd.angular.z = max(-1.5, min(1.5, cmd.angular.z))
        return cmd

    def _resolve_cmd_vel(self) -> Twist:
        """根据当前 nav_mode 选择速度来源，并对结果做模型上限钳位。

        auto/follow 模式使用外部 /cmd_vel（超时则归零），manual 模式使用最新 intent 速度。
        motion_stop 激活时始终返回零速。
        """
        if self._stop_active:
            return Twist()
        if self._nav_mode in ("auto", "follow"):
            if self._is_external_cmd_vel_fresh():
                return self._clamp_cmd_vel(self._latest_external_cmd_vel)
            return Twist()
        return self._clamp_cmd_vel(self._latest_cmd_vel)

    def _policy_imu_age_sec(self) -> Optional[float]:
        """返回最近 policy IMU 语义输入年龄。"""
        if self._last_policy_imu_time_sec is None:
            return None
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        return max(0.0, now_sec - self._last_policy_imu_time_sec)

    def _is_policy_imu_fresh(self) -> bool:
        """判断厂家 NavAll 转换后的 policy IMU 是否新鲜。"""
        age_sec = self._policy_imu_age_sec()
        return self._latest_policy_imu is not None and age_sec is not None and (age_sec <= self._nav_all_timeout_sec)

    def _policy_imu(self) -> Optional[PolicyImu]:
        """返回经过新鲜度检查的 policy IMU 语义输入，过期或不存在时返回 None。"""
        if not self._is_policy_imu_fresh() or self._latest_policy_imu is None:
            return None
        return self._latest_policy_imu

    def _missing_policy_inputs(self) -> set[str]:
        """返回当前 policy adapter 缺失的输入名称。"""
        if self._latest_joint_states is None:
            joint_states = None
        else:
            joint_states = self._latest_joint_states
        context = PolicyContext(
            cmd_vel=self._resolve_cmd_vel(),
            joint_states=joint_states,
            model_info=self._model_registry.active_model,
            joint_configs=list(self._joint_configs),
            policy_imu=self._policy_imu(),
            depth_image=self._latest_depth,
            target_point=self._latest_target_point,
            target_destination=self._latest_target_destination,
        )
        missing = self._policy_runtime.missing_required_inputs(context)
        return missing

    def destroy_node(self) -> bool:
        """销毁节点前先把正在录制的最新序列收口落盘。"""
        try:
            self._finalize_model_input_sequence(reason="shutdown")
        finally:
            return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    """运行 robot_rl_manager_node。"""
    rclpy.init(args=args)
    node = RobotRlManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._policy_runtime.close()
        node.destroy_node()
