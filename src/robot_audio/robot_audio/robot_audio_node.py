#!/usr/bin/env python3

import os
from pathlib import Path
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from robot_interfaces.msg import AudioRequest, SpeakStatus
from robot_audio.sherpa_tts_engine import SherpaOnnxTTSEngine
import heapq
import threading
import time
import yaml


def _get_package_share_dir(package_name: str) -> str:
    try:
        from ament_index_python import get_package_share_directory

        return get_package_share_directory(package_name)
    except Exception:
        return ""


class EmptyQueue(Exception):
    pass


class PriorityQueue:
    def __init__(self, maxsize: int = 0):
        self._heap = []
        self._lock = threading.Lock()
        self._maxsize = maxsize
        self._counter = 0

    def put(self, item: tuple) -> bool:
        with self._lock:
            if self._maxsize > 0 and len(self._heap) >= self._maxsize:
                return False
            priority, data = item
            heapq.heappush(self._heap, (-priority, self._counter, data))
            self._counter += 1
            return True

    def get(self, timeout: float = None) -> tuple:
        with self._lock:
            if not self._heap:
                raise EmptyQueue()
            neg_priority, counter, data = heapq.heappop(self._heap)
            return (-neg_priority, data)

    def size(self) -> int:
        with self._lock:
            return len(self._heap)

    def clear(self):
        with self._lock:
            self._heap.clear()
            self._counter = 0


class FileAudioPlayer:
    def __init__(self, logger=None):
        self.logger = logger
        self._current_process = None
        self._lock = threading.Lock()

    def play_file(self, audio_file: str) -> bool:
        with self._lock:
            try:
                self._current_process = subprocess.Popen(
                    # ["aplay", "-q", audio_file],
                    ["aplay", "-D", "pulse", "-q", audio_file],
                    # ["aplay", "-D", "plughw:1,0", "-q", audio_file],
                    # ["aplay", "-D", "plughw:CARD=Device,DEV=0", "-q", audio_file],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    text=True,
                )
                _, stderr = self._current_process.communicate()
                return_code = self._current_process.returncode
                self._current_process = None
                if return_code == 0:
                    return True
                if self.logger:
                    file_name = Path(audio_file).name
                    detail = (stderr or "").strip()
                    if detail:
                        self.logger.error(f"音频播放失败: file={file_name} returncode={return_code} stderr={detail}")
                    else:
                        self.logger.error(f"音频播放失败: file={file_name} returncode={return_code}")
                return False
            except Exception as e:
                if self.logger:
                    self.logger.error(f"音频播放失败: {e}")
                return False

    def speak(self, text: str) -> bool:
        if self.logger:
            self.logger.error(f"TTS未启用，拒绝文本播报: {text[:30]}...")
        return False

    def stop(self) -> None:
        if self._current_process:
            try:
                self._current_process.terminate()
                self._current_process.wait(timeout=1)
            except Exception:
                pass
            self._current_process = None


class RobotAudioNode(Node):
    STATE_IDLE = 0
    STATE_PLAYING = 1

    def __init__(self):
        super().__init__("robot_audio_node")

        self._load_params()

        self.tips = self._load_tips_manifest(self.tips_manifest)

        if self.tts_enabled:
            try:
                self.audio_engine = self._create_tts_engine()
                self.get_logger().info("TTS引擎初始化成功")
            except Exception as e:
                self.get_logger().error(f"TTS引擎初始化失败: {e}")
                self.audio_engine = None
        else:
            self.audio_engine = FileAudioPlayer(logger=self.get_logger())
            self.get_logger().info("TTS已禁用，仅启用音频文件播放")

        self.play_queue = PriorityQueue(maxsize=self.queue_size)
        self.is_playing = False
        self.current_text = ""
        self.current_state = self.STATE_IDLE
        self._state_lock = threading.Lock()

        self._running = True
        self.play_thread = threading.Thread(target=self._play_worker, daemon=True)
        self.play_thread.start()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )

        self.audio_request_sub = self.create_subscription(
            AudioRequest, self.request_topic, self._handle_audio_request, qos_profile
        )

        self.status_pub = self.create_publisher(
            SpeakStatus, self.status_topic, qos_profile
        )
        self.status_timer = self.create_timer(0.5, self._publish_status)

        self.get_logger().info(f"robot_audio节点已启动")
        self.get_logger().info(f"订阅话题: {self.request_topic}")
        self.get_logger().info(f"状态话题: {self.status_topic}")

    def _load_params(self) -> None:
        self.declare_parameter("tts_rate", 80)
        self.declare_parameter("tts_volume", 1.0)
        self.declare_parameter("queue_size", 10)
        self.declare_parameter("interrupt_enabled", True)
        self.declare_parameter("request_topic", "audio/request")
        self.declare_parameter("status_topic", "audio/status")
        self.declare_parameter("actions_dir", "")
        self.declare_parameter("tips_manifest", "")
        self.declare_parameter("tts_enabled", True)
        self.declare_parameter("sherpa_model_dir", "")
        self.declare_parameter("sherpa_num_threads", 1)
        self.declare_parameter("sherpa_provider", "cpu")
        self.declare_parameter("sherpa_noise_scale", 1.0)
        self.declare_parameter("sherpa_length_scale", 1.0)
        self.declare_parameter("tts_target_peak", 20000)
        self.declare_parameter("tts_max_gain", 4.0)
        self.declare_parameter("tts_cache_enabled", True)
        self.declare_parameter("tts_cache_dir", "")

        self.tts_rate = self.get_parameter("tts_rate").value
        self.tts_volume = self.get_parameter("tts_volume").value
        self.queue_size = self.get_parameter("queue_size").value
        self.interrupt_enabled = self.get_parameter("interrupt_enabled").value
        self.request_topic = self.get_parameter("request_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.actions_dir = self._resolve_config_path(self.get_parameter("actions_dir").value, "config/data/actions")
        self.tips_manifest = self._resolve_config_path(
            self.get_parameter("tips_manifest").value,
            "config/data/tips.yaml",
        )
        self.tts_enabled = bool(self.get_parameter("tts_enabled").value)
        self.sherpa_model_dir = self._resolve_sherpa_model_dir(
            self.get_parameter("sherpa_model_dir").value,
        )
        self.sherpa_num_threads = self.get_parameter("sherpa_num_threads").value
        self.sherpa_provider = self.get_parameter("sherpa_provider").value
        self.sherpa_noise_scale = self.get_parameter("sherpa_noise_scale").value
        self.sherpa_length_scale = self.get_parameter("sherpa_length_scale").value
        self.tts_target_peak = self.get_parameter("tts_target_peak").value
        self.tts_max_gain = self.get_parameter("tts_max_gain").value
        self.tts_cache_enabled = bool(self.get_parameter("tts_cache_enabled").value)
        self.tts_cache_dir = self._resolve_tts_cache_dir(self.get_parameter("tts_cache_dir").value)

    def _create_tts_engine(self):
        return SherpaOnnxTTSEngine(
            model_dir=self.sherpa_model_dir,
            num_threads=self.sherpa_num_threads,
            provider=self.sherpa_provider,
            logger=self.get_logger(),
            cache_enabled=self.tts_cache_enabled,
            cache_dir=self.tts_cache_dir,
            noise_scale=self.sherpa_noise_scale,
            length_scale=self.sherpa_length_scale,
            target_peak=self.tts_target_peak,
            max_gain=self.tts_max_gain,
        )

    def _resolve_config_path(self, raw_path: str, fallback_relative: str) -> str:
        path_text = str(raw_path or "").strip()
        if path_text:
            return str(Path(path_text).expanduser().resolve())

        current = Path.cwd().resolve()
        for candidate_root in [current, *current.parents]:
            candidate = candidate_root / fallback_relative
            if candidate.exists():
                return str(candidate)
        return str((current / fallback_relative).resolve())

    def _resolve_sherpa_model_dir(self, raw_path: str) -> str:
        path_text = str(raw_path or "").strip()
        if path_text:
            return str(Path(path_text).expanduser().resolve())

        share_dir = _get_package_share_dir("robot_audio")
        if share_dir:
            share_candidate = Path(share_dir) / "models/matcha-icefall-zh-baker"
            if share_candidate.exists():
                return str(share_candidate.resolve())

        raise FileNotFoundError(
            "sherpa-onnx模型目录不存在，请设置参数sherpa_model_dir，"
            "或确认已安装share/robot_audio/models/matcha-icefall-zh-baker"
        )

    def _resolve_tts_cache_dir(self, raw_path: str) -> str:
        path_text = str(raw_path or "").strip()
        if path_text:
            return str(Path(path_text).expanduser().resolve())
        return self._resolve_config_path("", "config/data/temps/tts_cache")

    def _load_tips_manifest(self, manifest_path: str) -> dict[str, str]:
        path = Path(manifest_path)
        if not path.exists():
            self.get_logger().warn(f"提示音manifest不存在: {path}")
            return {}
        try:
            data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().warn(f"提示音manifest加载失败 {path}: {exc}")
            return {}

        tips = data.get("tips", {})
        if not isinstance(tips, dict):
            self.get_logger().warn(f"提示音manifest格式错误: {path}")
            return {}
        parsed = {}
        for key, value in tips.items():
            text = value.strip() if isinstance(value, str) else ""
            if text:
                parsed[str(key)] = text
        return parsed

    def _handle_audio_request(self, msg: AudioRequest) -> None:
        name = msg.name.strip() if hasattr(msg, "name") else ""
        priority = msg.priority if hasattr(msg, "priority") else 100
        interrupt = bool(msg.interrupt) if hasattr(msg, "interrupt") else False

        if not name:
            return

        if not self.audio_engine:
            self.get_logger().error("音频引擎未初始化")
            return

        if msg.kind == AudioRequest.KIND_TIP:
            tip_text = self.tips.get(name, "")
            if not tip_text:
                self.get_logger().warn(f"未知提示音key: {name}")
                return
            if not self.tts_enabled:
                self.get_logger().warn(f"TTS未启用，拒绝系统提示播报: {name}")
                return
            self._enqueue({"kind": "text", "value": tip_text}, priority, interrupt=interrupt)
            return

        if msg.kind == AudioRequest.KIND_FILE:
            file_path = self._resolve_action_file(name)
            if not file_path:
                return
            self._enqueue({"kind": "file", "value": file_path}, priority, interrupt=interrupt)
            return

        if msg.kind == AudioRequest.KIND_TEXT:
            if not self.tts_enabled:
                self.get_logger().warn(f"TTS未启用，拒绝文本播报: {name[:30]}...")
                return
            self._enqueue({"kind": "text", "value": name}, priority, interrupt=interrupt)
            return

        self.get_logger().warn(f"未知音频请求类型: {msg.kind}")

    def _resolve_action_file(self, file_name: str) -> str:
        return self._resolve_file(Path(self.actions_dir), file_name, "动作音频文件不存在")

    def _resolve_file(self, base_dir: Path, name: str, missing_prefix: str) -> str:
        path = Path(name).expanduser()
        if not path.is_absolute():
            path = base_dir / path
        path = path.resolve()
        if not path.exists():
            self.get_logger().warn(f"{missing_prefix}: {path}")
            return ""
        if not os.path.isfile(path):
            self.get_logger().warn(f"音频路径不是文件: {path}")
            return ""
        return str(path)

    def _enqueue(self, item, priority: int, interrupt: bool = False) -> bool:
        display_text = item["value"] if isinstance(item, dict) else str(item)
        should_interrupt = interrupt or priority >= 200
        if self.interrupt_enabled and should_interrupt:
            with self._state_lock:
                if self.is_playing:
                    self.audio_engine.stop()
                    self.get_logger().info(f"高优先级播报中断当前: {self._display_item(item)[:50]}...")

        if self.play_queue.put((priority, item)):
            self.get_logger().debug(f"入队成功: {display_text[:30]}... (优先级:{priority})")
            return True
        else:
            self.get_logger().warn(f"队列已满，拒绝: {display_text[:30]}...")
            return False

    def _play_worker(self) -> None:
        while self._running:
            try:
                priority, item = self.play_queue.get(timeout=0.1)
            except EmptyQueue:
                time.sleep(0.05)
                continue
            except Exception as e:
                self.get_logger().error(f"队列获取异常: {e}")
                continue

            if isinstance(item, dict):
                item_kind = item.get("kind", "text")
                item_value = str(item.get("value", ""))
            else:
                item_kind = "text"
                item_value = str(item)

            display_text = self._display_item(item)
            with self._state_lock:
                self.is_playing = True
                self.current_text = item_value
                self.current_state = self.STATE_PLAYING

            self.get_logger().info(f"开始播放: {display_text[:50]}...")

            played = False
            try:
                if self.audio_engine:
                    if item_kind == "file":
                        played = self.audio_engine.play_file(item_value)
                    else:
                        played = self.audio_engine.speak(item_value)
            except Exception as e:
                self.get_logger().error(f"播放异常: {e}")

            with self._state_lock:
                self.is_playing = False
                self.current_text = ""
                self.current_state = self.STATE_IDLE

            if played:
                self.get_logger().info(f"播放完成: {display_text[:50]}...")
            else:
                self.get_logger().warn(f"播放失败: {display_text[:50]}...")

    @staticmethod
    def _display_item(item) -> str:
        if isinstance(item, dict):
            item_kind = item.get("kind", "text")
            item_value = str(item.get("value", ""))
            if item_kind == "file":
                return Path(item_value).name or item_value
            return item_value
        return str(item)

    def _publish_status(self) -> None:
        msg = SpeakStatus()
        with self._state_lock:
            msg.status = self.current_state
            msg.current_text = self.current_text
            msg.queue_size = self.play_queue.size()
        self.status_pub.publish(msg)

    def shutdown(self) -> None:
        self.get_logger().info("正在关闭robot_audio节点...")

        self._running = False

        if self.audio_engine:
            self.audio_engine.stop()

        if self.play_thread.is_alive():
            self.play_thread.join(timeout=2.0)

        try:
            while True:
                self.play_queue.get(timeout=0.01)
        except EmptyQueue:
            pass

        self.get_logger().info("robot_audio节点已关闭")


def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = RobotAudioNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点错误: {e}")
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
