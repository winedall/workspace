import os
from dataclasses import dataclass, field

AGENT_ROOT = os.path.expanduser("~/.zl_robot")


def _default_agent_root() -> str:
    configured = os.environ.get("ZL_ROBOT_AGENT_ROOT", AGENT_ROOT)
    return os.path.abspath(os.path.expanduser(configured))


@dataclass
class AgentConfig:
    """OTA Agent 配置。

    字段说明：
      listen_addr / listen_port  Agent HTTP API 监听地址
      agent_root                 zl_robot 安装根目录
      server_url                 OTA Server 地址（Agent 用于心跳/对账）
      device_token               设备身份令牌，用于 Agent API 认证
      device_id                  设备唯一标识
      device_yaml_path           device.yaml 文件路径（自动推导）
      public_key_path            公钥路径，用于验证发布包签名
      healthcheck_timeout        健康检查总体超时（秒）
      healthcheck_interval       健康检查轮询间隔（秒）
      stable_window              连续健康检查成功次数，达到后视为稳定
      max_retain_versions        磁盘保留的最大版本数
    """
    listen_addr: str = "0.0.0.0"
    listen_port: int = 8001
    agent_root: str = field(default_factory=_default_agent_root)
    releases_dir: str = field(init=False)
    shared_dir: str = field(init=False)
    ota_state_dir: str = field(init=False)
    downloads_dir: str = field(init=False)
    logs_dir: str = field(init=False)
    device_yaml_path: str = field(init=False)
    server_url: str = "http://ota-server:8000"
    device_token: str = ""
    device_id: str = ""
    public_key_path: str = os.path.join(
        os.path.dirname(__file__), "..", "scripts", "signing", "public_key.key"
    )
    healthcheck_timeout: int = 60
    healthcheck_interval: float = 2.0
    stable_window: int = 3
    max_retain_versions: int = 3

    def __post_init__(self):
        """根据 agent_root 自动推导子目录路径。"""
        self._derive_paths()
        self._load_device_yaml()

    def _derive_paths(self):
        self.releases_dir = os.path.join(self.agent_root, "releases")
        self.shared_dir = os.path.join(self.agent_root, "shared")
        self.ota_state_dir = os.path.join(self.shared_dir, "ota-state")
        self.downloads_dir = os.path.join(self.shared_dir, "downloads")
        self.logs_dir = os.path.join(self.shared_dir, "logs")
        self.device_yaml_path = os.path.join(self.shared_dir, "device", "device.yaml")

    def _load_device_yaml(self):
        """读取设备身份文件，填充 device_id / device_token / server_url / agent_root。

        文件不存在时静默跳过（测试环境或首次安装未配置时也正常工作）。
        """
        if not os.path.exists(self.device_yaml_path):
            return
        try:
            import yaml
            with open(self.device_yaml_path, "r") as f:
                data = yaml.safe_load(f) or {}
            if "device_id" in data:
                self.device_id = data["device_id"]
            if "device_token" in data:
                self.device_token = data["device_token"]
            if "server_url" in data:
                self.server_url = data["server_url"]
            if "agent_root" in data and "ZL_ROBOT_AGENT_ROOT" not in os.environ:
                self.agent_root = os.path.expanduser(data["agent_root"])
                self._derive_paths()
        except Exception:
            pass
