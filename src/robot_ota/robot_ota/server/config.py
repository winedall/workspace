"""OTA 服务器配置模块。

通过环境变量注入配置，支持 ROS2 参数动态设置。
"""

import os
from pydantic_settings import BaseSettings


def _find_workspace_root() -> str:
    # COLCON_PREFIX_PATH 由 colcon 的 setup.bash 设置，始终可用。
    install_prefix = os.environ.get("COLCON_PREFIX_PATH", "")
    if not install_prefix:
        raise RuntimeError(
            "COLCON_PREFIX_PATH not set — did you source install/setup.bash?"
        )
    # COLCON_PREFIX_PATH 可能是冒号分隔的多路径列表（多次 source 不同 setup.bash 时）。
    # colcon 每次 source 会把新 workspace 的 install 目录前置到列表，因此第一个条目
    # 即为当前 workspace 的 install 目录，必须拆分后取第一个，避免把整串当作单一路径。
    first_prefix = install_prefix.split(os.pathsep)[0]
    return os.path.abspath(os.path.join(first_prefix, ".."))


class Settings(BaseSettings):
    """OTA 服务器配置类。

    所有配置项均可通过环境变量覆盖（名称与字段名一致）。
    """

    server_host: str = "0.0.0.0"  # 服务器监听地址
    server_port: int = 8000  # 服务器监听端口
    data_root: str = "./data"  # 数据存储根目录
    agent_root: str = "~/.zl_robot"  # Agent 端默认安装根目录

    @property
    def releases_dir(self) -> str:
        """发布包存储目录。"""
        return os.path.join(self.data_root, "releases")

    @property
    def db_path(self) -> str:
        """SQLite 数据库文件路径。"""
        return os.path.join(self.data_root, "ota.db")

    @property
    def deploy_packages_dir(self) -> str:
        """部署包临时存储目录。"""
        return os.path.join(self.data_root, "deploy_packages")

    @property
    def workspace_root(self) -> str:
        """获取 ROS2 工作空间根目录。

        从 COLCON_PREFIX_PATH 环境变量获取，该变量由 colcon 的 setup.bash 设置。
        """
        return _find_workspace_root()

    @property
    def public_key_path(self) -> str:
        """签名公钥文件路径。

        位于工作空间 scripts/signing/ 目录下。
        """
        ws = self.workspace_root
        if ws:
            return os.path.join(ws, "scripts", "signing", "public_key.key")
        return ""

    @property
    def agent_wheels_dir(self) -> str:
        """Agent 离线 Python wheel 目录。

        位于工作空间 scripts/wheels/ 目录下。
        """
        ws = self.workspace_root
        if ws:
            return os.path.join(ws, "scripts", "wheels")
        return ""

    @property
    def agent_depends_dir(self) -> str:
        """Agent 离线 deb 包目录。

        位于工作空间 scripts/depends/ 目录下。
        """
        ws = self.workspace_root
        if ws:
            return os.path.join(ws, "scripts", "depends")
        return ""
