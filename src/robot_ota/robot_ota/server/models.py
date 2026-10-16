from dataclasses import dataclass, field
from typing import Optional


@dataclass
class Device:
    """设备实体。

    对应数据库 devices 表。
    id: 设备唯一标识（如 robot-01）
    token_hash: 设备 Token 的 SHA256 哈希值
    agent_online / robot_online: 不持久化，由时间戳动态计算
    """
    id: str
    name: str = ""
    model: str = ""
    agent_address: str = ""
    token_hash: str = ""
    current_version: str = ""
    robot_state: str = ""
    last_robot_state_at: str = ""
    last_heartbeat: str = ""
    created_at: str = ""
    updated_at: str = ""


@dataclass
class Release:
    """版本发布实体。

    对应数据库 releases 表。
    archive_path/signature_path: 发布包和签名文件在服务端的存储路径
    manifest: manifest.json 的 JSON 字符串内容
    """
    id: Optional[int] = None
    version: str = ""
    product: str = "zl_robot"
    platform: str = "linux-amd64"
    archive_path: str = ""
    archive_hash: str = ""
    signature_path: str = ""
    file_size: int = 0
    manifest: str = "{}"
    release_notes: str = ""
    created_by: str = ""
    created_at: str = ""


@dataclass
class Deployment:
    """设备升级任务实体。

    一行 = 一台设备的一次升级。
    同批次批量操作通过 batch_id 关联。
    """
    id: Optional[int] = None
    batch_id: str = ""
    device_id: str = ""
    release_id: int = 0
    release_version: str = ""
    from_version: str = ""
    status: str = "pending"
    progress: int = 0
    error: str = ""
    created_by: str = ""
    created_at: str = ""
    updated_at: str = ""


@dataclass
class AuditLog:
    """审计日志条目，记录所有敏感操作。"""
    id: Optional[int] = None
    action: str = ""
    actor: str = ""
    target: str = ""
    detail: str = ""
    ip_address: str = ""
    created_at: str = ""
