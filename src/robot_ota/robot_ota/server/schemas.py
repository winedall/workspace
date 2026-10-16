from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime


class DeviceCreate(BaseModel):
    """创建设备的请求体。"""
    id: str
    agent_address: str = ""
    current_version: str = ""
    server_url: str = ""


class DeviceUpdate(BaseModel):
    """更新设备信息的请求体（所有字段可选）。"""
    agent_address: Optional[str] = None
    current_version: Optional[str] = None
    robot_state: Optional[str] = None


class DeviceResponse(BaseModel):
    """设备响应体（不含 Token）。"""
    id: str
    agent_address: str
    current_version: str
    agent_online: bool
    robot_online: bool
    robot_state: str
    last_robot_state_at: str
    last_heartbeat: str
    created_at: str
    updated_at: str


class DeviceCreateResponse(DeviceResponse):
    """创建设备响应体（含 Token，仅创建时返回一次）。"""
    token: str


class ReleaseUpload(BaseModel):
    """版本上传元数据。"""
    version: str
    product: str = "zl_robot"
    platform: str = "linux-amd64"
    file_size: int = 0
    manifest: str = "{}"
    release_notes: str = ""


class ReleaseResponse(BaseModel):
    """版本响应体。"""
    id: int
    version: str
    product: str
    platform: str
    archive_path: str
    archive_hash: str
    signature_path: str
    file_size: int
    release_notes: str
    created_by: str
    created_at: str


class DeploymentCreate(BaseModel):
    """创建部署的请求体。"""
    release_id: int
    device_ids: list[str] = Field(default_factory=list)


class DeploymentResponse(BaseModel):
    """设备升级任务响应体。"""
    id: int
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


class DeviceDetailResponse(BaseModel):
    """设备详情响应体（含版本历史）。"""
    id: str
    agent_address: str
    current_version: str
    agent_online: bool
    robot_online: bool
    robot_state: str
    last_robot_state_at: str
    last_heartbeat: str
    created_at: str
    updated_at: str
    version_history: list[dict]


class AuditLogResponse(BaseModel):
    """审计日志响应体。"""
    id: int
    action: str
    actor: str
    target: str
    detail: str
    ip_address: str
    created_at: str


class LoginRequest(BaseModel):
    """登录请求体。"""
    username: str
    password: str


class TokenResponse(BaseModel):
    """登录成功后的 Token 响应。"""
    access_token: str
    token_type: str = "bearer"
