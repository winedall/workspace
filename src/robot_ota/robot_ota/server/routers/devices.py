import hashlib
import logging
import os
import re
import secrets
import shutil
import sqlite3
import tarfile
import tempfile
from fastapi import APIRouter, BackgroundTasks, Depends, HTTPException, Request
from fastapi.responses import FileResponse
from pydantic import BaseModel, Field
from ..agent_deploy import (
    AgentDeployConfig,
    SshAgentDeployer,
    create_agent_deployment,
    get_agent_deployment,
    latest_agent_deployment,
)
from ..auth import verify_token
from ..audit_log import write_audit_log
from ..config import Settings
from ..database import Database, get_db_path
from ..schemas import DeviceCreate, DeviceUpdate, DeviceResponse, DeviceCreateResponse, DeviceDetailResponse

router = APIRouter(prefix="/api/v1/devices", tags=["devices"])
_agent_deploy_router = APIRouter(tags=["agent-deployments"])

logger = logging.getLogger(__name__)

# Agent 源代码目录
AGENT_SRC_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "..", "agent")
# 部署包目录
DEPLOY_PACKAGES_DIR = Settings().deploy_packages_dir
# Agent 源文件分类规则
_AGENT_PYTHON_FILES = lambda f: f.endswith(".py")
_AGENT_ROOT_FILES = {"install.sh", "ota_agent.service", "zl_robot.service"}

# 设备查询语句
_DEVICE_SELECT = """SELECT
    id, agent_address, token_hash, current_version,
    robot_state, last_robot_state_at, last_heartbeat, created_at, updated_at,
    CASE
      WHEN last_heartbeat != ''
       AND datetime(last_heartbeat) >= datetime('now', '-5 seconds')
      THEN 1 ELSE 0
    END AS agent_online,
    CASE
      WHEN robot_state != ''
       AND last_robot_state_at != ''
       AND datetime(last_robot_state_at) >= datetime('now', '-5 seconds')
      THEN 1 ELSE 0
    END AS robot_online
"""

class AgentDeploymentRequest(BaseModel):
    """Agent 部署请求体。"""
    target_host: str = Field(..., min_length=1, max_length=255)
    ssh_user: str = Field("root", min_length=1, max_length=64)
    ssh_port: int = Field(22, ge=1, le=65535)
    password: str = ""
    private_key_path: str = ""
    agent_root: str = ""


def get_db() -> Database:
    """获取数据库连接。"""
    return Database(get_db_path())


def _device_dict(row) -> dict:
    """将数据库查询结果转换为设备字典。"""
    item = dict(row)
    item["agent_online"] = bool(item["agent_online"])
    item["robot_online"] = bool(item["robot_online"])
    return item


@router.get("/", response_model=list[DeviceResponse])
def list_devices(_=Depends(verify_token)):
    """查询所有已注册设备。"""
    db = get_db()
    with db.connect() as conn:
        rows = conn.execute(
            _DEVICE_SELECT + "FROM devices ORDER BY created_at DESC"
        ).fetchall()
        return [_device_dict(r) for r in rows]


@router.get("/{device_id}", response_model=DeviceResponse)
def get_device(device_id: str, _=Depends(verify_token)):
    """查询单个设备详情。"""
    db = get_db()
    with db.connect() as conn:
        row = conn.execute(
            _DEVICE_SELECT + "FROM devices WHERE id = ?",
            (device_id,),
        ).fetchone()
        if not row:
            raise HTTPException(status_code=404, detail="Device not found")
        return _device_dict(row)


@router.get("/{device_id}/detail", response_model=DeviceDetailResponse)
def get_device_detail(device_id: str, _=Depends(verify_token)):
    """查询设备详情，包含版本历史清单。"""
    db = get_db()
    with db.connect() as conn:
        row = conn.execute(
            _DEVICE_SELECT + "FROM devices WHERE id = ?",
            (device_id,),
        ).fetchone()
        if not row:
            raise HTTPException(status_code=404, detail="Device not found")
        item = _device_dict(row)
        history = conn.execute(
            """SELECT release_version, from_version, status, created_at
               FROM deployments
               WHERE device_id = ?
               ORDER BY created_at DESC""",
            (device_id,),
        ).fetchall()
        item["version_history"] = [dict(h) for h in history]
        return item


def _generate_deploy_package(device_id: str, device_token: str, server_url: str) -> str:
    """生成部署包。

    将 Agent 源码、配置文件、签名密钥打包为 tar.gz。
    """
    pkg_dir = os.path.join(DEPLOY_PACKAGES_DIR, device_id)
    os.makedirs(pkg_dir, exist_ok=True)
    pkg_path = os.path.join(pkg_dir, "ota_agent.tar.gz")

    tmp_dir = tempfile.mkdtemp(prefix="ota_deploy_")
    try:
        _copy_agent_files(tmp_dir)
        _write_device_yaml(tmp_dir, device_id, device_token, server_url)
        _copy_signing_key(tmp_dir)
        _copy_agent_wheels(tmp_dir)
        _copy_agent_depends(tmp_dir)
        _create_tarball(pkg_path, tmp_dir)
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)

    return pkg_path


def _copy_agent_files(tmp_dir: str) -> None:
    """复制 Agent 源文件到临时目录。"""
    pkg_agent_dir = os.path.join(tmp_dir, "agent")
    os.makedirs(pkg_agent_dir, exist_ok=True)

    for fname in os.listdir(AGENT_SRC_DIR):
        src = os.path.join(AGENT_SRC_DIR, fname)
        if _AGENT_PYTHON_FILES(fname):
            shutil.copy2(src, os.path.join(pkg_agent_dir, fname))
        elif fname in _AGENT_ROOT_FILES:
            shutil.copy2(src, os.path.join(tmp_dir, fname))


def _write_device_yaml(tmp_dir: str, device_id: str, device_token: str, server_url: str) -> None:
    """生成设备身份配置文件。"""
    settings = Settings()
    content = f"""# 设备身份配置（由 OTA Server 自动生成）
device_id: "{device_id}"
device_token: "{device_token}"
server_url: "{server_url}"
agent_root: "{settings.agent_root}"
"""
    with open(os.path.join(tmp_dir, "device.yaml"), "w") as f:
        f.write(content)


def _copy_signing_key(tmp_dir: str) -> None:
    """复制签名公钥到临时目录。"""
    settings = Settings()
    if settings.public_key_path:
        signing_dir = os.path.join(tmp_dir, "scripts", "signing")
        os.makedirs(signing_dir, exist_ok=True)
        shutil.copy2(
            settings.public_key_path,
            os.path.join(signing_dir, "public_key.key"),
        )


def _copy_agent_wheels(tmp_dir: str) -> None:
    """复制 Agent 离线依赖 wheel 到部署包。"""
    wheels_src_dir = Settings().agent_wheels_dir
    if not wheels_src_dir or not os.path.isdir(wheels_src_dir):
        raise RuntimeError(f"offline Python wheels directory not found: {wheels_src_dir}")

    wheel_names = sorted(name for name in os.listdir(wheels_src_dir) if name.endswith(".whl"))
    if not wheel_names:
        raise RuntimeError(f"offline Python wheels not found in: {wheels_src_dir}")

    wheels_dir = os.path.join(tmp_dir, "wheels")
    os.makedirs(wheels_dir, exist_ok=True)
    for name in wheel_names:
        shutil.copy2(
            os.path.join(wheels_src_dir, name),
            os.path.join(wheels_dir, name),
        )


def _copy_agent_depends(tmp_dir: str) -> None:
    """复制 Agent 离线 deb 依赖包到部署包。"""
    debs_src_dir = Settings().agent_depends_dir
    if not debs_src_dir or not os.path.isdir(debs_src_dir):
        raise RuntimeError(f"offline deb packages directory not found: {debs_src_dir}")

    deb_names = sorted(name for name in os.listdir(debs_src_dir) if name.endswith(".deb"))
    if not deb_names:
        raise RuntimeError(f"offline deb packages not found in: {debs_src_dir}")

    debs_dir = os.path.join(tmp_dir, "depends")
    os.makedirs(debs_dir, exist_ok=True)
    for name in deb_names:
        shutil.copy2(
            os.path.join(debs_src_dir, name),
            os.path.join(debs_dir, name),
        )


def _create_tarball(pkg_path: str, src_dir: str) -> None:
    """将临时目录打包为 tar.gz。"""
    with tarfile.open(pkg_path, "w:gz") as tar:
        for name in os.listdir(src_dir):
            fpath = os.path.join(src_dir, name)
            tar.add(fpath, arcname=name)


@router.post("/", response_model=DeviceCreateResponse, status_code=201)
def create_device(payload: DeviceCreate, request: Request, _=Depends(verify_token)):
    """注册新设备，自动生成设备 Token（返回明文 Token 仅此一次）。

    Token 存入数据库，Agent 端在 device.yaml 中配置相同的值，
    Server 推送升级命令时凭此 Token 认证 Agent API。
    """
    if not payload.server_url:
        raise HTTPException(status_code=400, detail="server_url is required")

    db = get_db()
    token_plain = secrets.token_hex(16)
    token_hash = hashlib.sha256(token_plain.encode()).hexdigest()
    with db.connect() as conn:
        try:
            conn.execute(
                """INSERT INTO devices (id, agent_address, token_hash, token, current_version)
                   VALUES (?, ?, ?, ?, ?)""",
                (payload.id, payload.agent_address, token_hash, token_plain, payload.current_version),
            )
            conn.commit()
        except sqlite3.IntegrityError:
            conn.rollback()
            raise HTTPException(status_code=409, detail=f"设备 ID '{payload.id}' 已存在")

        try:
            _generate_deploy_package(payload.id, token_plain, payload.server_url)
        except Exception as e:
            logger.warning(f"Failed to generate deploy package for device {payload.id}: {e}")

        row = conn.execute(
            _DEVICE_SELECT + ", token FROM devices WHERE id = ?", (payload.id,)
        ).fetchone()
        ip = request.client.host if request.client else ""
        write_audit_log("create_device", target=payload.id, ip_address=ip)
        return _device_dict(row)


@router.put("/{device_id}", response_model=DeviceResponse)
def update_device(device_id: str, payload: DeviceUpdate, _=Depends(verify_token)):
    """更新设备信息（仅更新传入的字段）。"""
    db = get_db()
    with db.connect() as conn:
        existing = conn.execute("SELECT 1 FROM devices WHERE id = ?", (device_id,)).fetchone()
        if not existing:
            raise HTTPException(status_code=404, detail="Device not found")
        updates = payload.model_dump(exclude_unset=True)
        if not updates:
            row = conn.execute(
                _DEVICE_SELECT + "FROM devices WHERE id = ?", (device_id,)
            ).fetchone()
            return _device_dict(row)
        set_clause = ", ".join(f"{k} = ?" for k in updates)
        values = list(updates.values()) + [device_id]
        conn.execute(f"UPDATE devices SET {set_clause}, updated_at = datetime('now') WHERE id = ?", values)
        conn.commit()
        write_audit_log("update_device", target=device_id, detail=f"fields: {list(updates.keys())}")
        row = conn.execute(
            _DEVICE_SELECT + "FROM devices WHERE id = ?", (device_id,)
        ).fetchone()
        return _device_dict(row)


@router.delete("/{device_id}", status_code=204)
def delete_device(device_id: str, _=Depends(verify_token)):
    """删除设备（不可逆，相关任务记录一并删除）。"""
    db = get_db()
    with db.connect() as conn:
        existing = conn.execute("SELECT * FROM devices WHERE id = ?", (device_id,)).fetchone()
        if not existing:
            raise HTTPException(status_code=404, detail="Device not found")
        conn.execute("DELETE FROM deployments WHERE device_id = ?", (device_id,))
        conn.execute("DELETE FROM agent_deployments WHERE device_id = ?", (device_id,))
        conn.execute("DELETE FROM devices WHERE id = ?", (device_id,))
        conn.commit()
    write_audit_log("delete_device", target=device_id, detail=f"version={existing['current_version']}")
    pkg_dir = os.path.join(DEPLOY_PACKAGES_DIR, device_id)
    if os.path.exists(pkg_dir):
        shutil.rmtree(pkg_dir, ignore_errors=True)


@router.get("/{device_id}/deploy-package")
def download_deploy_package(device_id: str):
    """下载设备部署包。"""
    if "/" in device_id or ".." in device_id:
        raise HTTPException(status_code=400, detail="Invalid device_id")
    pkg_dir = os.path.join(DEPLOY_PACKAGES_DIR, device_id)
    pkg_path = os.path.join(pkg_dir, "ota_agent.tar.gz")
    real_pkg = os.path.realpath(pkg_path)
    real_base = os.path.realpath(DEPLOY_PACKAGES_DIR)
    if not real_pkg.startswith(real_base + "/"):
        raise HTTPException(status_code=400, detail="Invalid device_id")
    if not os.path.exists(real_pkg):
        raise HTTPException(status_code=404, detail="Deploy package not found. Has the device been registered?")
    return FileResponse(
        real_pkg,
        filename=f"ota_agent-{device_id}.tar.gz",
        media_type="application/gzip",
    )


# ── Agent 部署相关路由 ──────────────────────────────────────────


def _package_path(device_id: str) -> str:
    """获取设备部署包路径。"""
    return os.path.join(DEPLOY_PACKAGES_DIR, device_id, "ota_agent.tar.gz")


def _validate_ssh_input(payload: AgentDeploymentRequest):
    """验证 SSH 输入参数。"""
    host_pattern = re.compile(r"^[A-Za-z0-9_.:-]+$")
    user_pattern = re.compile(r"^[A-Za-z0-9_.-]+$")
    if not host_pattern.match(payload.target_host):
        raise HTTPException(status_code=400, detail="Invalid target_host")
    if not user_pattern.match(payload.ssh_user):
        raise HTTPException(status_code=400, detail="Invalid ssh_user")
    if not payload.password and not payload.private_key_path:
        raise HTTPException(status_code=400, detail="password or private_key_path is required")


def _run_deployment(deployment_id: int, package_path: str, config: AgentDeployConfig):
    """执行 Agent 部署任务。"""
    SshAgentDeployer(get_db()).deploy(deployment_id, package_path, config)
    row = get_agent_deployment(get_db(), deployment_id)
    if row:
        detail = f"host={config.target_host}"
        if row["error"]:
            detail += f" error={row['error']}"
        write_audit_log(
            f"agent_deploy_{row['status']}",
            target=row["device_id"],
            detail=detail,
        )


@router.post("/{device_id}/agent-deployments", status_code=201)
def start_agent_deployment(
    device_id: str,
    payload: AgentDeploymentRequest,
    background_tasks: BackgroundTasks,
    _=Depends(verify_token),
):
    """启动 Agent 部署。"""
    _validate_ssh_input(payload)
    db = get_db()
    with db.connect() as conn:
        device = conn.execute("SELECT * FROM devices WHERE id = ?", (device_id,)).fetchone()
    if not device:
        raise HTTPException(status_code=404, detail="Device not found")

    package_path = _package_path(device_id)
    if not os.path.exists(package_path):
        raise HTTPException(status_code=404, detail="Deploy package not found")

    deployment_id = create_agent_deployment(
        db,
        device_id=device_id,
        target_host=payload.target_host,
        ssh_user=payload.ssh_user,
        ssh_port=payload.ssh_port,
    )
    config = AgentDeployConfig(
        target_host=payload.target_host,
        ssh_user=payload.ssh_user,
        ssh_port=payload.ssh_port,
        password=payload.password,
        private_key_path=payload.private_key_path,
        agent_root=payload.agent_root or Settings().agent_root,
    )
    background_tasks.add_task(_run_deployment, deployment_id, package_path, config)
    return {
        "id": deployment_id,
        "device_id": device_id,
        "target_host": payload.target_host,
        "ssh_user": payload.ssh_user,
        "ssh_port": payload.ssh_port,
        "agent_root": config.agent_root,
        "status": "pending",
        "log": "",
        "error": "",
    }


@router.get("/{device_id}/agent-deployments/latest")
def get_latest_device_agent_deployment(device_id: str, _=Depends(verify_token)):
    """获取设备最新 Agent 部署记录。"""
    row = latest_agent_deployment(get_db(), device_id)
    if not row:
        raise HTTPException(status_code=404, detail="Agent deployment not found")
    return row


@_agent_deploy_router.get("/api/v1/agent-deployments/{deployment_id}")
def get_agent_deployment_detail(deployment_id: int, _=Depends(verify_token)):
    """获取 Agent 部署详情。"""
    row = get_agent_deployment(get_db(), deployment_id)
    if not row:
        raise HTTPException(status_code=404, detail="Agent deployment not found")
    return row
