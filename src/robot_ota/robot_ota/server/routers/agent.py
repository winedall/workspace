import hashlib

from fastapi import APIRouter, Depends, Header, HTTPException, Request
from pydantic import BaseModel, Field

from ..database import Database, get_db_path
from ..audit_log import write_audit_log

router = APIRouter(prefix="/api/v1/agent", tags=["agent"])


class AgentHeartbeat(BaseModel):
    current_version: str = ""
    agent_version: str = ""
    agent_address: str = ""
    robot_online: bool = False
    robot_state: str = ""


class AgentTaskStatus(BaseModel):
    status: str
    progress: int = Field(default=0, ge=0, le=100)
    error: str = ""
    current_version: str = ""


def get_db() -> Database:
    return Database(get_db_path())


def authenticate_device(
    x_device_id: str = Header(..., alias="X-Device-Id"),
    authorization: str = Header(..., alias="Authorization"),
) -> dict:
    if not authorization.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Invalid device credentials")
    token = authorization[7:]
    token_hash = hashlib.sha256(token.encode()).hexdigest()

    db = get_db()
    with db.connect() as conn:
        row = conn.execute("SELECT * FROM devices WHERE id = ?", (x_device_id,)).fetchone()
    if not row or row["token_hash"] != token_hash:
        raise HTTPException(status_code=401, detail="Invalid device credentials")
    return dict(row)


def _url_for(request: Request, path: str) -> str:
    return str(request.base_url).rstrip("/") + path


@router.post("/heartbeat")
def heartbeat(payload: AgentHeartbeat, request: Request, device=Depends(authenticate_device)):
    agent_address = payload.agent_address
    if not agent_address and request.client:
        agent_address = request.client.host
    db = get_db()
    with db.connect() as conn:
        conn.execute(
            """UPDATE devices
               SET last_heartbeat = datetime('now'),
                   current_version = CASE
                       WHEN ? != '' AND ? != '0.0.0' THEN ?
                       ELSE current_version
                   END,
                   agent_address = COALESCE(NULLIF(?, ''), agent_address),
                   robot_state = ?,
                   last_robot_state_at = CASE
                       WHEN ? THEN datetime('now')
                       ELSE last_robot_state_at
                   END,
                   updated_at = datetime('now')
               WHERE id = ?""",
            (
                payload.current_version,
                payload.current_version,
                payload.current_version,
                agent_address,
                payload.robot_state,
                payload.robot_online,
                device["id"],
            ),
        )
        conn.commit()
    return {"status": "ok"}


@router.get("/tasks/next")
def next_task(request: Request, device=Depends(authenticate_device)):
    db = get_db()
    with db.connect() as conn:
        row = conn.execute(
            """SELECT
                   d.id AS deployment_id,
                   d.release_version AS release_version,
                   d.from_version AS from_version,
                   r.id AS release_id,
                   r.version AS version,
                   r.archive_hash AS archive_hash
               FROM deployments d
               JOIN releases r ON r.id = d.release_id
               WHERE d.device_id = ?
                 AND d.status IN ('pending', 'CREATED')
               ORDER BY d.created_at ASC
               LIMIT 1""",
            (device["id"],),
        ).fetchone()

        if not row:
            return {"task": None}

        conn.execute(
            "UPDATE deployments SET status = 'CREATED', updated_at = datetime('now') WHERE id = ?",
            (row["deployment_id"],),
        )
        conn.commit()

    release_id = row["release_id"]
    return {
        "task_id": row["deployment_id"],
        "deployment_id": row["deployment_id"],
        "release_id": release_id,
        "device_id": device["id"],
        "version": row["version"],
        "from_version": row["from_version"],
        "download_url": _url_for(request, f"/api/v1/releases/{release_id}/download"),
        "signature_url": _url_for(request, f"/api/v1/releases/{release_id}/download.sig"),
        "archive_hash": row["archive_hash"],
    }


def _version_tuple(v: str) -> tuple:
    try:
        return tuple(int(x) for x in v.split("."))
    except (ValueError, AttributeError):
        return (0, 0, 0)


@router.post("/tasks/{task_id}/status")
def report_task_status(task_id: str, payload: AgentTaskStatus, device=Depends(authenticate_device)):
    db = get_db()
    with db.connect() as conn:
        dep = conn.execute(
            "SELECT * FROM deployments WHERE id = ? AND device_id = ?",
            (int(task_id), device["id"]),
        ).fetchone()
        if not dep:
            raise HTTPException(status_code=404, detail="Task not found")

        conn.execute(
            """UPDATE deployments
               SET status = ?, progress = ?, error = ?, updated_at = datetime('now')
               WHERE id = ?""",
            (payload.status, payload.progress, payload.error, int(task_id)),
        )
        if payload.current_version:
            conn.execute(
                """UPDATE devices
                   SET current_version = ?,
                       last_heartbeat = datetime('now'),
                       updated_at = datetime('now')
                   WHERE id = ?""",
                (payload.current_version, device["id"]),
            )
        else:
            conn.execute(
                """UPDATE devices
                   SET last_heartbeat = datetime('now'),
                       updated_at = datetime('now')
                   WHERE id = ?""",
                (device["id"],),
            )
        conn.commit()

    terminal_statuses = {'SUCCEEDED', 'FAILED', 'ROLLED_BACK', 'RECOVERY_REQUIRED', 'CANCELLED'}
    if payload.status in terminal_statuses:
        action = payload.status.lower()
        if payload.status in ('SUCCEEDED', 'FAILED') and _version_tuple(dep['release_version']) < _version_tuple(dep['from_version']):
            action = f"rollback_{action}"
        from_ver = dep['from_version'] or "—"
        detail = f"device={device['id']}; from={from_ver}"
        if payload.error:
            detail += f"; error={payload.error}"
        write_audit_log(action, target=f"v{dep['release_version']}", actor=device["id"], detail=detail)

    return {"status": "ok"}
