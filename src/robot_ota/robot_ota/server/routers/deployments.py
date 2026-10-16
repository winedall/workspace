import uuid
import logging
from fastapi import APIRouter, Depends, HTTPException
from ..database import Database, get_db_path
from ..schemas import DeploymentCreate, DeploymentResponse
from ..auth import verify_token
from ..audit_log import write_audit_log

logger = logging.getLogger("ota_server.deployments")

router = APIRouter(prefix="/api/v1/deployments", tags=["deployments"])


def get_db() -> Database:
    return Database(get_db_path())


@router.get("/", response_model=list[DeploymentResponse])
def list_deployments(_=Depends(verify_token)):
    """查询所有设备升级任务。"""
    db = get_db()
    with db.connect() as conn:
        rows = conn.execute("SELECT * FROM deployments ORDER BY created_at DESC").fetchall()
        return [dict(r) for r in rows]


@router.get("/{deployment_id}", response_model=DeploymentResponse)
def get_deployment(deployment_id: int, _=Depends(verify_token)):
    """查询单个设备升级任务。"""
    db = get_db()
    with db.connect() as conn:
        row = conn.execute("SELECT * FROM deployments WHERE id = ?", (deployment_id,)).fetchone()
        if not row:
            raise HTTPException(status_code=404, detail="Deployment not found")
        return dict(row)


@router.post("/", response_model=dict, status_code=201)
def create_deployment(payload: DeploymentCreate, _=Depends(verify_token)):
    """创建设备升级任务，每台设备一行记录。

    同批设备共享一个 batch_id 用于关联。
    """
    db = get_db()
    with db.connect() as conn:
        release = conn.execute("SELECT * FROM releases WHERE id = ?", (payload.release_id,)).fetchone()
        if not release:
            raise HTTPException(status_code=404, detail="Release not found")

        if not payload.device_ids:
            raise HTTPException(status_code=400, detail="No devices selected")

        batch_id = str(uuid.uuid4())
        created_rows = []
        for device_id in payload.device_ids:
            dev = conn.execute("SELECT current_version FROM devices WHERE id = ?", (device_id,)).fetchone()
            if not dev:
                continue
            conn.execute(
                """INSERT INTO deployments
                   (batch_id, device_id, release_id, release_version, from_version, status)
                   VALUES (?, ?, ?, ?, ?, 'pending')""",
                (batch_id, device_id, payload.release_id, release["version"], dev["current_version"]),
            )
            conn.commit()
            row = conn.execute("SELECT * FROM deployments WHERE id = last_insert_rowid()").fetchone()
            created_rows.append(dict(row))

        device_detail = ",".join(f"{r['device_id']}({r['from_version'] or '—'})" for r in created_rows)
        write_audit_log("create_deployment", target=f"v{release['version']}", detail=f"devices={device_detail}")
        return {"batch_id": batch_id, "deployments": created_rows}


@router.post("/{deployment_id}/rollback", response_model=dict, status_code=201)
def rollback_deployment(deployment_id: int, _=Depends(verify_token)):
    """手动回滚：找到 from_version 对应的 release，创建新的部署将设备回退到上一个版本。"""
    db = get_db()
    with db.connect() as conn:
        dep = conn.execute(
            "SELECT * FROM deployments WHERE id = ?", (deployment_id,)
        ).fetchone()
        if not dep:
            raise HTTPException(status_code=404, detail="Deployment not found")
        if dep["status"] != "SUCCEEDED":
            raise HTTPException(status_code=400, detail="Only SUCCEEDED deployments can be rolled back")

        # 检查该行是否是该设备最新的 SUCCEEDED 记录
        latest = conn.execute(
            """SELECT id FROM deployments
               WHERE device_id = ? AND status = 'SUCCEEDED'
               ORDER BY id DESC LIMIT 1""",
            (dep["device_id"],),
        ).fetchone()
        if not latest or latest["id"] != deployment_id:
            raise HTTPException(
                status_code=400,
                detail="Only the latest SUCCEEDED deployment can be rolled back",
            )

        if not dep["from_version"] or dep["from_version"] in ("", "0.0.0"):
            raise HTTPException(status_code=400, detail="No previous version to roll back to")

        prev_release = conn.execute(
            "SELECT * FROM releases WHERE version = ? AND product = 'zl_robot'",
            (dep["from_version"],),
        ).fetchone()
        if not prev_release:
            raise HTTPException(
                status_code=404,
                detail=f"Release for version '{dep['from_version']}' not found",
            )

        # 创建新的部署记录（版本对调）
        batch_id = str(uuid.uuid4())
        current_version = dep["release_version"]
        target_version = dep["from_version"]
        conn.execute(
            """INSERT INTO deployments
               (batch_id, device_id, release_id, release_version, from_version, status)
               VALUES (?, ?, ?, ?, ?, 'pending')""",
            (batch_id, dep["device_id"], prev_release["id"], target_version, current_version),
        )
        conn.commit()

        row = conn.execute("SELECT * FROM deployments WHERE id = last_insert_rowid()").fetchone()
        write_audit_log("rollback", target=f"v{target_version}", detail=f"devices={dep['device_id']}; {current_version}->{target_version}")
        logger.info("Rollback deployment %s created for device %s: %s -> %s",
                     row["id"], dep["device_id"], current_version, target_version)
        return dict(row)
