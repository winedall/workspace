"""版本发布管理路由。

提供版本上传、查询、删除、下载等接口。
上传采用异步方案：流式写入临时文件后立即返回，后台处理签名和数据库写入。
"""
import hashlib
import logging
import os
import secrets
import shutil
import tempfile
import uuid
from fastapi import APIRouter, BackgroundTasks, Depends, HTTPException, Request, UploadFile, File, Form
from fastapi.responses import FileResponse
from pydantic import BaseModel
from ..database import Database, get_db_path
from ..schemas import ReleaseResponse
from ..auth import verify_token
from ..audit_log import write_audit_log
from ..config import Settings

router = APIRouter(prefix="/api/v1/releases", tags=["releases"])

UPLOAD_DIR = Settings().releases_dir
UPLOAD_TASKS_DIR = os.path.join(Settings().data_root, "upload_tasks")
UPLOAD_TEMP_DIR = os.path.join(Settings().data_root, "tmp")
logger = logging.getLogger(__name__)

# 上传任务在内存中跟踪进度（避免频繁写数据库）
_upload_tasks: dict[str, dict] = {}

# 进度更新节流：后台任务每N次调用才写一次数据库
_progress_db_counter: dict[str, int] = {}


def get_db() -> Database:
    """获取数据库连接。"""
    return Database(get_db_path())


class UploadTaskCreate(BaseModel):
    version: str
    product: str = "zl_robot"
    platform: str = "linux-amd64"
    filename: str
    release_notes: str = ""


def _safe_task_dir(task_id: str) -> str:
    if "/" in task_id or "\\" in task_id or ".." in task_id:
        raise HTTPException(status_code=400, detail="Invalid task_id")
    task_dir = os.path.realpath(os.path.join(UPLOAD_TASKS_DIR, task_id))
    base = os.path.realpath(UPLOAD_TASKS_DIR)
    if not task_dir.startswith(base + os.sep):
        raise HTTPException(status_code=400, detail="Invalid task_id")
    return task_dir


def _task_file(task_id: str, name: str) -> str:
    return os.path.join(_safe_task_dir(task_id), name)


def _load_upload_task(task_id: str) -> dict:
    if task_id in _upload_tasks:
        return _upload_tasks[task_id]
    db = get_db()
    with db.connect() as conn:
        row = conn.execute("SELECT * FROM release_uploads WHERE task_id = ?", (task_id,)).fetchone()
    if not row:
        raise HTTPException(status_code=404, detail="Upload task not found")
    item = dict(row)
    item.setdefault("release_notes", "")
    _upload_tasks[task_id] = item
    return item


def _create_upload_record(task_id: str, payload: UploadTaskCreate):
    task_info = {
        "task_id": task_id,
        "status": "created",
        "progress": 1,
        "filename": payload.filename,
        "version": payload.version,
        "product": payload.product,
        "platform": payload.platform,
        "release_notes": payload.release_notes,
        "error": "",
        "created_at": "",
        "updated_at": "",
    }
    _upload_tasks[task_id] = task_info
    db = get_db()
    with db.connect() as conn:
        conn.execute(
            """INSERT INTO release_uploads (task_id, status, progress, filename, version, product, platform)
               VALUES (?, 'created', 1, ?, ?, ?, ?)""",
            (task_id, payload.filename, payload.version, payload.product, payload.platform),
        )
        conn.commit()
    return task_info


def _update_upload_task_fast(task_id: str, status: str, progress: int):
    """快速更新内存进度，按节流策略写数据库。"""
    if task_id in _upload_tasks:
        _upload_tasks[task_id]["status"] = status
        _upload_tasks[task_id]["progress"] = progress
    # 节流：每5次调用写一次数据库
    _progress_db_counter[task_id] = _progress_db_counter.get(task_id, 0) + 1
    if _progress_db_counter[task_id] % 5 == 0:
        _update_upload_db(task_id, status, progress)


def _update_upload_db(task_id: str, status: str, progress: int = 0, error: str = ""):
    """更新上传任务到数据库。"""
    db = get_db()
    with db.connect() as conn:
        conn.execute(
            """UPDATE release_uploads
               SET status = ?, progress = ?, error = ?, updated_at = datetime('now')
               WHERE task_id = ?""",
            (status, progress, error, task_id),
        )
        conn.commit()


def _process_upload(task_id: str, archive_tmp: str, sig_tmp: str,
                    version: str, product: str, platform: str, release_notes: str,
                    tmp_dir: str | None = None):
    """后台处理上传任务。

    流程：移动临时文件 → 计算哈希 → 写入数据库
    进度节点：85%(上传完成) → 90%(文件就位) → 95%(哈希完成) → 100%(完成)
    """
    try:
        _update_upload_task_fast(task_id, "processing", 90)

        os.makedirs(UPLOAD_DIR, exist_ok=True)

        archive_name = f"{product}-{version}-{platform}.tar.gz"
        archive_path = os.path.join(UPLOAD_DIR, archive_name)
        sig_name = f"{archive_name}.sig"
        sig_path = os.path.join(UPLOAD_DIR, sig_name)

        # 移动临时文件到最终位置
        os.rename(archive_tmp, archive_path)
        os.rename(sig_tmp, sig_path)

        _update_upload_task_fast(task_id, "processing", 92)

        # 计算哈希
        with open(archive_path, "rb") as f:
            archive_hash = hashlib.sha256(f.read()).hexdigest()
        file_size = os.path.getsize(archive_path)

        _update_upload_task_fast(task_id, "processing", 95)

        # 写入数据库（使用同一个连接，避免数据库锁竞争）
        db = get_db()
        with db.connect() as conn:
            try:
                conn.execute(
                    """INSERT INTO releases (version, product, platform, archive_path, archive_hash, signature_path, file_size, release_notes)
                       VALUES (?, ?, ?, ?, ?, ?, ?, ?)""",
                    (version, product, platform, archive_path, archive_hash, sig_path, file_size, release_notes),
                )
                conn.commit()
            except Exception as e:
                conn.rollback()
                # 清理已移动的文件
                for fpath in (archive_path, sig_path):
                    if os.path.exists(fpath):
                        os.remove(fpath)
                _update_upload_db(task_id, "failed", 0, str(e))
                if task_id in _upload_tasks:
                    _upload_tasks[task_id]["status"] = "failed"
                    _upload_tasks[task_id]["error"] = str(e)
                logger.warning(f"Upload task {task_id} failed: {e}")
                return

        _update_upload_db(task_id, "succeeded", 100)
        if task_id in _upload_tasks:
            _upload_tasks[task_id]["status"] = "succeeded"
            _upload_tasks[task_id]["progress"] = 100
        # 清理节流计数器
        _progress_db_counter.pop(task_id, None)
        write_audit_log("upload_release", target=f"v{version}", detail=f"{product}-{version}-{platform}, size={file_size}")
        logger.info(f"Upload task {task_id} completed: {archive_name}")

    except Exception as e:
        _update_upload_db(task_id, "failed", 0, str(e))
        if task_id in _upload_tasks:
            _upload_tasks[task_id]["status"] = "failed"
            _upload_tasks[task_id]["error"] = str(e)
        _progress_db_counter.pop(task_id, None)
        logger.error(f"Upload task {task_id} failed: {e}")
    finally:
        if tmp_dir and os.path.exists(tmp_dir):
            shutil.rmtree(tmp_dir, ignore_errors=True)


@router.get("/", response_model=list[ReleaseResponse])
def list_releases(_=Depends(verify_token)):
    """查询所有已上传的版本发布包。"""
    db = get_db()
    with db.connect() as conn:
        rows = conn.execute("SELECT * FROM releases ORDER BY created_at DESC").fetchall()
        return [dict(r) for r in rows]


@router.get("/uploads")
def list_upload_tasks(_=Depends(verify_token)):
    """查询所有上传任务（用于显示进度）。"""
    db = get_db()
    with db.connect() as conn:
        rows = conn.execute(
            """SELECT * FROM release_uploads
               WHERE status != 'succeeded'
                 AND (status != 'failed' OR updated_at > datetime('now', '-60 seconds'))
               ORDER BY created_at DESC"""
        ).fetchall()
        return [dict(r) for r in rows]


@router.post("/uploads", status_code=201)
def create_upload_task(payload: UploadTaskCreate, _=Depends(verify_token)):
    """创建分阶段上传任务，立即返回 task_id。"""
    task_id = uuid.uuid4().hex[:16]
    os.makedirs(_safe_task_dir(task_id), exist_ok=True)
    task_info = _create_upload_record(task_id, payload)
    return {
        "task_id": task_id,
        "status": task_info["status"],
        "progress": task_info["progress"],
    }


@router.get("/uploads/{task_id}")
def get_upload_task(task_id: str, _=Depends(verify_token)):
    """查询上传任务状态。"""
    # 先查内存（更快）
    if task_id in _upload_tasks:
        return _upload_tasks[task_id]
    db = get_db()
    with db.connect() as conn:
        row = conn.execute("SELECT * FROM release_uploads WHERE task_id = ?", (task_id,)).fetchone()
        if not row:
            raise HTTPException(status_code=404, detail="Upload task not found")
        return dict(row)


async def _write_request_stream(task_id: str, request: Request, dest: str, start: int, end: int, done_status: str):
    task = _load_upload_task(task_id)
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    content_length = int(request.headers.get("content-length", "0") or "0")
    written = 0
    _update_upload_db(task_id, "uploading", start)
    if task_id in _upload_tasks:
        _upload_tasks[task_id]["status"] = "uploading"
        _upload_tasks[task_id]["progress"] = start
    with open(dest, "wb") as f:
        async for chunk in request.stream():
            if not chunk:
                continue
            f.write(chunk)
            written += len(chunk)
            if content_length > 0:
                progress = start + int((written / content_length) * (end - start))
                _update_upload_task_fast(task_id, "uploading", min(end, max(start, progress)))
    task["file_size"] = os.path.getsize(dest)
    _update_upload_db(task_id, done_status, end)
    if task_id in _upload_tasks:
        _upload_tasks[task_id]["status"] = done_status
        _upload_tasks[task_id]["progress"] = end
    return {"task_id": task_id, "status": done_status, "progress": end}


@router.put("/uploads/{task_id}/archive")
async def upload_archive_file(task_id: str, request: Request, _=Depends(verify_token)):
    """上传发布包归档，按服务端实际接收字节更新 5-80% 进度。"""
    _load_upload_task(task_id)
    return await _write_request_stream(
        task_id,
        request,
        _task_file(task_id, "archive"),
        5,
        80,
        "archive_uploaded",
    )


@router.put("/uploads/{task_id}/signature")
async def upload_signature_file(task_id: str, request: Request, _=Depends(verify_token)):
    """上传签名文件，按服务端实际接收字节更新 80-85% 进度。"""
    _load_upload_task(task_id)
    return await _write_request_stream(
        task_id,
        request,
        _task_file(task_id, "signature"),
        80,
        85,
        "signature_uploaded",
    )


@router.post("/uploads/{task_id}/complete", status_code=202)
def complete_upload_task(task_id: str, background_tasks: BackgroundTasks, _=Depends(verify_token)):
    """完成分阶段上传，进入后端处理与入库阶段。"""
    task = _load_upload_task(task_id)
    archive_tmp = _task_file(task_id, "archive")
    sig_tmp = _task_file(task_id, "signature")
    if not os.path.exists(archive_tmp):
        raise HTTPException(status_code=400, detail="Archive has not been uploaded")
    if not os.path.exists(sig_tmp):
        raise HTTPException(status_code=400, detail="Signature has not been uploaded")
    _update_upload_db(task_id, "processing", 85)
    task["status"] = "processing"
    task["progress"] = 85
    background_tasks.add_task(
        _process_upload,
        task_id,
        archive_tmp,
        sig_tmp,
        task["version"],
        task["product"],
        task["platform"],
        task.get("release_notes", ""),
    )
    return {"task_id": task_id, "status": "processing", "progress": 85}


@router.post("/upload", status_code=202)
async def upload_release(
    version: str = Form(...),
    product: str = Form("zl_robot"),
    platform: str = Form("linux-amd64"),
    release_notes: str = Form(""),
    archive: UploadFile = File(...),
    signature: UploadFile = File(...),
    background_tasks: BackgroundTasks = None,
    _=Depends(verify_token),
):
    """异步上传发布包和签名文件。

    流式写入临时文件后立即返回202，后台完成签名、哈希计算和数据库写入。
    客户端通过 GET /uploads/{task_id} 轮询进度。
    """
    task_id = uuid.uuid4().hex[:16]

    task_info = {
        "task_id": task_id,
        "status": "uploading",
        "progress": 1,
        "filename": archive.filename,
        "version": version,
        "product": product,
        "platform": platform,
        "error": "",
        "created_at": "",
        "updated_at": "",
    }
    _upload_tasks[task_id] = task_info

    # 创建数据库记录
    db = get_db()
    with db.connect() as conn:
        conn.execute(
            """INSERT INTO release_uploads (task_id, status, progress, filename, version, product, platform)
               VALUES (?, 'uploading', 1, ?, ?, ?, ?)""",
            (task_id, archive.filename, version, product, platform),
        )
        conn.commit()

    # 流式写入临时文件（不阻塞响应）
    os.makedirs(UPLOAD_TEMP_DIR, exist_ok=True)
    tmp_dir = tempfile.mkdtemp(prefix="upload_", dir=UPLOAD_TEMP_DIR)
    archive_tmp = os.path.join(tmp_dir, f"archive_{task_id}")
    sig_tmp = os.path.join(tmp_dir, f"sig_{task_id}")

    # 获取归档文件大小用于计算进度
    archive_size = int(archive.headers.get("content-length", 0)) if hasattr(archive, "headers") else 0

    written = 0
    with open(archive_tmp, "wb") as f:
        while chunk := await archive.read(65536):
            f.write(chunk)
            written += len(chunk)
            # 流式写入阶段：1~60%
            if archive_size > 0:
                progress = max(1, min(60, int(written / archive_size * 60)))
                _update_upload_task_fast(task_id, "uploading", progress)

    with open(sig_tmp, "wb") as f:
        while chunk := await signature.read(65536):
            f.write(chunk)

    _update_upload_task_fast(task_id, "uploading", 60)

    # 启动后台任务
    background_tasks.add_task(
        _process_upload,
        task_id,
        archive_tmp,
        sig_tmp,
        version,
        product,
        platform,
        release_notes,
        tmp_dir,
    )

    logger.info(f"Upload task {task_id} started: {archive.filename} ({archive_size} bytes)")
    return {
        "task_id": task_id,
        "status": "pending",
        "message": "Upload task started",
    }


@router.get("/{release_id}", response_model=ReleaseResponse)
def get_release(release_id: int, _=Depends(verify_token)):
    """查询单个版本详情。"""
    db = get_db()
    with db.connect() as conn:
        row = conn.execute("SELECT * FROM releases WHERE id = ?", (release_id,)).fetchone()
        if not row:
            raise HTTPException(status_code=404, detail="Release not found")
        return dict(row)


@router.delete("/{release_id}", status_code=204)
def delete_release(release_id: int, _=Depends(verify_token)):
    """删除版本（物理删除文件 + 数据库记录）。

    级联删除关联的部署和设备任务，异常时回滚事务。
    """
    db = get_db()
    with db.connect() as conn:
        try:
            row = conn.execute("SELECT * FROM releases WHERE id = ?", (release_id,)).fetchone()
            if not row:
                raise HTTPException(status_code=404, detail="Release not found")

            # 级联删除：先删部署 → 最后删版本
            conn.execute("DELETE FROM deployments WHERE release_id = ?", (release_id,))

            for path_field in ("archive_path", "signature_path"):
                fpath = row[path_field]
                if fpath and os.path.exists(fpath):
                    os.remove(fpath)
            conn.execute("DELETE FROM releases WHERE id = ?", (release_id,))
            conn.commit()
            write_audit_log("delete_release", target=f"v{row['version']}", detail=f"{row['product']}-{row['version']}-{row['platform']}")
        except HTTPException:
            raise
        except Exception:
            conn.rollback()
            raise


@router.get("/{release_id}/download")
def download_release(release_id: int):
    """下载发布包文件。

    开放接口，Agent 通过此地址下载 .tar.zst 归档。
    """
    db = get_db()
    with db.connect() as conn:
        row = conn.execute("SELECT * FROM releases WHERE id=?", (release_id,)).fetchone()
    if not row:
        raise HTTPException(404, "Release not found")
    if not os.path.exists(row["archive_path"]):
        raise HTTPException(404, "Archive file not found")
    return FileResponse(
        row["archive_path"],
        filename=os.path.basename(row["archive_path"]),
        media_type="application/gzip"
    )


@router.get("/{release_id}/download.sig")
def download_signature(release_id: int):
    """下载发布包签名文件。

    开放接口，Agent 通过此地址下载 .sig 文件。
    """
    db = get_db()
    with db.connect() as conn:
        row = conn.execute("SELECT * FROM releases WHERE id=?", (release_id,)).fetchone()
    if not row:
        raise HTTPException(404, "Release not found")
    if not os.path.exists(row["signature_path"]):
        raise HTTPException(404, "Signature file not found")
    return FileResponse(
        row["signature_path"],
        filename=os.path.basename(row["signature_path"]),
        media_type="application/octet-stream"
    )
