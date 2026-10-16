from fastapi import APIRouter, Depends, Query
from ..database import Database, get_db_path
from ..schemas import AuditLogResponse
from ..auth import verify_token

router = APIRouter(prefix="/api/v1/audit", tags=["audit"])


def get_db() -> Database:
    return Database(get_db_path())


@router.get("/")
def list_audit_logs(limit: int = Query(20, ge=1, le=500), offset: int = Query(0, ge=0), _=Depends(verify_token)):
    """查询审计日志，按时间倒序排列，返回分页数据和总数。"""
    db = get_db()
    with db.connect() as conn:
        total = conn.execute("SELECT COUNT(*) FROM audit_logs").fetchone()[0]
        rows = conn.execute(
            "SELECT * FROM audit_logs ORDER BY created_at DESC, id DESC LIMIT ? OFFSET ?", (limit, offset)
        ).fetchall()
        return {"data": [dict(r) for r in rows], "total": total}
