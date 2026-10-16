from .database import Database, get_db_path


def write_audit_log(action: str, actor: str = "admin", target: str = "", detail: str = "", ip_address: str = ""):
    db = Database(get_db_path())
    with db.connect() as conn:
        conn.execute(
            "INSERT INTO audit_logs (action, actor, target, detail, ip_address) VALUES (?, ?, ?, ?, ?)",
            (action, actor, target, detail, ip_address),
        )
        conn.commit()
