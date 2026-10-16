import os
import sqlite3
import threading
from contextlib import contextmanager

from .config import Settings

DB_PATH = Settings().db_path


def get_db_path():
    return DB_PATH


class Database:
    """SQLite 数据库封装，提供线程安全的连接管理。

    - 使用 threading.local 为每个线程维护独立连接
    - 启用 WAL 模式提升并发读性能
    - 启用外键约束保证数据完整性
    """

    def __init__(self, db_path: str = None):
        self._path = db_path or DB_PATH
        os.makedirs(os.path.dirname(self._path), exist_ok=True)
        self._local = threading.local()

    @contextmanager
    def connect(self):
        """获取数据库连接（上下文管理器，自动提交/回滚）。"""
        if not hasattr(self._local, "conn") or self._local.conn is None:
            self._local.conn = sqlite3.connect(self._path)
            self._local.conn.row_factory = sqlite3.Row
            self._local.conn.execute("PRAGMA journal_mode=WAL")
            self._local.conn.execute("PRAGMA foreign_keys=ON")
        yield self._local.conn

    def init_db(self):
        """初始化数据库表结构（幂等，多次调用安全）。"""
        with self.connect() as conn:
            conn.executescript("""
                CREATE TABLE IF NOT EXISTS devices (
                    id TEXT PRIMARY KEY,
                    agent_address TEXT NOT NULL DEFAULT '',
                    token_hash TEXT NOT NULL DEFAULT '',
                    token TEXT NOT NULL DEFAULT '',
                    current_version TEXT NOT NULL DEFAULT '',
                    robot_state TEXT NOT NULL DEFAULT '',
                    last_robot_state_at TEXT NOT NULL DEFAULT '',
                    last_heartbeat TEXT NOT NULL DEFAULT '',
                    created_at TEXT NOT NULL DEFAULT (datetime('now')),
                    updated_at TEXT NOT NULL DEFAULT (datetime('now'))
                );
                CREATE TABLE IF NOT EXISTS releases (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    version TEXT NOT NULL,
                    product TEXT NOT NULL DEFAULT 'zl_robot',
                    platform TEXT NOT NULL DEFAULT 'linux-amd64',
                    archive_path TEXT NOT NULL,
                    archive_hash TEXT NOT NULL,
                    signature_path TEXT NOT NULL,
                    file_size INTEGER NOT NULL DEFAULT 0,
                    manifest TEXT NOT NULL DEFAULT '{}',
                    release_notes TEXT NOT NULL DEFAULT '',
                    created_by TEXT NOT NULL DEFAULT '',
                    created_at TEXT NOT NULL DEFAULT (datetime('now')),
                    UNIQUE(product, version, platform)
                );
                CREATE TABLE IF NOT EXISTS deployments (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    batch_id TEXT NOT NULL DEFAULT '',
                    device_id TEXT NOT NULL DEFAULT '',
                    release_id INTEGER NOT NULL DEFAULT 0,
                    release_version TEXT NOT NULL DEFAULT '',
                    from_version TEXT NOT NULL DEFAULT '',
                    status TEXT NOT NULL DEFAULT 'pending',
                    progress INTEGER NOT NULL DEFAULT 0,
                    error TEXT NOT NULL DEFAULT '',
                    created_by TEXT NOT NULL DEFAULT '',
                    created_at TEXT NOT NULL DEFAULT (datetime('now')),
                    updated_at TEXT NOT NULL DEFAULT (datetime('now')),
                    FOREIGN KEY (device_id) REFERENCES devices(id),
                    FOREIGN KEY (release_id) REFERENCES releases(id)
                );
                CREATE TABLE IF NOT EXISTS agent_deployments (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    device_id TEXT NOT NULL,
                    target_host TEXT NOT NULL,
                    ssh_user TEXT NOT NULL,
                    ssh_port INTEGER NOT NULL DEFAULT 22,
                    status TEXT NOT NULL DEFAULT 'pending',
                    log TEXT NOT NULL DEFAULT '',
                    error TEXT NOT NULL DEFAULT '',
                    started_at TEXT NOT NULL DEFAULT '',
                    finished_at TEXT NOT NULL DEFAULT '',
                    created_at TEXT NOT NULL DEFAULT (datetime('now')),
                    updated_at TEXT NOT NULL DEFAULT (datetime('now')),
                    FOREIGN KEY (device_id) REFERENCES devices(id)
                );
                CREATE TABLE IF NOT EXISTS audit_logs (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    action TEXT NOT NULL,
                    actor TEXT NOT NULL DEFAULT '',
                    target TEXT NOT NULL DEFAULT '',
                    detail TEXT NOT NULL DEFAULT '',
                    ip_address TEXT NOT NULL DEFAULT '',
                    created_at TEXT NOT NULL DEFAULT (datetime('now'))
                );
                CREATE TABLE IF NOT EXISTS release_uploads (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    task_id TEXT UNIQUE NOT NULL,
                    status TEXT NOT NULL DEFAULT 'pending',
                    progress INTEGER NOT NULL DEFAULT 0,
                    filename TEXT NOT NULL DEFAULT '',
                    version TEXT NOT NULL DEFAULT '',
                    product TEXT NOT NULL DEFAULT '',
                    platform TEXT NOT NULL DEFAULT '',
                    error TEXT NOT NULL DEFAULT '',
                    created_at TEXT NOT NULL DEFAULT (datetime('now')),
                    updated_at TEXT NOT NULL DEFAULT (datetime('now'))
                );
            """)
            for table, migrations in {
                "devices": {
                    "robot_state": "TEXT NOT NULL DEFAULT ''",
                    "last_robot_state_at": "TEXT NOT NULL DEFAULT ''",
                },
                "deployments": {
                    "batch_id": "TEXT NOT NULL DEFAULT ''",
                    "device_id": "TEXT NOT NULL DEFAULT ''",
                    "release_version": "TEXT NOT NULL DEFAULT ''",
                    "from_version": "TEXT NOT NULL DEFAULT ''",
                    "progress": "INTEGER NOT NULL DEFAULT 0",
                    "error": "TEXT NOT NULL DEFAULT ''",
                    "updated_at": "TEXT NOT NULL DEFAULT (datetime('now'))",
                },
            }.items():
                existing_columns = {
                    row["name"]
                    for row in conn.execute(f"PRAGMA table_info({table})").fetchall()
                }
                for column, definition in migrations.items():
                    if column not in existing_columns:
                        conn.execute(f"ALTER TABLE {table} ADD COLUMN {column} {definition}")
            conn.commit()
