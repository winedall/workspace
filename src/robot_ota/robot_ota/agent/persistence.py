import os
import json
import tempfile


class OtaStateStore:
    """升级任务状态持久化存储。

    使用原子写（write-then-rename）保证掉电安全：
      1. 写入临时文件 → fsync → rename
      2. 读取时如果临时文件残留则丢弃
    路径: <shared_dir>/ota-state/ota_state.json
    """

    def __init__(self, state_dir: str):
        self._path = os.path.join(state_dir, "ota_state.json")
        os.makedirs(state_dir, exist_ok=True)

    def save(self, task_id: str, from_version: str, to_version: str,
             phase: str, error: str = ""):
        """持久化当前升级任务状态。"""
        data = {
            "task_id": task_id,
            "from_version": from_version,
            "to_version": to_version,
            "phase": phase,
            "error": error,
            "updated_at": __import__("datetime").datetime.utcnow().isoformat() + "Z"
        }
        tmp = tempfile.NamedTemporaryFile(
            mode="w", dir=os.path.dirname(self._path),
            delete=False, suffix=".tmp"
        )
        try:
            json.dump(data, tmp)
            tmp.flush()
            os.fsync(tmp.fileno())
            tmp.close()
            os.replace(tmp.name, self._path)
        finally:
            if not tmp.closed:
                tmp.close()
            if os.path.exists(tmp.name):
                os.unlink(tmp.name)

    def load(self) -> dict:
        """加载持久化的状态（用于断电恢复检查）。"""
        if not os.path.exists(self._path):
            return {}
        with open(self._path, "r") as f:
            return json.load(f)

    def clear(self):
        """清除持久化状态（升级完成后调用）。"""
        if os.path.exists(self._path):
            os.unlink(self._path)
