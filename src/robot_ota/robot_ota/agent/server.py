import json
import logging
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

from agent.config import AgentConfig
from agent.upgrade_engine import UpgradeEngine, UpgradeError
from agent.persistence import OtaStateStore


def _send_json(handler, data, status=200):
    """发送 JSON 响应。"""
    body = json.dumps(data).encode()
    handler.send_response(status)
    handler.send_header("Content-Type", "application/json")
    handler.send_header("Content-Length", str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)


def _send_error(handler, status, message):
    """发送 JSON 错误响应。"""
    _send_json(handler, {"error": message}, status)


class OTAHandler(BaseHTTPRequestHandler):
    """Agent HTTP API 处理类。

    提供三个接口：
      GET  /v1/status          查询 Agent 和升级任务状态
      POST /v1/tasks/install   发起升级任务
      POST /v1/tasks/rollback  发起回滚任务
      POST /v1/tasks/{id}/cancel  取消升级任务
    """

    engine: UpgradeEngine = None
    config: AgentConfig = None
    valid_token: str = ""
    device_id: str = ""

    def log_message(self, format, *args):
        """将 HTTP 访问日志写入 Python logging 而非 stderr。"""
        logging.getLogger(__name__).info("%s - %s", self.client_address[0], format % args)

    def _authenticate(self) -> bool:
        """Bearer Token 认证校验。"""
        auth = self.headers.get("Authorization", "")
        if not self.valid_token:
            return True
        if not auth.startswith("Bearer ") or auth[7:] != self.valid_token:
            _send_error(self, 401, "Unauthorized")
            return False
        return True

    def _read_body(self) -> dict:
        """读取并解析 JSON 请求体。"""
        try:
            content_length = int(self.headers.get("Content-Length", 0))
            if content_length == 0:
                return {}
            raw = self.rfile.read(content_length)
            return json.loads(raw)
        except (ValueError, json.JSONDecodeError):
            return {}

    def do_GET(self):
        if not self._authenticate():
            return

        parsed = urlparse(self.path)
        if parsed.path == "/v1/status":
            self._handle_status()
        else:
            _send_error(self, 404, "Not Found")

    def do_POST(self):
        if not self._authenticate():
            return

        parsed = urlparse(self.path)
        path = parsed.path.rstrip("/")
        data = self._read_body()

        if path == "/v1/tasks/install":
            self._handle_install(data)
        elif path == "/v1/tasks/rollback":
            self._handle_rollback(data)
        elif path.startswith("/v1/tasks/") and path.endswith("/cancel"):
            task_id = path.split("/")[3]
            self._handle_cancel(task_id)
        else:
            _send_error(self, 404, "Not Found")

    def _handle_status(self):
        """查询当前状态：有任务进行中时返回任务详情，否则返回空闲。"""
        sm = self.engine.state_machine
        if sm:
            status_data = {
                "device_id": self.device_id,
                "upgrade_in_progress": True,
                "task_id": sm.task_id,
                "from_version": sm.from_version,
                "to_version": sm.to_version,
                "state": sm.state.value,
                "error": sm.error,
            }
        else:
            status_data = {
                "device_id": self.device_id,
                "upgrade_in_progress": False,
            }
        _send_json(self, status_data)

    def _handle_install(self, data: dict):
        """发起升级：在后台线程中执行 UpgradeEngine.run。"""
        if self.engine.state_machine is not None:
            _send_error(self, 409, "Upgrade already in progress")
            return

        task_id = data.get("task_id")
        version = data.get("version")
        download_url = data.get("download_url")

        if not all([task_id, version, download_url]):
            _send_error(self, 400, "Missing required fields: task_id, version, download_url")
            return

        from_version = self.engine._detect_current_version()

        thread = threading.Thread(
            target=self.engine.run,
            args=(task_id, from_version, version, download_url),
            daemon=True,
        )
        thread.start()

        _send_json(self, {"task_id": task_id, "status": "started"}, 202)

    def _handle_rollback(self, data: dict):
        """发起回滚：在后台线程中执行 UpgradeEngine.rollback_task。"""
        if self.engine.state_machine is not None:
            _send_error(self, 409, "Upgrade already in progress")
            return

        task_id = data.get("task_id")
        if not task_id:
            _send_error(self, 400, "Missing required field: task_id")
            return

        thread = threading.Thread(
            target=self.engine.rollback_task,
            args=(task_id,),
            daemon=True,
        )
        thread.start()

        _send_json(self, {"task_id": task_id, "status": "rollback_started"}, 202)

    def _handle_cancel(self, task_id: str):
        """取消指定任务。"""
        sm = self.engine.state_machine
        if not sm or sm.task_id != task_id:
            _send_error(self, 404, f"No active task: {task_id}")
            return
        self.engine.cancel()
        _send_json(self, {"task_id": task_id, "status": "cancelled"})


def run_server(config: AgentConfig, engine: UpgradeEngine, state_store: OtaStateStore):
    """启动 Agent HTTP 服务（阻塞，直到 Ctrl+C）。"""
    OTAHandler.engine = engine
    OTAHandler.config = config
    OTAHandler.valid_token = config.device_token
    OTAHandler.device_id = config.device_id

    server = HTTPServer((config.listen_addr, config.listen_port), OTAHandler)
    log = logging.getLogger(__name__)
    log.info("Listening on %s:%s", config.listen_addr, config.listen_port)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log.info("Shutting down")
        server.shutdown()
