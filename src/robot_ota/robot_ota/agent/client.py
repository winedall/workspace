import json
import logging
import urllib.request
from typing import Optional

from agent.config import AgentConfig


class ServerClient:
    """Small HTTP client used by the agent to report state and pull tasks."""

    def __init__(self, config: AgentConfig, timeout: float = 5.0):
        self._config = config
        self._timeout = timeout
        self._log = logging.getLogger(__name__)

    def heartbeat(
        self,
        current_version: str = "",
        agent_address: str = "",
        robot_online: bool = False,
        robot_state: str = "",
    ) -> bool:
        payload = {
            "current_version": current_version,
            "agent_version": "1.0.0",
            "agent_address": agent_address,
            "robot_online": robot_online,
            "robot_state": robot_state,
        }
        return self._post("/api/v1/agent/heartbeat", payload) is not None

    def next_task(self) -> Optional[dict]:
        data = self._request("GET", "/api/v1/agent/tasks/next")
        if not data or data.get("task") is None and "task_id" not in data:
            return None
        return data

    def report_status(
        self,
        task_id: str,
        status: str,
        progress: int = 0,
        error: str = "",
        current_version: str = "",
    ) -> bool:
        payload = {
            "status": status,
            "progress": progress,
            "error": error,
            "current_version": current_version,
        }
        return self._post(f"/api/v1/agent/tasks/{task_id}/status", payload) is not None

    def _post(self, path: str, payload: dict) -> Optional[dict]:
        return self._request("POST", path, payload)

    def _request(self, method: str, path: str, payload: Optional[dict] = None) -> Optional[dict]:
        if not self._config.server_url or not self._config.device_id or not self._config.device_token:
            return None

        url = self._config.server_url.rstrip("/") + path
        data = None
        headers = {
            "X-Device-Id": self._config.device_id,
            "Authorization": f"Bearer {self._config.device_token}",
        }
        if payload is not None:
            data = json.dumps(payload).encode("utf-8")
            headers["Content-Type"] = "application/json"

        req = urllib.request.Request(url, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(req, timeout=self._timeout) as resp:
                raw = resp.read()
                if not raw:
                    return {}
                return json.loads(raw.decode("utf-8"))
        except Exception as e:
            self._log.warning("Server request failed: %s %s: %s", method, url, e)
            return None
