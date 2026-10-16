import os
import sys
import logging
import threading
import time

from agent.config import AgentConfig
from agent.client import ServerClient
from agent.healthcheck import read_robot_state
from agent.persistence import OtaStateStore
from agent.upgrade_engine import UpgradeEngine
from agent.server import run_server


TERMINAL_PHASES = {
    "SUCCEEDED",
    "FAILED",
    "ROLLED_BACK",
    "RECOVERY_REQUIRED",
    "CANCELLED",
}


def setup_logging(logs_dir: str):
    """配置日志输出：同时写入文件和控制台。"""
    log_file = os.path.join(logs_dir, "ota_agent.log")
    os.makedirs(logs_dir, exist_ok=True)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        handlers=[
            logging.FileHandler(log_file),
            logging.StreamHandler(sys.stdout),
        ],
    )


def _start_server_sync_loop(config: AgentConfig, engine: UpgradeEngine, client: ServerClient):
    """Start background heartbeat and task-pull loop."""
    log = logging.getLogger(__name__)

    def loop():
        while True:
            try:
                current_version = engine._detect_current_version()
                try:
                    robot_state = read_robot_state(engine._ros_environment(), timeout=5)
                except Exception:
                    robot_state = ""
                client.heartbeat(
                    current_version,
                    robot_online=bool(robot_state),
                    robot_state=robot_state,
                )

                if engine.state_machine is None:
                    task = client.next_task()
                    if task:
                        task_id = task["task_id"]
                        version = task["version"]
                        download_url = task["download_url"]
                        from_version = current_version
                        log.info("Pulled OTA task %s -> %s", task_id, version)
                        thread = threading.Thread(
                            target=engine.run,
                            args=(task_id, from_version, version, download_url),
                            daemon=True,
                        )
                        thread.start()
            except Exception:
                log.warning("OTA server sync loop failed", exc_info=True)

            time.sleep(2)

    thread = threading.Thread(target=loop, daemon=True)
    thread.start()
    return thread


def _load_pending_state(state_store: OtaStateStore, log):
    saved = state_store.load()
    if not saved:
        return {}

    phase = saved.get("phase", "")
    if phase in TERMINAL_PHASES:
        log.info(
            "Clearing completed OTA task %s in state %s",
            saved.get("task_id"), phase,
        )
        state_store.clear()
        return {}

    log.warning(
        "Found incomplete task %s in state %s",
        saved.get("task_id"), phase,
    )
    return saved


def main():
    """OTA Agent 入口函数。

    流程：
      1. 加载配置文件
      2. 初始化目录结构和持久化存储
      3. 检查是否有未完成的任务（断电恢复场景）
      4. 创建 UpgradeEngine 并启动 HTTP 服务
    """
    config = AgentConfig()
    setup_logging(config.logs_dir)
    log = logging.getLogger(__name__)

    os.makedirs(config.ota_state_dir, exist_ok=True)
    os.makedirs(config.downloads_dir, exist_ok=True)
    os.makedirs(config.releases_dir, exist_ok=True)

    state_store = OtaStateStore(config.ota_state_dir)

    # 检查是否有未完成的升级任务（掉电恢复场景）
    _load_pending_state(state_store, log)

    client = ServerClient(config)

    def report_status(status: dict):
        client.report_status(
            status["task_id"],
            status["status"],
            status["progress"],
            status["error"],
            status["current_version"],
        )

    engine = UpgradeEngine(config, state_store, status_reporter=report_status)
    _start_server_sync_loop(config, engine, client)

    log.info(
        "Starting OTA agent on %s:%s (device=%s)",
        config.listen_addr, config.listen_port, config.device_id,
    )
    run_server(config, engine, state_store)


if __name__ == "__main__":
    main()
