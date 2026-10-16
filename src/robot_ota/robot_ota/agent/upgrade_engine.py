import os
import shutil
import logging
import threading
import subprocess
import time
import urllib.request
import json
import pwd

from typing import Callable, Optional

from agent.config import AgentConfig
from agent.state_machine import UpgradeStateMachine, UpgradeState
from agent.package import (
    verify_archive, verify_files, safe_extract,
    load_manifest, create_release_dir, set_tree_owner, switch_symlink, cleanup_old_versions,
)
from agent.systemd_ctl import stop_service, start_service, is_active
from agent.healthcheck import check_robot_healthy, monitor_health, parse_robot_state
from agent.persistence import OtaStateStore


class UpgradeError(Exception):
    """升级过程中的可预期错误。"""
    pass


TERMINAL_STATES = {
    UpgradeState.SUCCEEDED,
    UpgradeState.FAILED,
    UpgradeState.ROLLED_BACK,
    UpgradeState.RECOVERY_REQUIRED,
    UpgradeState.CANCELLED,
}


def _read_manifest_version(manifest_path: str) -> str:
    if not os.path.exists(manifest_path):
        return ""
    try:
        with open(manifest_path, "r") as f:
            manifest = json.load(f)
        return manifest.get("version", "") or ""
    except Exception:
        logging.getLogger(__name__).warning("Failed to read manifest %s", manifest_path, exc_info=True)
        return ""


class UpgradeEngine:
    """升级引擎，协调完整的升级或回滚流程。

    升级流程（8 阶段）：
      下载并校验 → 等待安全状态 → 停止服务 → 切换软链接
      → 启动服务 → 健康检查 → 完成

    回滚触发条件：
      切换后任一阶段失败（启动失败 / 健康检查失败）
    """

    def __init__(
        self,
        config: AgentConfig,
        state_store: OtaStateStore,
        status_reporter: Optional[Callable[[dict], None]] = None,
    ):
        self._config = config
        self._store = state_store
        self._sm: Optional[UpgradeStateMachine] = None
        self._status_reporter = status_reporter
        self._service_name = "zl_robot.service"
        self._cached_ros_env: Optional[dict] = None
        # 取消事件，支持在下载/等待等阶段取消升级
        self._cancel_event = threading.Event()

    @property
    def state_machine(self) -> Optional[UpgradeStateMachine]:
        return self._sm

    def _detect_current_version(self) -> str:
        """通过 current 软链接检测当前运行的版本号。"""
        link_path = os.path.join(self._config.agent_root, "current")
        if os.path.islink(link_path):
            target = os.readlink(link_path)
            version = _read_manifest_version(os.path.join(target, "manifest.json"))
            if version:
                return version
            if os.path.isdir(target):
                for entry in os.listdir(target):
                    version = _read_manifest_version(os.path.join(target, entry, "manifest.json"))
                    if version:
                        return version
            return os.path.basename(os.path.normpath(target))
        return ""

    def run(self, task_id: str, from_version: str, to_version: str, download_url: str):
        """执行升级任务（在独立线程中运行）。"""
        # 清除取消事件
        self._cancel_event.clear()

        # 创建状态机
        sm = UpgradeStateMachine(task_id, from_version, to_version)
        self._sm = sm
        self._persist(sm)

        # 记录升级开始时间
        log = logging.getLogger(__name__)
        log.info("Upgrade %s: %s -> %s", task_id, from_version, to_version)

        is_fresh = not from_version or from_version == "0.0.0"
        if is_fresh:
            log.info("Fresh install detected, skipping safety check")

        try:
            # 如果目标版本已存在（回滚场景），跳过下载校验
            if self._is_valid_release(to_version):
                log.info("Target version %s already exists locally, skipping download", to_version)
                sm.transferring()
                self._persist(sm)
                sm.verifying()
                self._persist(sm)
                sm.staged()
                self._persist(sm)
            else:
                self._download_and_verify(sm, download_url, to_version)
            # 等待安全状态（首次部署跳过）
            if not is_fresh:
                self._wait_for_safe_state(sm)
            else:
                sm.safety_checking()
                self._persist(sm)
            # 首次部署也要停止可能残留且处于自动重启中的 systemd 服务。
            self._stop_service(sm)
            # 等待 5 秒，确保 ROS daemon / DDS 资源完全释放，避免新实例启动时端口或 SHM 冲突
            time.sleep(5)
            # 切换软链接
            self._switch_symlink(sm, to_version)
            # 执行新版本附带的预启动脚本（如 udev/setcap/ldconfig 等系统配置）
            self._run_prestart_script(to_version)
            # 安装当前版本的服务文件到 systemd
            self._install_service_file()
            # 启动服务
            self._start_service(sm)
            # 健康检查
            self._health_check(sm)

            # 切换到成功状态
            sm.succeeded()
            self._persist(sm)
            log.info("Upgrade %s succeeded", task_id)

            # 清理旧版本目录
            cleanup_old_versions(
                self._config.releases_dir, to_version,
                previous_version=from_version,
                max_retain=self._config.max_retain_versions,
            )

        except UpgradeError as e:
            # 仅在切换后（STARTING / HEALTH_CHECKING）才需要回滚
            if sm.state in (UpgradeState.SWITCHING, UpgradeState.STARTING, UpgradeState.HEALTH_CHECKING):
                self._rollback(sm, str(e))
            else:
                # 其他阶段失败，直接失败
                sm.fail(str(e))
                log.error("Upgrade %s failed: %s", task_id, e)
            self._persist(sm)

        except Exception as e:
            log.exception("Unexpected error during upgrade")
            # 仅在切换后（STARTING / HEALTH_CHECKING）才需要回滚
            if sm.state in (UpgradeState.SWITCHING, UpgradeState.STARTING, UpgradeState.HEALTH_CHECKING):
                self._rollback(sm, str(e))
            else:
                # 其他阶段失败，直接失败
                sm.fail(str(e))
            self._persist(sm)

        finally:
            if sm.state in TERMINAL_STATES:
                self._store.clear()
            self._sm = None

    def rollback_task(self, task_id: str):
        """手动回滚到前一个版本。

        从持久化状态读取 from_version 作为回滚目标。
        """
        # 加载持久化状态
        saved = self._store.load()
        if not saved or saved.get("task_id") != task_id:
            raise UpgradeError(f"No task found for rollback: {task_id}")

        # 清除取消事件
        self._cancel_event.clear()
        # 创建状态机
        sm = UpgradeStateMachine(task_id, saved["from_version"], saved["to_version"])
        self._sm = sm
        log = logging.getLogger(__name__)
        log.info("Manual rollback %s: %s -> %s", task_id, saved["to_version"], saved["from_version"])

        try:
            if not self._is_valid_release(sm.from_version):
                raise UpgradeError("no valid previous release available for rollback")
            # 切换到回滚状态
            sm.rollback("manual rollback")
            self._persist(sm)

            # 切换软链接到前一个版本
            link_path = os.path.join(self._config.agent_root, "current")
            target = os.path.join(self._config.releases_dir, sm.from_version)
            switch_symlink(link_path, target)
            self._cached_ros_env = None

            # 安装回滚目标版本的服务文件
            self._install_service_file()

            # 启动服务
            if not start_service(self._service_name):
                raise UpgradeError("failed to start service during rollback")

            # 执行健康检查
            check_fn = lambda: check_robot_healthy(self._service_name)
            if monitor_health(check_fn, self._config.healthcheck_interval, self._config.stable_window, self._config.healthcheck_timeout):
                # 切换到回滚成功状态
                sm.rolled_back()
                log.info("Rollback %s succeeded", task_id)
            else:
                # 切换到回滚失败状态
                sm.recovery_required("rollback health check failed")
        except Exception as e:
            log.exception("Rollback failed")
            # 切换到回滚失败状态
            sm.recovery_required(f"rollback failed: {e}")
        finally:
            self._persist(sm)
            if sm.state in TERMINAL_STATES:
                self._store.clear()
            self._sm = None

    def cancel(self):
        """取消当前升级任务。"""
        self._cancel_event.set()
        if self._sm:
            self._sm.cancel()
            self._persist(self._sm)

    def _check_cancel(self):
        """在耗时操作前检查取消标志。"""
        if self._cancel_event.is_set():
            raise UpgradeError("task cancelled")

    # ── 各阶段实现 ──────────────────────────────────────

    def _download_and_verify(self, sm: UpgradeStateMachine, download_url: str, version: str):
        """阶段 1-3: 下载发布包 → 校验签名 → 解包 → 校验文件哈希。"""
        # 切换到下载状态
        sm.transferring()
        self._persist(sm)
        self._check_cancel()

        # 下载发布包
        archive_path = self._download(download_url)

        # 切换到校验状态
        sm.verifying()
        self._persist(sm)
        self._check_cancel()

        # 校验签名
        sig_url = download_url + ".sig"
        sig_path = self._download(sig_url)
        if not verify_archive(archive_path, sig_path, self._config.public_key_path):
            raise UpgradeError("archive signature verification failed")

        # 创建发布包目录（同版本重部署时清理旧内容）
        pkg_dir = create_release_dir(self._config.releases_dir, version)
        if os.path.exists(pkg_dir):
            shutil.rmtree(pkg_dir)
            os.makedirs(pkg_dir, exist_ok=True)

        # 解包发布包
        if not safe_extract(archive_path, pkg_dir):
            raise UpgradeError("archive extraction failed")

        # 加载发布包清单
        manifest = load_manifest(pkg_dir)
        if manifest is None:
            raise UpgradeError("manifest not found in extracted package")

        # 校验文件哈希
        if not verify_files(pkg_dir, manifest):
            raise UpgradeError("file hash verification failed")

        owner = os.stat(self._config.agent_root)
        set_tree_owner(pkg_dir, owner.st_uid, owner.st_gid)

        # 切换到已准备状态
        sm.staged()
        self._persist(sm)

    def _download(self, url: str) -> str:
        """下载文件到本地 downloads 目录。"""
        filename = os.path.basename(url)
        dest = os.path.join(self._config.downloads_dir, filename)
        os.makedirs(self._config.downloads_dir, exist_ok=True)
        urllib.request.urlretrieve(url, dest)
        return dest

    def _wait_for_safe_state(self, sm: UpgradeStateMachine):
        """阶段 4: 等待机器人进入 STATE_IDLE 安全状态。

        通过 ros2 topic echo 读取 /robot/state 话题，
        值为 2 表示 STATE_IDLE。

        如果机器人服务未运行（程序已崩溃/被杀），
        则跳过安全等待，直接允许升级修复。
        """
        # 切换到安全检查状态
        sm.safety_checking()
        self._persist(sm)

        log = logging.getLogger(__name__)

        if not is_active(self._service_name):
            log.warning("zl_robot.service is not running, skipping STATE_IDLE wait")
            return

        # 等待机器人进入安全状态（非运动状态即放行）
        log.info("Waiting for robot safe state")
        timeout = 30
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            self._check_cancel()
            try:
                result = subprocess.run(
                    ["ros2", "topic", "echo", "/robot/state", "--once", "--field", "/robot/state", "--no-daemon"],
                    capture_output=True, timeout=5, text=True, env=self._ros_environment()
                )
                if result.returncode == 0:
                    state_value = parse_robot_state(result.stdout)
                    # 只有运动状态才阻塞升级，其余状态（IDLE/FAULT/DAMP等）均放行
                    if state_value not in ("3", "4", "5", "6", "7", "8", "9"):
                        log.info("Robot state=%s, safe to upgrade", state_value)
                        return
                    log.info("Robot in motion state=%s, waiting", state_value)
            except Exception as e:
                log.warning("Failed to read /robot/state: %s", e)
            time.sleep(2)
        raise UpgradeError("robot did not enter safe state within timeout")

    def _stop_service(self, sm: UpgradeStateMachine):
        """阶段 5: 停止机器人服务。"""
        # 切换到停止状态
        sm.stopping()
        self._persist(sm)

        # 停止服务
        if not stop_service(self._service_name):
            raise UpgradeError(f"failed to stop service {self._service_name}")

    def _switch_symlink(self, sm: UpgradeStateMachine, version: str):
        """阶段 6: 原子切换 current 软链接到新版本。"""
        # 切换到切换状态
        sm.switching()
        self._persist(sm)

        # 切换软链接到新版本
        target = os.path.join(self._config.releases_dir, version)
        link_path = os.path.join(self._config.agent_root, "current")
        switch_symlink(link_path, target)
        self._cached_ros_env = None

    def _run_prestart_script(self, version: str):
        """执行新版本附带的预启动脚本。

        在切换软链接之后、启动新服务之前执行。
        脚本路径：{releases_dir}/{version}/scripts/install.sh

        注入真实用户的 HOME/USER 环境变量，确保脚本中 $HOME、~ 路径
        指向用户目录而非 root。
        """
        script = os.path.join(self._config.releases_dir, version, "scripts", "install.sh")
        if not os.path.isfile(script):
            return
        log = logging.getLogger(__name__)
        log.info("Running pre-start script: %s", script)

        # 从 agent_root 目录的 owner 获取真实用户信息（首次安装时已 chown）
        env = os.environ.copy()
        try:
            uid = os.stat(self._config.agent_root).st_uid
            pw = pwd.getpwuid(uid)
            env["HOME"] = pw.pw_dir
            env["USER"] = pw.pw_name
            env["SUDO_USER"] = pw.pw_name
            log.info("Injected user env: USER=%s HOME=%s", pw.pw_name, pw.pw_dir)
        except (KeyError, OSError) as e:
            log.warning("Cannot resolve agent_root owner, script may use root home: %s", e)

        proc = subprocess.run(
            ["bash", script],
            capture_output=True, text=True, timeout=120,
            env=env,
        )
        if proc.returncode != 0:
            raise UpgradeError(
                f"pre-start script failed (exit={proc.returncode}): {proc.stderr.strip()}"
            )
        log.info("Pre-start script completed")

    def _install_service_file(self):
        """安装 current 版本中的 robot.service 到 systemd。

        用 __AGENT_ROOT__ 占位符替换为实际路径后，写入 /etc/systemd/system/。
        """
        log = logging.getLogger(__name__)
        service_src = os.path.join(
            self._config.agent_root, "current",
            "scripts", "service", "zl_robot.service"
        )
        if not os.path.exists(service_src):
            service_src = os.path.join(
                self._config.agent_root, "ota_agent", "zl_robot.service"
            )
            if not os.path.exists(service_src):
                raise UpgradeError("zl_robot.service not found in release or agent installation")
            log.info("Release has no zl_robot.service; using agent template")

        with open(service_src) as f:
            content = f.read()
        content = content.replace("__AGENT_ROOT__", self._config.agent_root)
        content = content.replace("/opt/zl_robot", self._config.agent_root)
        service_user = pwd.getpwuid(os.stat(self._config.agent_root).st_uid).pw_name
        content = content.replace("__USER__", service_user)

        proc = subprocess.run(
            ["tee", "/etc/systemd/system/zl_robot.service"],
            input=content, text=True, capture_output=True
        )
        if proc.returncode != 0:
            raise UpgradeError(f"failed to write zl_robot.service: {proc.stderr.strip()}")

        proc = subprocess.run(["systemctl", "daemon-reload"], capture_output=True, text=True)
        if proc.returncode != 0:
            raise UpgradeError(f"systemctl daemon-reload failed: {proc.stderr.strip()}")

        proc = subprocess.run(["systemctl", "enable", "zl_robot.service"], capture_output=True, text=True)
        if proc.returncode != 0:
            log.warning("systemctl enable zl_robot.service failed: %s", proc.stderr.strip())

        log.info("zl_robot.service installed (agent_root=%s)", self._config.agent_root)

    def _start_service(self, sm: UpgradeStateMachine):
        """阶段 7: 启动机器人服务。"""
        # 切换到启动状态
        sm.starting()
        self._persist(sm)

        # 启动服务
        if not start_service(self._service_name):
            raise UpgradeError(f"failed to start service {self._service_name}")

    def _health_check(self, sm: UpgradeStateMachine):
        """阶段 8: 对新版本执行健康检查。

        Agent 内置检查：服务 active + ROS 话题 state == 2。
        连续 stable_window 次成功才算通过。
        """
        # 切换到健康检查状态
        sm.health_checking()
        self._persist(sm)

        # 执行健康检查
        check_fn = lambda: check_robot_healthy(self._service_name)
        if not monitor_health(check_fn, self._config.healthcheck_interval, self._config.stable_window, self._config.healthcheck_timeout):
            raise UpgradeError("health check failed")

    def _rollback(self, sm: UpgradeStateMachine, error: str):
        """自动回滚：切换到前一个版本并执行健康检查。"""
        log = logging.getLogger(__name__)

        # 切换到回滚状态
        sm.rollback(error)
        self._persist(sm)
        log.warning("Rolling back upgrade for task %s", sm.task_id)

        try:
            if not self._is_valid_release(sm.from_version):
                stop_service(self._service_name)
                link_path = os.path.join(self._config.agent_root, "current")
                if os.path.islink(link_path):
                    os.unlink(link_path)
                sm.recovery_required(
                    f"{error}; no valid previous release available for rollback"
                )
                self._persist(sm)
                return

            # 切换软链接到前一个版本
            link_path = os.path.join(self._config.agent_root, "current")
            target = os.path.join(self._config.releases_dir, sm.from_version)
            switch_symlink(link_path, target)
            self._cached_ros_env = None

            # 恢复旧版本随包提供的 systemd unit。
            self._install_service_file()

            # 启动服务
            if not start_service(self._service_name):
                raise UpgradeError("failed to start previous release during rollback")

            # 执行健康检查
            check_fn = lambda: check_robot_healthy(self._service_name)
            if monitor_health(check_fn, self._config.healthcheck_interval, self._config.stable_window, self._config.healthcheck_timeout):
                # 切换到回滚成功状态
                sm.rolled_back()
            else:
                # 切换到回滚失败状态
                sm.recovery_required("rollback health check failed")
        except Exception as e:
            log.exception("Rollback execution failed")
            # 切换到回滚失败状态
            sm.recovery_required(f"rollback failed: {e}")
        self._persist(sm)

    def _is_valid_release(self, version: str) -> bool:
        if not version or version == "0.0.0":
            return False
        release_dir = os.path.join(self._config.releases_dir, version)
        return (
            os.path.isdir(release_dir)
            and load_manifest(release_dir) is not None
            and os.path.isfile(os.path.join(release_dir, "scripts", "run.sh"))
        )

    def _ros_environment(self) -> dict:
        if self._cached_ros_env is not None:
            return self._cached_ros_env.copy()

        release_setup = os.path.join(
            self._config.agent_root, "current", "install", "setup.bash"
        )
        proc = subprocess.run(
            [
                "bash", "-c",
                'source /opt/ros/humble/setup.bash && source "$1" && env',
                "bash", release_setup,
            ],
            capture_output=True,
            text=True,
            timeout=10,
            env=os.environ.copy(),
        )
        if proc.returncode != 0:
            raise UpgradeError(f"failed to load ROS environment: {proc.stderr.strip()}")

        env = os.environ.copy()
        for line in proc.stdout.splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                env[key] = value
        env["ROS_DISTRO"] = "humble"
        env["ROS_VERSION"] = "2"
        env["ROS_LOCALHOST_ONLY"] = "0"
        env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
        ros_log_dir = os.path.join(self._config.logs_dir, "ros")
        os.makedirs(ros_log_dir, exist_ok=True)
        env["ROS_LOG_DIR"] = ros_log_dir
        env["CYCLONEDDS_URI"] = "file://" + os.path.join(
            self._config.agent_root, "current", "config", "cyclonedds_bridge_whitelist.xml"
        )
        self._cached_ros_env = env
        return env.copy()

    def _persist(self, sm: UpgradeStateMachine):
        """持久化状态机快照（用于断电恢复）。"""
        self._store.save(
            sm.task_id, sm.from_version, sm.to_version,
            sm.state.value, sm.error or ""
        )
        if self._status_reporter:
            try:
                current_version = sm.to_version if sm.state == UpgradeState.SUCCEEDED else ""
                self._status_reporter({
                    "task_id": sm.task_id,
                    "status": sm.state.value,
                    "progress": _progress_for_state(sm.state),
                    "error": sm.error or "",
                    "current_version": current_version,
                })
            except Exception:
                logging.getLogger(__name__).warning("Failed to report OTA task status", exc_info=True)


def _progress_for_state(state: UpgradeState) -> int:
    progress = {
        UpgradeState.CREATED: 0,
        UpgradeState.TRANSFERRING: 10,
        UpgradeState.VERIFYING: 25,
        UpgradeState.STAGED: 40,
        UpgradeState.SAFETY_CHECKING: 50,
        UpgradeState.STOPPING: 60,
        UpgradeState.SWITCHING: 70,
        UpgradeState.STARTING: 80,
        UpgradeState.HEALTH_CHECKING: 90,
        UpgradeState.SUCCEEDED: 100,
        UpgradeState.FAILED: 100,
        UpgradeState.ROLLING_BACK: 90,
        UpgradeState.ROLLED_BACK: 100,
        UpgradeState.RECOVERY_REQUIRED: 100,
        UpgradeState.CANCELLED: 100,
    }
    return progress.get(state, 0)
