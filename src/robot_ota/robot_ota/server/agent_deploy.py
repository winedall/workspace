"""Agent SSH 部署模块。

通过 SSH 连接到目标设备，上传并安装 Agent。
"""
import os
from dataclasses import dataclass
from typing import Callable, Optional

from .database import Database


@dataclass
class AgentDeployConfig:
    """SSH 部署配置。"""
    target_host: str  # 目标设备 IP 地址
    ssh_user: str = "root"  # SSH 用户名
    ssh_port: int = 22  # SSH 端口
    password: str = ""  # SSH 密码（与 private_key_path 二选一）
    private_key_path: str = ""  # SSH 私钥路径
    agent_root: str = ""  # Agent 安装根目录


def create_agent_deployment(
    db: Database,
    device_id: str,
    target_host: str,
    ssh_user: str,
    ssh_port: int,
) -> int:
    """创建部署记录，返回记录 ID。

    初始状态为 pending，后续由 SshAgentDeployer 更新。
    """
    with db.connect() as conn:
        conn.execute(
            """INSERT INTO agent_deployments
               (device_id, target_host, ssh_user, ssh_port, status)
               VALUES (?, ?, ?, ?, 'pending')""",
            (device_id, target_host, ssh_user, ssh_port),
        )
        conn.commit()
        return conn.execute("SELECT last_insert_rowid()").fetchone()[0]


def get_agent_deployment(db: Database, deployment_id: int) -> Optional[dict]:
    """根据 ID 查询部署记录。"""
    with db.connect() as conn:
        row = conn.execute("SELECT * FROM agent_deployments WHERE id = ?", (deployment_id,)).fetchone()
        return dict(row) if row else None


def latest_agent_deployment(db: Database, device_id: str) -> Optional[dict]:
    """查询指定设备的最新部署记录。"""
    with db.connect() as conn:
        row = conn.execute(
            "SELECT * FROM agent_deployments WHERE device_id = ? ORDER BY created_at DESC, id DESC LIMIT 1",
            (device_id,),
        ).fetchone()
        return dict(row) if row else None


class SshAgentDeployer:
    """通过 SSH 执行 Agent 远程部署。

    使用 paramiko 库实现 SSH 连接和命令执行。
    支持密码和私钥两种认证方式。
    """

    def __init__(self, db: Database, ssh_client_factory: Optional[Callable] = None):
        self._db = db
        self._ssh_client_factory = ssh_client_factory or self._default_ssh_client

    def deploy(self, deployment_id: int, package_path: str, config: AgentDeployConfig):
        """执行部署主流程。

        流程：SSH连接 → 准备目录 → 上传包 → 解压 → 执行安装脚本
        """
        client = None
        try:
            self._set_status(deployment_id, "running", "开始 SSH 部署", started=True)
            if not os.path.exists(package_path):
                raise RuntimeError(f"部署包不存在: {package_path}")

            # 建立 SSH 连接
            client = self._ssh_client_factory()
            self._connect(client, config)

            # 清理并创建远程临时目录
            self._append_log(deployment_id, "准备远程目录")
            self._run(client, "rm -rf /tmp/ota_agent && mkdir -p /tmp/ota_agent")

            # 上传部署包
            self._append_log(deployment_id, "上传部署包")
            sftp = client.open_sftp()
            try:
                sftp.put(package_path, "/tmp/ota_agent.tar.gz")
            finally:
                sftp.close()

            # 解压部署包
            self._append_log(deployment_id, "解压部署包")
            self._run(client, "tar -xzf /tmp/ota_agent.tar.gz -C /tmp/ota_agent")

            # 执行安装脚本
            self._append_log(deployment_id, "安装 Agent")
            install_cmd = "bash /tmp/ota_agent/install.sh"
            if config.agent_root:
                install_cmd += f" --agent-root {config.agent_root}"
            if config.password:
                # 使用 sudo -S 从 stdin 读取密码
                self._run(
                    client,
                    f"sudo -S -p '' {install_cmd}",
                    stdin_text=config.password + "\n",
                )
            else:
                self._run(client, install_cmd)

            self._set_status(deployment_id, "succeeded", "Agent 部署成功", finished=True)
        except Exception as e:
            self._set_status(deployment_id, "failed", f"Agent 部署失败: {e}", error=str(e), finished=True)
        finally:
            if client is not None:
                try:
                    client.close()
                except Exception:
                    pass

    def _default_ssh_client(self):
        """创建默认 paramiko SSH 客户端。"""
        try:
            import paramiko
        except ImportError as e:
            raise RuntimeError("需要 paramiko 库，请安装服务端依赖") from e
        client = paramiko.SSHClient()
        # 自动接受未知主机密钥
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        return client

    def _connect(self, client, config: AgentDeployConfig):
        """建立 SSH 连接。"""
        kwargs = {
            "hostname": config.target_host,
            "port": config.ssh_port,
            "username": config.ssh_user,
            "timeout": 15,
        }
        if config.private_key_path:
            kwargs["key_filename"] = config.private_key_path
        if config.password:
            kwargs["password"] = config.password
        client.connect(**kwargs)

    def _run(self, client, command: str, stdin_text: str = ""):
        """执行远程命令。

        通过 stdin 传递密码（用于 sudo），检查退出码，非零则抛出异常。
        """
        stdin, stdout, stderr = client.exec_command(command)
        if stdin_text and stdin is not None:
            stdin.write(stdin_text)
            stdin.flush()
        status = stdout.channel.recv_exit_status()
        out = stdout.read().decode(errors="replace").strip()
        err = stderr.read().decode(errors="replace").strip()
        if status != 0:
            detail = err or out or f"exit status {status}"
            raise RuntimeError(f"远程命令执行失败: {command}: {detail}")

    def _append_log(self, deployment_id: int, line: str):
        """追加部署日志。"""
        with self._db.connect() as conn:
            conn.execute(
                """UPDATE agent_deployments
                   SET log = log || ? || char(10), updated_at = datetime('now')
                   WHERE id = ?""",
                (line, deployment_id),
            )
            conn.commit()

    def _set_status(
        self,
        deployment_id: int,
        status: str,
        line: str,
        error: str = "",
        started: bool = False,
        finished: bool = False,
    ):
        """更新部署状态和日志。

        started=True 时记录开始时间（仅首次），finished=True 时记录结束时间。
        """
        started_sql = ", started_at = CASE WHEN started_at = '' THEN datetime('now') ELSE started_at END" if started else ""
        finished_sql = ", finished_at = datetime('now')" if finished else ""
        with self._db.connect() as conn:
            conn.execute(
                f"""UPDATE agent_deployments
                    SET status = ?,
                        error = ?,
                        log = log || ? || char(10),
                        updated_at = datetime('now')
                        {started_sql}
                        {finished_sql}
                    WHERE id = ?""",
                (status, error, line, deployment_id),
            )
            conn.commit()
