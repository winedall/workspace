import subprocess


def _run_systemctl(*args: str, timeout: int = 30) -> subprocess.CompletedProcess:
    """执行 systemctl 命令的底层封装。"""
    return subprocess.run(
        ["systemctl"] + list(args),
        capture_output=True, timeout=timeout
    )


def stop_service(service_name: str) -> bool:
    """停止 systemd 服务。"""
    try:
        result = _run_systemctl("stop", service_name)
        return result.returncode == 0
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


def start_service(service_name: str) -> bool:
    """启动 systemd 服务。"""
    try:
        result = _run_systemctl("start", service_name)
        return result.returncode == 0
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


def is_active(service_name: str) -> bool:
    """检查 systemd 服务是否处于 active 状态。"""
    try:
        result = _run_systemctl("is-active", service_name)
        return result.returncode == 0
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


def get_service_status(service_name: str) -> str:
    """获取 systemd 服务的运行状态字符串。"""
    try:
        result = _run_systemctl("is-active", service_name)
        return result.stdout.decode().strip()
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return "unknown"
