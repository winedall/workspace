import subprocess
import time

from agent.systemd_ctl import is_active


def parse_robot_state(output: str) -> str:
    """Extract the first state value from ros2 topic echo YAML output."""
    for line in output.splitlines():
        value = line.strip()
        if value and value != "---":
            return value
    return ""


def read_robot_state(ros_env=None, timeout: float = 5) -> str:
    """Return the current /robot/state value, or empty string when unavailable."""
    try:
        result = subprocess.run(
            ["ros2", "topic", "echo", "/robot/state", "--once", "--field", "/robot/state", "--no-daemon"],
            capture_output=True, timeout=timeout, text=True, env=ros_env,
        )
        if result.returncode != 0:
            return ""
        return parse_robot_state(result.stdout)
    except Exception:
        return ""


def check_robot_healthy(service_name: str) -> bool:
    """Agent 内置健康检查：仅检查机器人服务是否 active。"""
    return is_active(service_name)


def monitor_health(check_fn, interval: float = 2.0,
                   stable_window: int = 10, timeout: int = 60) -> bool:
    """持续监测健康状态，直到满足稳定窗口或超时。

    Args:
      check_fn:      每次检查的可调用对象，返回 bool
      interval:      检查间隔（秒）
      stable_window: 连续成功次数阈值
      timeout:       总超时时间（秒）

    Returns:
      稳定窗口内连续通过返回 True，超时返回 False
    """
    start = time.monotonic()
    stable_count = 0
    while time.monotonic() - start < timeout:
        if check_fn():
            stable_count += 1
            if stable_count >= stable_window:
                return True
        else:
            stable_count = 0
        time.sleep(interval)
    return False
