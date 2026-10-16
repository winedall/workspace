from enum import Enum
from typing import Optional


class UpgradeState(str, Enum):
    """升级状态枚举（15 状态 + CANCELLED）。

    状态流转：
      CREATED → TRANSFERRING → VERIFYING → STAGED → SAFETY_CHECKING
      → STOPPING → SWITCHING → STARTING → HEALTH_CHECKING → SUCCEEDED

    异常分支：
      切换前失败 → FAILED
      切换后失败 → ROLLING_BACK → ROLLED_BACK / RECOVERY_REQUIRED
    """
    CREATED = "CREATED"
    TRANSFERRING = "TRANSFERRING"
    VERIFYING = "VERIFYING"
    STAGED = "STAGED"
    SAFETY_CHECKING = "SAFETY_CHECKING"
    STOPPING = "STOPPING"
    SWITCHING = "SWITCHING"
    STARTING = "STARTING"
    HEALTH_CHECKING = "HEALTH_CHECKING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    ROLLING_BACK = "ROLLING_BACK"
    ROLLED_BACK = "ROLLED_BACK"
    RECOVERY_REQUIRED = "RECOVERY_REQUIRED"
    CANCELLED = "CANCELLED"


class UpgradeStateMachine:
    """升级状态机，严格限制合法状态转换。

    用法：
      sm = UpgradeStateMachine(task_id, from_ver, to_ver)
      sm.transferring()
      sm.verifying()
      ...
      sm.succeeded()
    """

    _transitions = {
        UpgradeState.CREATED: [UpgradeState.TRANSFERRING, UpgradeState.CANCELLED],
        UpgradeState.TRANSFERRING: [UpgradeState.VERIFYING, UpgradeState.FAILED, UpgradeState.CANCELLED],
        UpgradeState.VERIFYING: [UpgradeState.STAGED, UpgradeState.FAILED, UpgradeState.CANCELLED],
        UpgradeState.STAGED: [UpgradeState.SAFETY_CHECKING, UpgradeState.FAILED, UpgradeState.CANCELLED],
        UpgradeState.SAFETY_CHECKING: [UpgradeState.STOPPING, UpgradeState.FAILED, UpgradeState.CANCELLED],
        UpgradeState.STOPPING: [UpgradeState.SWITCHING, UpgradeState.FAILED],
        UpgradeState.SWITCHING: [UpgradeState.STARTING, UpgradeState.FAILED],
        UpgradeState.STARTING: [UpgradeState.HEALTH_CHECKING, UpgradeState.ROLLING_BACK],
        UpgradeState.HEALTH_CHECKING: [UpgradeState.SUCCEEDED, UpgradeState.ROLLING_BACK],
        UpgradeState.ROLLING_BACK: [UpgradeState.ROLLED_BACK, UpgradeState.RECOVERY_REQUIRED],
    }
    _terminal = {UpgradeState.SUCCEEDED, UpgradeState.FAILED, UpgradeState.ROLLED_BACK,
                 UpgradeState.RECOVERY_REQUIRED, UpgradeState.CANCELLED}

    def __init__(self, task_id: str, from_version: str, to_version: str):
        self.task_id = task_id
        self.from_version = from_version
        self.to_version = to_version
        self.state = UpgradeState.CREATED
        self.error: Optional[str] = None

    def _transition(self, target: UpgradeState):
        """执行状态转换，非法转换抛出 RuntimeError。"""
        if self.state in self._terminal:
            raise RuntimeError(f"Cannot transition from terminal state {self.state}")
        allowed = self._transitions.get(self.state, [])
        if target not in allowed:
            raise RuntimeError(f"Invalid transition: {self.state} -> {target}")
        self.state = target

    def transferring(self): self._transition(UpgradeState.TRANSFERRING)
    def verifying(self): self._transition(UpgradeState.VERIFYING)
    def staged(self): self._transition(UpgradeState.STAGED)
    def safety_checking(self): self._transition(UpgradeState.SAFETY_CHECKING)
    def stopping(self): self._transition(UpgradeState.STOPPING)
    def switching(self): self._transition(UpgradeState.SWITCHING)
    def starting(self): self._transition(UpgradeState.STARTING)
    def health_checking(self): self._transition(UpgradeState.HEALTH_CHECKING)
    def succeeded(self): self._transition(UpgradeState.SUCCEEDED)

    def fail(self, error: str):
        """直接跳转到 FAILED（任何非终态，但切换后不允许）。"""
        if self.state in self._terminal:
            raise RuntimeError(f"Cannot fail from terminal state {self.state}")
        self.error = error
        self.state = UpgradeState.FAILED

    def rollback(self, error: str):
        """直接跳转到 ROLLING_BACK（切换后的失败场景）。"""
        self.error = error
        self.state = UpgradeState.ROLLING_BACK

    def rolled_back(self):
        """回滚成功。"""
        self.state = UpgradeState.ROLLED_BACK

    def recovery_required(self, error: str):
        """两个版本都无法启动，需要人工介入。"""
        self.error = error
        self.state = UpgradeState.RECOVERY_REQUIRED

    def cancel(self):
        """取消任务（仅切换前可取消，切换后不允许）。"""
        if self.state in (UpgradeState.STOPPING, UpgradeState.SWITCHING,
                          UpgradeState.STARTING, UpgradeState.HEALTH_CHECKING):
            raise RuntimeError(f"Cannot cancel after {self.state}")
        self.state = UpgradeState.CANCELLED
