"""外部遥控输入到 RobotIntent 的构造工具。

该模块只负责生成 RobotIntent，不生成 JointCommands，也不接触任何硬件接口。
"""

from __future__ import annotations

from dataclasses import dataclass

from robot_interfaces.msg import RobotIntent


@dataclass(frozen=True)
class IntentSpec:
    """输入解析层生成的中间意图描述。

    Joy、键盘或未来其他遥控输入都可以先归一化成 IntentSpec，再统一构造
    robot_interfaces/msg/RobotIntent。
    """
    # 意图类型，例如 status / motion / mode / model / config / reset。
    intent_type: str
    # 具体命令，例如 ready / walk / stop / greet / speed。
    command: str
    # mode/model/config 类意图使用，表示命令对象或参数。
    action: str = ""

    # motion + walk 使用的连续速度字段。
    linear_x_mps: float = 0.0
    linear_y_mps: float = 0.0
    angular_z_radps: float = 0.0
    
    # 单条意图可覆盖默认优先级；None 表示使用 IntentBuilder 默认值。
    priority: int | None = None


class IntentBuilder:
    """统一填充 RobotIntent 公共字段。"""

    def __init__(self, source_name: str = "teleop", priority: int = 0):
        # source_name 标识输入来源，teleop 默认优先级高于 API/云端输入。
        self._source_name = source_name
        self._priority = int(priority)

    def build(self, spec: IntentSpec) -> RobotIntent:
        """把 IntentSpec 转换为 RobotIntent 消息。"""
        msg = RobotIntent()
        # 所有未使用字段显式保持空字符串或 0，避免把文档中的 "/" 当成有效值。
        msg.source = self._source_name
        msg.intent_type = spec.intent_type
        msg.command = spec.command
        msg.linear_x_mps = float(spec.linear_x_mps)
        msg.linear_y_mps = float(spec.linear_y_mps)
        msg.angular_z_radps = float(spec.angular_z_radps)
        msg.action = spec.action
        # 允许单个 spec 覆盖优先级；否则使用来源默认优先级。
        msg.priority = int(self._priority if spec.priority is None else spec.priority)
        return msg

    def build_status(self, command: str) -> RobotIntent:
        """构造 ready/walk/damp/reinit 等状态切换意图。"""
        return self.build(IntentSpec(intent_type="status", command=command))

    def build_motion(
        self,
        command: str,
        linear_x_mps: float = 0.0,
        linear_y_mps: float = 0.0,
        angular_z_radps: float = 0.0,
    ) -> RobotIntent:
        """构造连续行走或普通 stop 的 motion 意图。"""
        return self.build(
            IntentSpec(
                intent_type="motion",
                command=command,
                linear_x_mps=linear_x_mps,
                linear_y_mps=linear_y_mps,
                angular_z_radps=angular_z_radps,
            )
        )

    def build_model(self, action: str) -> RobotIntent:
        """构造动作模型选择或执行意图。"""
        return self.build(
            IntentSpec(intent_type="model", command="switch_model", action=action)
        )

    def build_config(self, command: str, action: str) -> RobotIntent:
        """构造运行配置修改意图。"""
        return self.build(
            IntentSpec(intent_type="config", command=command, action=action)
        )
