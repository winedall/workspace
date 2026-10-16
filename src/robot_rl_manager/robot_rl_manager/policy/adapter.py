"""Policy adapter 接口与基础实现。

backend 只关心“如何运行模型”，adapter 负责把机器人上下文转换成模型输入，
并把模型输出解释成 robot_joints.yaml 顺序下的关节目标。
"""

from __future__ import annotations

from typing import Iterable, List, Protocol

from robot_rl_manager.policy.backend import ModelInput
from robot_rl_manager.policy.context import PolicyContext


class PolicyAdapter(Protocol):
    """不同策略模型的语义适配接口。"""

    def reset(self) -> None:
        """重置 adapter 内部状态，例如历史 action 或步态相位。"""

    def required_inputs(self) -> set[str]:
        """声明该 adapter 运行所需的上下文输入。"""

    def is_motion_end(self) -> bool:
        """返回模型参考运动是否已播放完毕。

        连续模型（如 walk）始终返回 False；一次性模型（如 heart）
        在参考运动播放完毕后返回 True。
        """

    def build_model_input(self, context: PolicyContext) -> ModelInput:
        """构造模型输入字典。"""

    def decode_action(self, raw_action: Iterable[float], context: PolicyContext) -> List[float]:
        """把模型输出解释为按关节顺序排列的位置目标。"""

    def select_joint_profile(self, context: PolicyContext) -> str:
        """选择当前 JointCommands 应使用的 joint profile。"""

    def finish_loop(self) -> None:
        """通知 adapter 结束 loop 循环。

        有 loop 的模型（如 hello）收到后应停止循环，让运动播完，
        随后 is_motion_end() 返回 True。无 loop 的模型无需覆写。
        """


class BasePolicyAdapter:
    """大多数初版 adapter 可复用的保守默认行为。"""

    def reset(self) -> None:
        """基础 adapter 当前没有内部状态。"""

    def required_inputs(self) -> set[str]:
        """基础 adapter 只要求有关节状态。"""
        return {"joint_states"}

    def is_motion_end(self) -> bool:
        """基础 adapter 默认无参考运动，始终返回 False。"""
        return False

    def build_model_input(self, context: PolicyContext) -> ModelInput:
        """构造固定长度模型输入。

        初版只把意图速度和关节反馈填入前部，其余位置补 0。真实模型接入后，
        walk/stand adapter 可以扩展 IMU、接触、历史 action、相位等特征，但节点
        和 backend 不需要改变。
        """
        model_info = context.require_model_info()
        joint_states = context.require_joint_states()
        cmd_vel = context.cmd_vel
        observation_dim = int(model_info.observation_dim)
        if observation_dim < 0:
            raise ValueError(f"observation_dim must be >= 0 for model {model_info.name}")

        observation: List[float] = []
        observation.extend(
            [
                float(cmd_vel.linear.x),
                float(cmd_vel.linear.y),
                float(cmd_vel.angular.z),
            ]
        )
        for state in joint_states.states:
            observation.extend([float(state.position), float(state.velocity)])

        if observation_dim == 0:
            return {"obs": observation}
        if len(observation) > observation_dim:
            return {"obs": observation[:observation_dim]}
        observation.extend([0.0] * (observation_dim - len(observation)))
        return {"obs": observation}

    def decode_action(self, raw_action: Iterable[float], context: PolicyContext) -> List[float]:
        """默认把 raw action 直接当作关节位置目标。"""
        del context
        return [float(value) for value in raw_action]

    def select_joint_profile(self, context: PolicyContext) -> str:
        """默认使用 run profile。"""
        del context
        return "run"

    def finish_loop(self) -> None:
        """默认无 loop 行为。"""
        pass
