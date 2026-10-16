"""PyTorch (TorchScript) policy backend。

该 backend 只负责 .pt 模型加载、输入检查、推理和 raw action 输出整理；
观测构造、动作解释、joint profile 选择都由 adapter/runtime 负责。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, List

from robot_rl_manager.model_registry import RegistryRlModelInfo
from robot_rl_manager.policy.backend import ModelInput


class TorchPolicyBackend:
    """使用 PyTorch TorchScript 执行 RL policy 的后端。

    加载 .pt 文件（TorchScript 序列化模型），通过 CPU 推理返回 raw action。
    支持模型输出格式：Tensor、dict（取首项）、tuple/list（取首项）。
    """

    def __init__(self) -> None:
        self._model = None          # torch.jit.ScriptModule，加载后的 TorchScript 模型
        self._input_names: List[str] = []  # 模型输入名列表（不含 self）
        self._model_info: RegistryRlModelInfo | None = None
        self._runtime_options: dict[str, Any] = {
            "intra_op_num_threads": 2,
            "inter_op_num_threads": 1,
        }

    def configure_runtime_options(self, options: dict[str, Any]) -> None:
        """配置 PyTorch 线程池参数；通常由 rl_manager 全局参数提供。"""
        self._runtime_options.update(options)

    def load(self, model_info: RegistryRlModelInfo) -> None:
        """加载 TorchScript 模型文件并设为 eval 模式。"""
        if not model_info.file:
            raise ValueError(f"model file is empty for {model_info.name}")
        model_path = Path(model_info.file)
        if not model_path.exists():
            raise FileNotFoundError(f"TorchScript model file not found: {model_path}")
        try:
            import torch
        except ImportError as exc:
            raise RuntimeError(
                "torch is required for TorchPolicyBackend; "
                "install with: pip install torch"
            ) from exc

        # 限制 PyTorch 线程池，避免默认用满全部 CPU 核导致高占用。
        # 必须在首次推理前调用，因此放在模型加载阶段。
        intra_op = int(self._runtime_options.get("intra_op_num_threads", 2))
        inter_op = int(self._runtime_options.get("inter_op_num_threads", 1))
        if intra_op > 0:
            torch.set_num_threads(intra_op)
        if inter_op > 0:
            # inter-op 线程池在进程内只能设置一次；重复设置会抛 RuntimeError。
            # 当前值已等于目标值则跳过；已被占用（如模型切换后重载）则保持首次设置的值。
            current_inter_op = torch.get_num_interop_threads()
            if current_inter_op != inter_op:
                try:
                    torch.set_num_interop_threads(inter_op)
                except RuntimeError:
                    pass

        self._model = torch.jit.load(str(model_path), map_location="cpu")
        self._model.eval()

        # 解析模型输入名，跳过第一个 self 参数
        self._input_names = [
            inp.debugName()
            for inp in self._model.graph.inputs()
            if not inp.debugName().startswith("self")
        ]
        if not self._input_names:
            raise RuntimeError(f"TorchScript model has no inputs: {model_path}")

        self._model_info = model_info

    def reset(self) -> None:
        """TorchScript feed-forward 模型默认没有隐状态。"""

    def infer(self, model_input: ModelInput) -> List[float]:
        """执行一次 TorchScript 推理，并校验输入/输出维度。

        输入 dict 的每个 value 转为 float32 tensor，自动添加 batch 维度；
        输出支持 Tensor / dict / tuple / list 四种常见格式。
        """
        if self._model is None or self._model_info is None:
            raise RuntimeError("Torch policy backend is not ready")

        import torch

        # adapter 按 key "obs" 输出，但 TorchScript 模型的输入名可能不同
        # （例如 "input.1"、"observations"）。自动按名匹配，左闭不需要的 adapter key。
        feed = {}
        for model_inp_name in self._input_names:
            # 先按模型输入名直接匹配
            if model_inp_name in model_input:
                values = model_input[model_inp_name]
            elif len(self._input_names) == 1 and len(model_input) == 1:
                # 单输入模型 + 单输入 adapter：取 adapter 唯一输入
                values = next(iter(model_input.values()))
            else:
                raise RuntimeError(
                    f"Torch model input '{model_inp_name}' not found in adapter inputs "
                    f"{sorted(model_input.keys())}"
                )
            tensor = torch.tensor(values, dtype=torch.float32)
            if tensor.ndim == 1:
                tensor = tensor.unsqueeze(0)
            feed[model_inp_name] = tensor

        # 校验 obs 维度（如果 model_info 中定义了 observation_dim）
        expected_obs_dim = int(self._model_info.observation_dim)
        obs_tensor = None
        for name in self._input_names:
            if "obs" in name.lower() or name in model_input:
                obs_tensor = feed[name]
                break
        if obs_tensor is None:
            obs_tensor = next(iter(feed.values()))
        if expected_obs_dim > 0 and obs_tensor.shape[-1] != expected_obs_dim:
            raise ValueError(
                f"Torch input dim mismatch for model {self._model_info.name}: "
                f"{obs_tensor.shape[-1]} != {expected_obs_dim}"
            )

        if len(feed) == 1:
            # 单输入模型：位置参数调用，避免 TorchScript 参数名不匹配（如 "input" vs "input.1"）
            input_tensor = next(iter(feed.values()))
            with torch.no_grad():
                outputs = self._model(input_tensor)
        else:
            with torch.no_grad():
                outputs = self._model(**feed)

        # 适配常见输出格式：Tensor / dict / tuple / list
        if isinstance(outputs, torch.Tensor):
            action = outputs.reshape(-1).tolist()
        elif isinstance(outputs, dict):
            first_key = next(iter(outputs))
            action = outputs[first_key].reshape(-1).tolist()
        elif isinstance(outputs, (tuple, list)):
            action = outputs[0].reshape(-1).tolist()
        else:
            raise RuntimeError(
                f"unexpected Torch model output type: {type(outputs)}"
            )

        return [float(v) for v in action]

    def is_ready(self) -> bool:
        """返回 TorchScript 模型是否已加载完成。"""
        return self._model is not None and self._model_info is not None

    def close(self) -> None:
        """释放 TorchScript 模型引用。"""
        self._model = None
        self._model_info = None
