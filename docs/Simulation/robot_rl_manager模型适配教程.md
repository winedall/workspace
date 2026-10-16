# robot_rl_manager 模型适配教程

本文面向负责训练和导出运动模型的人员，说明一个新 RL/运动模型接入
`robot_rl_manager` 推理框架时需要提供哪些文件，以及这些文件如何被框架调用。

以下说明以 Walk 模型为例。当前代码中的实际适配器目录是
`robot_rl_manager/policy/adapters/`，因此 Walk 适配器路径为：

```text
src/robot_rl_manager/robot_rl_manager/policy/adapters/walk.py
```

## 1. 新模型交付文件

以 Walk 模型为例，一个可运行模型至少需要以下三类文件：

```text
src/robot_rl_manager/robot_rl_manager/policy/adapters/walk.py
src/robot_rl_manager/config/walk.yaml
src/robot_rl_manager/model/t4_amp_02.onnx
```

此外，还需要在工作区根目录的 `config/rl_models.yaml` 中注册模型索引：

```yaml
models:
  walk:
    alias: "行走"
    category: "走跑类"
    duration_sec: 0.0
    has_duration: false
    config_path: "config/walk.yaml"
    adapter_path: "policy/adapters/walk.py"
    artifacts:
      x86_64:
        model_path: "model/t4_amp_02.onnx"
        backend: onnx
```

各文件职责如下：

- `walk.py`：策略适配器。负责把机器人上下文转换成 ONNX 输入，并把模型输出转换成机器人全身关节目标。
- `walk.yaml`：模型元数据和关节 profile。负责声明模型维度、控制频率、运行姿态、PD 参数。
- `t4_amp_02.onnx`：导出的 ONNX 模型文件。ONNX 输入名和输出维度必须与适配器和 YAML 一致。
- `rl_models.yaml`：模型索引。负责把模型名、配置文件、平台化模型 artifact、适配器文件关联起来。

`setup.py` 会把 `src/robot_rl_manager/config/*.yaml` 安装到
`share/robot_rl_manager/config/`，把 `src/robot_rl_manager/model/*` 安装到
`share/robot_rl_manager/model/`。因此新模型的 YAML 和 ONNX 文件应放在上述源码目录，
不要只放到 `install/` 或 `build/` 目录。

## 2. 框架调用链

运行时的核心分工是：

```text
ModelRegistry
  -> 读取 config/rl_models.yaml
  -> 读取单模型 config/<model>.yaml
  -> 按 runtime_platform 解析 adapter_path、artifact.model_path、artifact.backend

factory
  -> 根据 artifact.backend 创建 backend
  -> 根据 adapter_path 创建 adapter

PolicyRuntime
  -> 组装 PolicyContext
  -> 调用 adapter.build_model_input(context)
  -> 调用 backend.infer(model_input)
  -> 调用 adapter.decode_action(raw_action, context)
  -> 使用 joint profile 生成 JointCommands
```

其中：

- backend 只负责模型加载和推理，例如 `OnnxPolicyBackend`（ONNX 模型）或 `TorchPolicyBackend`（PyTorch TorchScript 模型）。
- adapter 负责模型语义，包括观测拼接、历史帧、相位、动作缩放、关节顺序映射。
- YAML 负责模型元数据和关节控制参数。
- `artifacts` 按 `runtime_platform` 严格匹配；当前平台未声明 artifact 的模型不会注册，`default_model_name` 和 `home_model` 必须在当前平台可加载。
- `alias`、`category`、`duration_sec`、`has_duration` 只作为查询服务返回的展示元信息，不参与模型控制逻辑。

训练人员交付新模型时，最重要的是保证三件事一致：

1. 模型输入名和 `build_model_input()` 返回的字典 key 一致。
2. 模型输出维度和 `action_dim`、`decode_action()` 期望维度一致。
3. `decode_action()` 返回的关节目标数量和机器人关节配置数量一致。

## 3. `walk.py` 实现方式

Walk 适配器类定义如下：

```python
class WalkPolicyAdapter(BasePolicyAdapter):
    ...
```

类名必须符合文件名推导规则：

- 文件名：`walk.py`
- 类名：`WalkPolicyAdapter`
- 注册路径：`policy/adapters/walk.py`
- 框架推导出的类路径：`robot_rl_manager.policy.adapters.walk.WalkPolicyAdapter`

如果文件名是 `new_walk.py`，类名应为 `NewWalkPolicyAdapter`。

### 3.1 常量区

Walk 适配器顶部定义了两组重要常量：

```python
AMP_REAL_JOINT_NAMES = [...]
AMP_POLICY_IDX = [...]
```

`AMP_REAL_JOINT_NAMES` 是旧 AMP policy 使用的 29 个真实关节顺序。适配器会按这个顺序
从机器人反馈中提取 `qpos` 和 `qvel`，也会按这个顺序把模型 action 写回对应关节。

`AMP_POLICY_IDX` 是模型实际控制的关节索引。Walk 模型输出 27 维 raw action，不直接覆盖
全部 29 个 AMP 关节，而是只覆盖 `AMP_POLICY_IDX` 指定的关节。

训练新模型时必须明确：

- 模型训练时的关节顺序。
- 模型输出 action 对应哪些关节。
- 是否存在不由模型输出直接控制的关节。
- 是否需要从完整机器人关节顺序映射到训练时的 policy 顺序。

### 3.2 内部状态

Walk 适配器维护以下状态：

```python
_frame_stack = 5
_action_scale = 0.25
_policy_interval_sec = 0.02
_period_sec = 0.6
```

- `_frame_stack`：历史观测堆叠帧数。Walk 使用 5 帧历史。
- `_action_scale`：raw action 转关节位置增量时的缩放系数。
- `_policy_interval_sec`：策略周期。Walk 为 0.02 秒，对应 50 Hz。
- `_period_sec`：步态相位周期。Walk 为 0.6 秒。

`reset()` 会初始化：

- `_phase`：步态相位。
- `_last_action`：上一帧 action，用作当前观测的一部分。
- `_smoothed_command`：平滑后的速度命令。
- `_hist_obs`：分组历史观测队列。

只要模型依赖历史帧、上一帧 action、相位或循环播放状态，就应在 `reset()` 中清理这些状态。
框架在加载模型或重置 runtime 时会调用 `adapter.reset()`。

### 3.3 输入依赖声明

Walk 的 `required_inputs()` 返回：

```python
return {"joint_states", "imu"}
```

这表示推理前必须有：

- `joint_states`：机器人关节状态。
- `policy_imu`：框架转换后的 IMU 语义输入。

`PolicyRuntime` 会在调用 `build_model_input()` 前检查这些输入。如果缺失，会直接报错，
不会进入模型推理。

如果新模型只依赖关节状态，可以返回 `{"joint_states"}`。如果还依赖深度图、接触状态等，
需要确保 `PolicyContext.has_input()` 能识别对应字段，并在节点层提供该输入。

### 3.4 构造模型输入

Walk 的 `build_model_input()` 返回：

```python
return {"obs": obs, "hist": hist}
```

这要求 ONNX 模型的输入名必须包含 `obs` 和 `hist`。`OnnxPolicyBackend` 会读取 ONNX
真实输入名，并校验 adapter 返回的 key 是否完全匹配。少传或多传输入都会报错。

Walk 当前观测 `obs` 的拼接顺序是：

```text
ang + grav + cmd + qpos + qvel + action + phase
```

各分组含义如下：

- `ang`：机身角速度，来自 `imu.base_ang_vel_radps`，3 维。
- `grav`：投影重力，来自 `imu.projected_gravity`，3 维。
- `cmd`：速度命令 `[linear.x, linear.y, angular.z]`，3 维，经过每周期最大 0.05 的平滑。
- `qpos`：当前关节位置相对 run profile 参考姿态的偏差，27 维。
- `qvel`：当前关节速度，27 维。
- `action`：上一帧模型 raw action，27 维。
- `phase`：步态相位 `[sin, cos]`，2 维。

因此 `obs` 总维度为：

```text
3 + 3 + 3 + 27 + 27 + 27 + 2 = 92
```

历史输入 `hist` 使用同样分组，但每组分别堆叠 5 帧：

```text
(3 + 3 + 3 + 27 + 27 + 27 + 2) * 5 = 460
```

适配器会通过 `_validate_model_dims()` 校验：

```text
obs  == 92
hist == 460
```

新模型如果改变观测内容，应同步修改：

- `build_model_input()` 的拼接逻辑。
- ONNX 输入名和输入维度。
- YAML 中的 `observation_dim`。
- 适配器内的维度校验。

### 3.5 速度命令平滑和相位

Walk 使用 `_update_smoothed_command()` 限制速度命令每个推理周期的变化量：

```text
每个速度分量每周期最多变化 0.05
```

`_next_phase()` 每周期推进：

```text
phase += policy_interval_sec / period_sec
```

当速度命令模长小于等于 `0.1` 时，Walk 输出零相位 `[0.0, 0.0]`；否则输出
`[sin(2*pi*phase), cos(2*pi*phase)]`。

如果新模型训练时使用了不同命令滤波、相位定义或停止逻辑，应在适配器中保持与训练环境一致。

### 3.6 解码模型输出

Walk 的 `decode_action()` 接收 ONNX raw action，并返回机器人全身关节位置目标：

```python
def decode_action(self, raw_action, context) -> List[float]:
    ...
```

Walk 当前逻辑是：

1. 校验 raw action 维度必须等于 `len(AMP_POLICY_IDX)`，即 27。
2. 保存 `_last_action`，供下一帧观测使用。
3. 从 `walk.yaml` 的 `joint_profiles.run` 读取全身参考姿态。
4. 按 `AMP_POLICY_IDX` 把 action 写入 AMP 关节目标：

   ```text
   target = run_profile_position + action_scale * raw_action
   ```
5. 再按机器人 `joint_configs` 的顺序返回全身关节目标。

注意：`decode_action()` 返回的是位置目标列表，不是 PD 参数。PD 参数来自 YAML 的
`joint_profiles.run.joints[*].kp/kd`，由 `JointCommandBuilder` 生成最终 `JointCommands`。

### 3.7 joint profile 选择

Walk 的 `select_joint_profile()` 返回 `"run"`：

```python
return "run"
```

当前 `PolicyRuntime.infer_commands()` 默认也使用 `profile_name="run"` 构造命令。
如果后续模型需要按状态选择不同 profile，需要同时确认节点层是否会使用
`adapter.select_joint_profile()` 的返回值。

### 3.8 运动结束逻辑

Walk 的 `is_motion_end()` 始终返回 `False`：

```python
return False
```

因为 Walk 是连续行走模型，没有固定参考动作播放完毕的概念。

一次性动作模型或循环动作模型应根据自身帧索引、参考轨迹长度或 `finish_loop()` 状态返回
是否结束。例如 hello、clap 这类模型会在收到 `rl_manager/finish_loop` 后停止循环，
播完剩余帧再结束。

## 4. `walk.yaml` 参数说明

Walk 配置文件结构如下：

```yaml
model:
  name: walk
  control_frequency_hz: 50
  observation_dim: 92
  action_dim: 27

joint_profiles:
  run:
    joints:
      - {joint_name: head_yaw, position: 0.0, kp: 20.0, kd: 1.0}
```

### 4.1 `model`

`model.name`

模型名称。通常应与 `config/rl_models.yaml` 中的 key 一致，例如 `walk`。该名称用于模型切换、
日志和状态展示。

模型类型不再写在 `walk.yaml` 中。静态注册模型的后端类型由 `rl_models.yaml`
中当前平台选中的 artifact 决定，例如 `backend: onnx`、`backend: pt`、
`backend: pipeline` 或 `backend: rknn`。
当类型为 `pt` 时，使用
`robot_rl_manager.policy.torch.TorchPolicyBackend`。

`model.control_frequency_hz`

策略控制频率，单位 Hz。Walk 为 50 Hz，对应 0.02 秒一个推理周期。该值应与训练时的策略
时间步、适配器中的 `_policy_interval_sec` 和节点实际调度频率保持一致。

`model.observation_dim`

`obs` 输入维度，用于运行时校验。Walk 为 92。注意当前通用校验只检查 `obs` 维度；
如果模型还有 `hist` 等额外输入，需要在适配器内部额外校验。

`model.action_dim`

backend 原始输出 raw action 维度。Walk 为 27。该值表示模型输出维度，不要求等于机器人
全身关节数量。raw action 到全身关节目标的转换由 `decode_action()` 负责。

### 4.2 `joint_profiles.run`

`joint_profiles.run` 是模型运行时使用的关节 profile。当前 `ModelRegistry` 强制每个模型都必须
提供 `run` profile。

每个关节项包含：

```yaml
- {joint_name: left_knee, position: 0.3, kp: 100.0, kd: 4.0}
```

字段含义：

- `joint_name`：关节名，必须与机器人关节配置中的名称一致。
- `position`：该 profile 下的参考位置，单位通常为 rad。
- `kp`：位置比例增益，必须大于等于 0。
- `kd`：速度微分增益，必须大于等于 0。

框架会校验：

- 每个关节必须有 `joint_name`、`position`、`kp`、`kd`。
- `kp` 和 `kd` 不能为负。
- profile 中不能有重复关节。
- profile 不能包含未知关节。
- profile 必须覆盖当前机器人关节配置中的所有关节。

Walk 还额外要求 `run` profile 覆盖 `AMP_REAL_JOINT_NAMES` 中的所有 AMP 关节，否则
`_run_profile_positions()` 会报错。

对 Walk 来说，`position` 有两层作用：

1. 作为模型解码时的参考姿态：

   ```text
   target = run_profile_position + action_scale * raw_action
   ```
2. 作为构造最终 `JointCommands` 时的默认 profile 信息和 PD 参数来源。

因此训练新模型时，`run.position` 应与训练环境中的默认站姿、参考姿态或 action 零点定义一致。

## 5. `adapter.py` 接口规范

`adapter.py` 定义了 `PolicyAdapter` 协议和 `BasePolicyAdapter` 默认实现。

新模型适配器通常继承：

```python
from robot_rl_manager.policy.adapter import BasePolicyAdapter


class XxxPolicyAdapter(BasePolicyAdapter):
    ...
```

### 5.1 `reset()`

```python
def reset(self) -> None:
    ...
```

调用时机：

- runtime 加载模型后。
- runtime 被重置时。

用途：

- 清空历史观测。
- 清空上一帧 action。
- 重置相位、帧索引、循环状态。

要求：

- 不应依赖当前帧 `PolicyContext`。
- 不应执行耗时模型推理或文件加载。

### 5.2 `required_inputs()`

```python
def required_inputs(self) -> set[str]:
    ...
```

用途是声明 `build_model_input()` 前必须具备哪些上下文输入。常见值：

- `"joint_states"`：关节状态。
- `"imu"`：策略 IMU 语义输入。
- `"cmd_vel"`：速度命令。
- `"model_info"`：模型配置。
- `"joint_configs"`：机器人关节配置。
- `"depth_image"`：深度图。

`PolicyRuntime` 会调用 `context.has_input()` 检查这些输入。缺失时会抛出：

```text
policy context missing required input: ...
```

建议只声明真正必须的输入，避免模型在不需要某传感器时被无谓阻塞。

### 5.3 `build_model_input(context)`

```python
def build_model_input(self, context: PolicyContext) -> ModelInput:
    ...
```

用途：

- 从 `PolicyContext` 中读取命令、关节状态、IMU、模型信息等。
- 构造 backend 所需输入字典。

返回值类型：

```python
dict[str, Iterable[float]]
```

ONNX 模型常见返回形式：

```python
return {"obs": obs}
return {"obs": obs, "hist": hist}
```

要求：

- key 必须与 ONNX 输入名一致。
- value 必须能转换为 float。
- `obs` 长度必须与 YAML 的 `observation_dim` 一致。
- 额外输入如 `hist` 的维度应由适配器自行校验。
- 缺少必要字段时应使用 `context.require_xxx()` 抛出明确错误。

### 5.4 `decode_action(raw_action, context)`

```python
def decode_action(self, raw_action: Iterable[float], context: PolicyContext) -> List[float]:
    ...
```

用途：

- 把 backend 输出的 raw action 转换为机器人全身关节位置目标。

要求：

- 应校验 raw action 维度。
- 返回列表长度必须等于机器人关节配置数量。
- 返回顺序必须与 `context.require_joint_configs()` 中的关节顺序一致。
- 如需使用 action scale、默认姿态、关节映射，应在这里处理。
- 不应返回 `kp/kd`，PD 参数来自 joint profile。

如果输出维度不等于关节数量，`PolicyRuntime` 会报：

```text
decoded action dim mismatch
```

### 5.5 `select_joint_profile(context)`

```python
def select_joint_profile(self, context: PolicyContext) -> str:
    ...
```

用途是选择当前 `JointCommands` 使用哪个 joint profile。基础实现返回 `"run"`。

当前 Walk 模型固定返回 `"run"`。如果模型有多个运行 profile，需要确保对应 profile 已在 YAML
中定义，并确认节点层会使用该返回值。

### 5.6 `is_motion_end()`

```python
def is_motion_end(self) -> bool:
    ...
```

用途是告诉节点当前参考运动是否结束。

- 连续模型：通常始终返回 `False`，例如 Walk。
- 一次性动作：当参考动作播完后返回 `True`。
- 带循环动作：循环中返回 `False`；收到结束循环信号并播完剩余动作后返回 `True`。

### 5.7 `finish_loop()`

```python
def finish_loop(self) -> None:
    ...
```

节点收到 `rl_manager/finish_loop` 后会调用当前 adapter 的 `finish_loop()`。

用途：

- 对 hello、clap 这类循环动作，通知 adapter 停止循环，继续播放剩余帧。
- 对 Walk 这类连续模型，通常无需覆写，使用默认空实现即可。

## 6. 新模型接入检查清单

提交新模型前建议逐项确认：

- 模型已导出为 ONNX，且文件放在 `src/robot_rl_manager/model/`。
- 单模型 YAML 放在 `src/robot_rl_manager/config/`。
- 适配器放在 `src/robot_rl_manager/robot_rl_manager/policy/adapters/`。
- `config/rl_models.yaml` 已添加 `alias`、`category`、`duration_sec`、`has_duration`、`config_path`、`adapter_path`、`artifacts`。
- 适配器文件名和类名符合推导规则，例如 `walk.py` -> `WalkPolicyAdapter`。
- ONNX 输入名与 `build_model_input()` 返回 key 完全一致。
- `obs` 维度与 `model.observation_dim` 一致。
- raw action 维度与 `model.action_dim` 和 `decode_action()` 期望一致。
- `decode_action()` 返回全身关节位置目标，且顺序与机器人关节配置一致。
- `joint_profiles.run` 覆盖所有机器人关节，没有未知关节或重复关节。
- `run.position` 与训练时 action 零点或参考姿态一致。
- `kp/kd` 为非负数，并与真实机器人或仿真控制器可接受的增益范围一致。
- `required_inputs()` 只声明真实需要的输入。
- 需要历史帧、相位、上一帧 action 的模型已在 `reset()` 中完整初始化状态。
- 连续模型和一次性动作模型已正确实现 `is_motion_end()`。
- 循环动作模型已正确实现 `finish_loop()`。

## 7. Walk 模型文件对应关系

Walk 当前三类文件的对应关系是：

```text
config/rl_models.yaml
  walk.config_path  -> config/walk.yaml
  walk.adapter_path -> policy/adapters/walk.py
  walk.artifacts.x86_64.model_path -> model/t4_amp_02.onnx
  walk.artifacts.x86_64.backend    -> onnx

config/walk.yaml
  model.name                -> walk
  model.control_frequency_hz -> 50
  model.observation_dim      -> 92
  model.action_dim           -> 27
  joint_profiles.run         -> 全身关节参考位置和 PD 参数

policy/adapters/walk.py
  build_model_input() -> {"obs": 92 维, "hist": 460 维}
  decode_action()     -> 27 维 raw action 映射为全身关节位置目标
```

只要新模型遵守这套契约，`robot_rl_manager` 的节点、backend 和命令构造逻辑通常不需要修改。
