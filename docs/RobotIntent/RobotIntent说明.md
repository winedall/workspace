# RobotIntent 使用说明

`RobotIntent` 是外部控制源的统一高层意图。遥控器、HTTP、MQTT、SDK 和云端服务应先发布 RobotIntent，由 `robot_router` 准入后再交给状态机或控制模块处理，不应直接向硬件发送关节命令。

`docs/RobotIntent说明.xlsx` 是 RobotIntent 合法组合的唯一权威定义。表格中的 `/` 表示字段不使用；ROS 消息中对应字符串应为空、数值应为 `0`。

## 1. 消息字段

```yaml
header: {}
source: "teleop"
intent_type: ""
command: ""
action: ""
linear_x_mps: 0.0
linear_y_mps: 0.0
angular_z_radps: 0.0
priority: 0
```

| 字段 | 含义 |
|---|---|
| `header.stamp` | 意图产生时间，用于超时判断 |
| `source` | 来源，例如 `teleop/http/mqtt/sdk/cloud/test` |
| `intent_type` | 意图类别 |
| `command` | 类别内的操作 |
| `action` | mode、model 或 config 的操作对象/参数 |
| 三个速度字段 | `motion/walk` 的连续速度，单位分别为 m/s、m/s、rad/s |
| `priority` | 来源或单条意图的准入优先级 |

`target_mode`、`model_name` 和 `action_name` 已删除，统一由 `action` 表达。该变更不影响 `SetRobotMode`、`LoadRlModel`、`SwitchRlModel` 等专用服务自身的字段。

## 2. 合法意图组合

| intent_type | command | action | 说明 |
|---|---|---|---|
| `status` | `ready` | 空 | 进入准备流程 |
| `status` | `walk` | 空 | 从 READY 进入行走控制状态 |
| `status` | `damp` | 空 | 进入阻尼状态 |
| `status` | `reinit` | 空 | 从 DAMP 回到 IDLE |
| `config` | `mode` | `rl` / `mix` / `spec` | 切换控制模式（spec=特技模式，仅影响遥控器按键映射） |
| `model` | `switch_model` | `t4_walk` / `t4_heart` / `t4_hello` / `t4_taichi` / `t4_taichi_w` / `t4_mj_dance` / `t4_xqxa123_dance` / `t4_squat_down` / `t4_squat_up` 等注册名 | 切换动作模型（完整注册名见 `config/humanoid/rl_models.yaml`；四足见 `config/dog/rl_models.yaml`） |
| `model` | `finish_loop` | 空 | 优雅退出当前循环动作（如 t4_hello/t4_clap），播放完当前周期后自动回到 t4_walk |
| `model` | `motion_stop` | `stop` / `resume` | 停止或恢复当前模型运动，停止时强制切回 t4_walk 零速站立 |
| `motion` | `walk` | 空 | 连续行走速度 |
| `config` | `getup` | `getup` / `stand` / `lie` / `face` | 起身配置菜单或值 |
| `config` | `nav` | `nav` / `auto` / `manual` | 导航配置菜单或值 |
| `config` | `speed` | `speed` / `std` / `high` | 速度配置菜单或值 |
| `reset` | `reset_fault` | 空 | 故障复位 |

一级配置菜单提示使用与 command 同名的 action，例如 `config/speed/speed`；选择具体配置时 action 改为具体值，例如 `config/speed/high`。

## 3. 典型示例

### 3.1 状态切换

```yaml
intent_type: "status"
command: "ready"
action: ""
```

`status/walk` 表示进入行走控制状态；它不携带速度，也不指定模型。

### 3.2 模式切换

```yaml
intent_type: "config"
command: "mode"
action: "rl"
```

mode 只允许 `rl`、`mix` 和 `spec`，不再支持 `idle` 或 `debug`。

### 3.3 模型切换

```yaml
intent_type: "model"
command: "switch_model"
action: "t4_mj_dance"
```

`action` 是模型注册名，不是模型文件路径。当前部署是否能执行某个 action，取决于 `config/humanoid/rl_models.yaml` 是否注册该模型；未注册模型必须拒绝并报告错误。

### 3.4 优雅退出循环动作

```yaml
intent_type: "model"
command: "finish_loop"
action: ""
```

`model/finish_loop` 用于通知当前循环动作模型（如 t4_hello、t4_clap）停止循环，当前参考运动播完最后一帧后自然结束，自动回退到 t4_walk 模型。与直接切换模型不同，finish_loop 不中断正在播放的运动帧，可实现平滑退出。

### 3.5 连续运动

```yaml
intent_type: "motion"
command: "walk"
action: ""
linear_x_mps: 0.4
linear_y_mps: 0.0
angular_z_radps: -0.2
```

motion 只保留 `walk`。普通停止使用同一意图的零速度值，不再定义 `motion/stop` 或 `motion/stand`。

### 3.6 配置

```yaml
intent_type: "config"
command: "speed"
action: "high"
```

config 只修改运行配置，不直接触发状态跳转或动作执行。

## 4. 准入与安全约束

- `robot_router` 应按本表拒绝所有非法的 intent_type/command/action 组合。
- `motion_stop` 具有最高安全优先级。
- model 切换在 DO_READY 阶段应被拒绝。
- RobotIntent 是高层意图，不得绕过 router 直接形成硬件输出。
