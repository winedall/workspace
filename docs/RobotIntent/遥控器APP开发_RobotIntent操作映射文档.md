# 遥控器 APP 与 RobotIntent 操作映射文档

> 目标：为 APP 开发同事提供参照 F710 遥控器操作流程的 APP 控制实现指南。
> 核心原则：APP 与 F710 遥控器共用同一套 **RobotIntent 消息协议**，发布到同一话题。
> 所有 APP 操作最终只需发布 `RobotIntent` 到 `input/intent`，无需关心下游路由与执行细节。

---

## 一、控制架构总览

```
APP (或 F710 遥控器)
   │  publish RobotIntent
   ▼
input/intent                     ← APP 只关心这个入口话题
   │
   ▼
robot_router (命令路由/准入/优先级仲裁)
   │
   ├── router/active_intent      → robot_state (状态机切换)
   │                                    → robot_rl_manager (速度/模型/配置)
   │
   ▼
robot_rl_manager → robot_control → robot_driver → 硬件
```

### RobotIntent 消息定义 (`robot_interfaces/msg/RobotIntent.msg`)

```
std_msgs/Header header       # 时间戳，用于超时判断
string source                # 来源标识: "app"
string intent_type           # status | motion | model | config | reset
string command               # 具体命令 verb
string action                # 命令对象/参数
float32 linear_x_mps         # 前进速度 m/s
float32 linear_y_mps         # 横移速度 m/s
float32 angular_z_radps      # 偏航角速度 rad/s
uint8 priority               # 优先级（数值越小越高）: teleop=0(固定最高) > app(1~49) > api(50~99) > unknown(255)
```

> APP 发布时推荐：`source = "app"`, `priority = 10`（位于 app 优先级区间 1~49 内）

---

## 二、最简启动流程（APP 的操作步骤）

完整的启动到运动流程，按顺序执行：

### 步骤 0：等待系统就绪

系统启动后自动经历 `BOOTING → INIT → IDLE`，无需 APP 操作。订阅 `state` 话题确认 `state=2 (STATE_IDLE)`。

### 步骤 1：准备（Stand Up）

机器人从 IDLE 状态起立，进入 READY 站立状态。

- **F710 按键**：`LB + RB + A`
- **APP 发布**：
  ```yaml
  topic: /robot/input/intent
  message:
    source: "app"
    intent_type: "status"
    command: "ready"
    action: ""
    priority: 10

  ```

- **状态变化**：`IDLE(2) → DO_READY(3) → READY(4)`
  - 期间 robot_rl_manager 执行准备插值（约数秒），进度通过 `control/preparation_progress` 监听
  - 准备完成后自动到达 `READY(4)`，订阅 `state` 可获知

### 步骤 2：启用行走控制（Authorize Walk）

在 READY 状态下通知机器人准备接收运动指令。

- **F710 按键**：`LB + RB + B`
- **APP 发布**：
  ```yaml
  topic: /robot/input/intent
  message:
    source: "app"
    intent_type: "status"
    command: "walk"
    action: ""
    priority: 10

  ```

- **状态变化**：`READY(4) → RL_STANDBY(5) → RL_ACTIVE(6)`
  - 当 RL 管理器就绪（`model_loaded`、`model_ready`、无推理错误）后自动进入 `RL_ACTIVE`

### 步骤 3：运动控制（Move）

持续发布速度指令控制机器人行走。

- **F710 操作**：左摇杆前后/左右、右摇杆左右
- **APP 发布**（连续周期发布，建议 20Hz）：
  ```yaml
  topic: /robot/input/intent
  message:
    source: "app"
    intent_type: "motion"
    command: "walk"
    action: ""
    linear_x_mps: 0.5        # 前进速度，范围 [-max, +max]
    linear_y_mps: 0.0        # 横移速度，范围 [-max, +max]
    angular_z_radps: 0.0     # 偏航速度，范围 [-max, +max]
    priority: 10

  ```

- **停止**：发布全零速度的 `motion/walk` 即可
- **速度限幅参考**：
  - 标准速度：`linear_x ±1.0 m/s`, `linear_y ±0.5 m/s`, `angular_z ±1.5 rad/s`
  - 高速模式：`linear_x ±3.0 m/s`, `linear_y ±1.0 m/s`, `angular_z ±1.5 rad/s`
- **超时保护**：router 的意图时效检查基于消息 `header.stamp`，过期意图会被拒绝

### 步骤 4：坐下/阻尼（进入 DAMP）

让机器人进入阻尼（柔性）状态，可以安全弯折关节。

- **F710 按键**：`LB + RB + Y`
- **APP 发布**：
  ```yaml
  topic: /robot/input/intent
  message:
    source: "app"
    intent_type: "status"
    command: "damp"
    action: ""
    priority: 10

  ```

- **状态变化**：任意状态 → `DAMP(12)`

### 步骤 5：重新初始化（从 DAMP 回到 IDLE）

从 DAMP 状态回到初始空闲状态（仅在 DAMP 下有效）。

- **F710 按键**：`LB + RB + X`
- **APP 发布**：
  ```yaml
  topic: /robot/input/intent
  message:
    source: "app"
    intent_type: "status"
    command: "reinit"
    action: ""
    priority: 10

  ```

- **前提条件**：当前必须在 `DAMP` 状态
- **状态变化**：`DAMP(12) → IDLE(2)`

---

## 三、动作模型切换（交互/表演类）

机器人处于 `RL_ACTIVE` 时可切换动作模型，执行完一次动作后自动回到行走模型。

### 单动作交互（LB + DPad 系列）

| F710 按键 | 动作 | RobotIntent |
|-----------|------|-------------|
| LB + 方向键左 | 打招呼 | `model / switch_model / t4_hello` |
| LB + 方向键上 | 鼓掌 | `model / switch_model / t4_clap` |
| LB + 方向键右 | 蹲起 | `model / switch_model / t4_squat_up` |
| LB + 方向键下 | 下蹲 | `model / switch_model / t4_squat_down` |
| RB + 方向键左 | 比心 | `model / switch_model / t4_heart` |
| RB + 方向键上 | 敬礼 | `model / switch_model / t4_salute` |
| RB + 方向键右 | 握手 | `model / switch_model / t4_handshake` |
| RB + 方向键下 | 击掌 | `model / switch_model / t4_high_five` |

**APP 发布示例（打招呼）**：
```yaml
topic: /robot/input/intent
message:
  source: "app"
  intent_type: "model"
  command: "switch_model"
  action: "t4_hello"
  priority: 10
  timeout_sec: 0.5
```

### 舞蹈/组合表演（LB + RB + DPad 系列）

| F710 按键 | 动作 | RobotIntent |
|-----------|------|-------------|
| LB + RB + 方向键左 | 动太极 | `model / switch_model / t4_taichi` |
| LB + RB + 方向键上 | 静太极 | `model / switch_model / t4_taichi_w` |
| LB + RB + 方向键右 | 滑步舞蹈 | `model / switch_model / t4_mj_dance` |
| LB + RB + 方向键下 | 相亲相爱舞蹈 | `model / switch_model / t4_xqxa123_dance` |

**APP 发布示例（滑步舞蹈）**：
```yaml
topic: /robot/input/intent
message:
  source: "app"
  intent_type: "model"
  command: "switch_model"
  action: "t4_mj_dance"
  priority: 10
  timeout_sec: 0.5
```

> **注意**：`action` 是模型注册名，需在 `config/humanoid/rl_models.yaml`（人形）或 `config/dog/rl_models.yaml`（四足）中注册才能执行。

### 优雅退出循环动作

对于持续循环播放的交互动作（如 t4_hello、t4_clap），可以用 `model/finish_loop` 让当前动作播完最后一帧后自然回到行走模型，比直接切换模型更平滑。

- **F710 按键**：`Back 长按 1 秒`
- **APP 发布**：
  ```yaml
  topic: /robot/input/intent
  message:
    source: "app"
    intent_type: "model"
    command: "finish_loop"
    action: ""
    priority: 10

  ```

> 非循环模型收到 `finish_loop` 无副作用。

### 运动停止/恢复

通过 `model/motion_stop` 控制机器人运动停止和恢复。停止时强制切回 t4_walk 零速站立，恢复时回到正常运动状态。

- **F710 按键**：`Back + Start 长按 3 秒`（在 stop/resume 间循环切换）
- **APP 发布（停止）**：
  
  ```yaml
  topic: /robot/input/intent
  message:
    source: "app"
    intent_type: "model"
    command: "motion_stop"
    action: "stop"
    priority: 10

  ```
- **APP 发布（恢复）**：
  
  ```yaml
  topic: /robot/input/intent
  message:
    source: "app"
    intent_type: "model"
    command: "motion_stop"
    action: "resume"
    priority: 10

  ```

> 系统收到 `stop` 后会播报"运动已停止"，收到 `resume` 后播报"运动已恢复"。

---

## 四、配置操作

配置消息分为两种语义：
- **进入菜单**：`command` 与 `action` 相同（如 `config/getup/getup`），表示当前处于该配置菜单
- **设置选项**：`command` 为菜单名，`action` 为具体配置值（如 `config/getup/stand`），表示设置该菜单下某个选项

| 配置项 | 进入菜单 (command=action) | 设置选项 (action 为具体值) |
|--------|--------------------------|---------------------------|
| 起身姿态 | `config / getup / getup` | `config / getup / stand` / `lie` / `face` |
| 速度模式 | `config / speed / speed` | `config / speed / std` / `high` |
| 导航模式 | `config / nav / nav` | `config / nav / manual` / `auto` |
| 定制模式 | `config / mode / mode` | `config / mode / rl` / `mix` / `spec`（spec=特技模式，仅遥控器映射层概念，APP/API 不受限） |

### 起身姿态配置

设置下次准备（`status/ready`）时使用的起身姿态。

- **进入起身配置菜单**（F710: `LT 按住 + RT 三连击` 切换到 getup 菜单）：
  ```yaml
  topic: /robot/input/intent
  message:
    source: "app"
    intent_type: "config"
    command: "getup"
    action: "getup"        # command == action → 标识当前在起身配置菜单
    priority: 10

  ```
- **设置具体姿态**（F710: `RT 按住 + LT 三连击` 选姿）：
  ```yaml
  topic: /robot/input/intent
  message:
    source: "app"
    intent_type: "config"
    command: "getup"
    action: "stand"        # stand | lie | face
    priority: 10

  ```

### 速度模式配置

- **进入速度菜单**：`config / speed / speed`
- **设置标准速度**：`config / speed / std`
- **设置高速模式**：`config / speed / high`

### 导航模式配置

- **进入导航菜单**：`config / nav / nav`
- **设置遥控模式**：`config / nav / manual`
- **设置自动导航**：`config / nav / auto`

---

## 五、安全操作

### 故障复位

- **F710 按键**：`Start 长按 1 秒`
- **APP 发布**：
  ```yaml
  topic: /robot/input/intent
  message:
    source: "app"
    intent_type: "reset"
    command: "reset_fault"
    action: ""
    priority: 10

  ```

---



## 六、附录：APP 需订阅的反馈话题

| 话题 | 类型 | 用途 |
|------|------|------|
| `state` | `RobotState` | 监听系统状态变化，状态机当前阶段 |
| `audio/voice_text` | `VoiceText` | 获取语音播报（菜单提示/状态播报） |
| `router/rejected_intent` | `RobotIntent` | 意图被拒时获得拒绝反馈 |
| `driver/status` | `DriverStatus` | 驱动侧状态（在线/故障/使能） |

### 关键状态码速查（`RobotState.state`）

| 值 | 常量名 | 含义 |
|----|--------|------|
| 0 | STATE_BOOTING | 启动中 |
| 1 | STATE_INIT | 初始化中 |
| 2 | STATE_IDLE | 空闲，可触发 `status/ready` |
| 3 | STATE_DO_READY | 准备插值中（起立中） |
| 4 | STATE_READY | 站立完成，可触发 `status/walk` |
| 5 | STATE_RL_STANDBY | RL 模型待命 |
| 6 | STATE_RL_ACTIVE | RL 控制运行中（可发 motion 和 model 指令） |
| 7 | STATE_RULE_STANDBY | 规则控制器待命 |
| 8 | STATE_RULE_ACTIVE | 规则控制运行中 |
| 9 | STATE_TRANSITIONING | 模式/动作/控制源切换过渡中 |
| 10 | STATE_FAULT | 故障 |
| 11 | STATE_DAMP | 阻尼状态 |
| 12 | STATE_SHUTDOWN | 关闭或下电流程 |

---

## 七、完整操作流程速查表

| 序号 | 操作 | 前置状态 | RobotIntent (intent_type/command/action) | 目标状态 |
|------|------|----------|------------------------------------------|----------|
| 1 | 准备(起立) | IDLE | `status/ready/""` | DO_READY → READY |
| 2 | 启用行走 | READY | `status/walk/""` | RL_STANDBY → RL_ACTIVE |
| 3 | 行走控制 | RL_ACTIVE | `motion/walk/""` + 速度 | RL_ACTIVE |
| 4 | 停止 | RL_ACTIVE | `motion/walk/""` + 零速度 | RL_ACTIVE |
| 5 | 交互动作 | RL_ACTIVE | `model/switch_model/{action}` | RL_ACTIVE |
| 6 | 退出循环动作 | RL_ACTIVE | `model/finish_loop/""` | RL_ACTIVE → 播完回 walk |
| 7 | 停止/恢复运动 | RL_ACTIVE | `model/motion_stop/stop\|resume` | RL_ACTIVE |
| 8 | 阻尼/坐下 | 任意 | `status/damp/""` | DAMP |
| 9 | 重新初始化 | DAMP | `status/reinit/""` | IDLE |
| 10 | 配置起身(菜单) | READY/DO_READY | `config/getup/getup` | 不变 |
| 11 | 配置起身(设置) | READY/DO_READY | `config/getup/stand\|lie\|face` | 不变 |
| 12 | 配置速度 | 任意 | `config/speed/std\|high` | 不变 |
| 13 | 配置导航 | 任意 | `config/nav/manual\|auto` | 不变 |
| 14 | 故障复位 | FAULT | `reset/reset_fault/""` | IDLE |

---

> **一句话总结**：APP 全部操作 = 构建对应 `RobotIntent` 消息，发布到 `input/intent` 即可。
> 参考实现见 `robot_teleop/robot_teleop/teleop_joy.py`（F710 映射逻辑）和 `robot_teleop/robot_teleop/intent_builder.py`（消息构建）。

> **注意**：PDF "人形机器人使用说明书-v1.4.pdf" 第 4.3 节的遥控器按键布局与 `docs/RobotInter对应关系.md` 一致。本文档已将 F710 按键组合直接映射为 RobotIntent 消息，APP 开发时无需关心按键映射细节，直接构造对应的 RobotIntent 消息即可。
