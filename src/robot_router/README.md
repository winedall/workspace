# robot_router

命令仲裁与路由中枢。在多源（遥控/App/API/RL 管理器/外部控制）竞争中执行优先级仲裁、安全准入和命令合并，确保仅有一条安全合法的命令流进入下游控制管线。

## 架构

```
                   ┌─────────────────────────────┐
  input/intent (RobotIntent)              │
  rl_manager/desired_commands             │
  external/desired_commands               │
  state                                   │  CommandRouter (纯逻辑，可单测)
  driver/status                           │
  control/hardware_states                 │
                   └──────────────┬──────────────┘
                                  ↓
                   ┌─────────────────────────────┐
                   │  意图评估 → 优先级抢占 →     │
                   │  关节命令评估 → 关节名校验 →  │
                   │  source 授权 → 控制模式合并   │
                   └──────────────┬──────────────┘
                                  ↓
  router/active_intent ─────────────────→ robot_state, robot_rl_manager
  router/accepted_commands ──────────────→ robot_control
  router/rejected_intent ────────────────→ 诊断用
  external/hardware_states ──────────────→ robot_external
```

`CommandRouter` 不持有 `rclcpp` 引用——所有外部状态通过 `updateSystemState()`/`updateDriverStatus()` 注入，核心逻辑可脱离 ROS 运行时单元测试。

## 意图评估（3 道门）

每道门返回 false 即停止处理并输出 `RouteDecision{false, reason}`：

**门 A — 系统阻塞**：`STATE_FAULT` 或 `fault_active` → 仅放行 `status/damp`（手动阻尼意图），其他全部拒绝。

**门 B — DO_READY 模型切换**：`STATE_DO_READY` 期间拒绝 `model` 类型意图（阻尼允许中断准备过程）。

**门 C — 优先级抢占**：候选意图优先级 ≤ 当前活跃优先级（≤ = 数字更小/优先级更高）。无活跃意图时任何候选通过。

## 关节命令评估（6 道门）

| 门 | 检查 | 拒绝条件 |
|----|------|----------|
| A | 空帧 | `commands` 为空 |
| B | 系统阻塞 | FAULT 且非故障阻尼命令 |
| C | 驱动可用 | `DriverStatus.online=false` 或 `fault_active=true` |
| D | 序列单调性 | `sequence ≤` 同 source 上次接受值 |
| E | Source 授权 | source 不在允许列表中 |
| F | 关节名校验 | 未知关节名或同帧内重复 |

### 故障阻尼命令条件（全部满足才放行）
- 系统处于阻塞状态
- 有活跃意图且为手动阻尼（`status/damp`）
- 命令 source 为 `"rl_manager"` 且 type 为 `TYPE_DAMPING`

### Source 授权规则
`rl_manager` 在以下状态直接放行：`DAMP`、`IDLE`、`DO_READY`、`READY`、`RL_STANDBY`、`RL_ACTIVE`。其他 source 需匹配活跃意图的 source。

## 优先级（按来源分段）

| 来源 | 有效优先级 | 默认范围 |
|------|-----------|----------|
| `"teleop"` | 固定 | 0（最高） |
| `"app"` | clamp(msg.priority, min, max) | [1, 49] |
| `"api"`/`"http"`/`"mqtt"`/`"cloud"` | clamp(msg.priority, min, max) | [50, 99] |
| 未知来源 | 固定 | 255（最低） |

**核心规则**：来源身份决定优先级段，消息内的 `priority` 字段仅在同一段内微调。遥控始终击败 App，App 始终击败 API。

## 控制模式

| 模式 | 行为 |
|------|------|
| **RL** | 纯 RL 控制，外部命令被忽略 |
| **MIX** | 外部命令按关节覆盖 RL 命令（position/kp/kd 用外部值，enable = RL ∧ 外部） |

通过 `config/mode` 意图的 `action="rl"` 或 `action="mix"` 切换。

## 输出重写

接受的 `JointCommands` 由路由器重新封装：

- `parent_sequence` / `parent_source` = 原始帧的 sequence/source
- `sequence` = 路由器级自增输出序列（从 1 开始）
- `source` = `"command_router"`
- `stage` = `"accepted"`
- `trace_id` 保留原值（贯穿全链路追踪）

## 外部关节状态过滤

收到 `control/hardware_states` 时，提取当前外部命令涉及的关节子集，发布到 `external/hardware_states`，供外部控制器使用。

## ROS 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `robot_joints_config` | `""` | 关节白名单 YAML 路径 |
| `allow_debug_joint_command` | `false` | 允许 `source="debug"` 的关节命令 |
| `allow_motion_when_hardware_disabled` | `false` | 仿真模式：绕过硬件使能检查 |
| `teleop_priority` | `0` | 遥控固定优先级 |
| `app_priority_min` / `max` | `1` / `49` | App 优先级范围 |
| `api_priority_min` / `max` | `50` / `99` | API 优先级范围 |
| `unknown_priority` | `255` | 未知来源优先级 |

## 保守默认

`CommandRouter` 构造时使用最安全初值：`STATE_BOOTING` + `DriverStatus.online=false` + `STATUS_UNKNOWN`。在真实状态到达前不允许任何运动命令通过。

## 订阅与发布

**输入（6 个）：**
`input/intent`、`rl_manager/desired_commands`、`external/desired_commands`、`control/hardware_states`、`state`、`driver/status`

**输出（4 个）：**
`router/active_intent`、`router/accepted_commands`、`router/rejected_intent`、`external/hardware_states`
