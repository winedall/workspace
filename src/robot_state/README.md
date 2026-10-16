# robot_state

机器人系统状态机。管理全机器人生命周期，是所有模块安全准入的权威来源。监控驱动状态、关节健康、RL 管理器状态，在异常时触发故障保护。

## 状态转移图

```
BOOTING ──[参数加载]──→ INIT ──[配置加载]──→ IDLE
                                              │
                    ┌─────────────────────────┤
                    │                         │
              [自检失败]                  [自检通过]
                    │                         │
                    ↓                         ↓
                  FAULT              IDLE (自检完成)
                    ↑                         │
                    │               [status/ready + 就绪条件]
                    │                         │
                    │              ┌──────────┤
                    │              │          │
                    │         DO_READY    [status/damp]
                    │              │          │
                    │    [准备完成] │          ↓
                    │              │        DAMP
                    │              ↓          │
                    │            READY ───────┘
                    │              │
                    │    ┌─────────┼─────────┐
                    │    │         │         │
                    │ [rl]     [rule]   [damp]
                    │    │         │         │
                    │    ↓         ↓         ↓
                    │ RL_STANDBY RULE_STANDBY DAMP
                    │    │         │
                    │ [rl就绪] [rule_active]
                    │    │         │
                    │    ↓         ↓
                    │ RL_ACTIVE RULE_ACTIVE
                    │    │         │
                    └────┼─────────┘
                    [驱动/关节/RL故障]
```

## 13 个系统状态

| 状态 | 常量 | 说明 |
|------|------|------|
| `BOOTING` | 0 | 启动中，等待参数 |
| `INIT` | 1 | 初始化，加载配置 |
| `IDLE` | 2 | 空闲，等待指令 |
| `DO_READY` | 3 | 执行站立/准备插补 |
| `READY` | 4 | 就绪，可接受高层控制 |
| `RL_STANDBY` | 5 | RL 模型就绪但未输出 |
| `RL_ACTIVE` | 6 | RL 控制运行中 |
| `RULE_STANDBY` | 7 | 规则控制器就绪但未输出 |
| `RULE_ACTIVE` | 8 | 规则控制运行中 |
| `TRANSITIONING` | 9 | 模式切换中 |
| `FAULT` | 10 | 故障，等待确认或恢复 |
| `DAMP` | 11 | 阻尼模式 |
| `SHUTDOWN` | 12 | 关机中 |

## ROS 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `robot_joints_config` | string | `""` | 关节配置 YAML 路径 |
| `state_publish_frequency_hz` | double | `20.0` | 状态发布频率 |
| `use_mujoco_sim` | bool | `false` | 仿真模式（跳过零点通信检测） |

## 话题与服务

| 类型 | 名称 | 用途 |
|------|------|------|
| **Sub** | `control/hardware_states` | 关节反馈（自检用） |
| **Sub** | `driver/status` | 驱动状态 |
| **Sub** | `rl_manager/status` | RL 管理器状态 |
| **Sub** | `router/active_intent` | 当前活跃意图 |
| **Sub** | `control/preparation_progress` | 准备插补进度 |
| **Pub** | `state` | 系统状态（权威发布） |
| **Pub** | `audio/play` | 预录制提示音播报（状态变更时，经 `AudioClient`） |
| **Srv** | `state/set_mode` | 设置运行模式 |
| **Srv** | `state/reset_fault` | 故障复位 |

## 意图处理（handleIntent）

| intent_type | command | 允许状态 | 效果 |
|-------------|---------|----------|------|
| `status` | `damp` | 任意（含 FAULT） | 进入 DAMP（FAULT 下接受但不离开 FAULT） |
| `status` | `reinit` | DAMP | 回到 IDLE |
| `status` | `ready` | IDLE + 就绪条件 | 进入 DO_READY |
| `status` | `walk` | READY + 就绪条件 | 进入 RL_STANDBY |
| `mode` | `switch_mode` | READY | 切换到 rl 或 rule |
| `reset` | `reset_fault` | 任意 | 尝试故障复位 |
| `motion` | 任意 | — | 忽略（不由状态机处理） |
| `model` | 任意 | — | 忽略（不由状态机处理） |

## 自检流程

```
数据稳定检测 → 至少 1/3 的 enable_default 关节有非零位置
    ↓
稳定帧计数 → 再等待 100 帧确保编码器读数稳定
    ↓
逐关节校验：
  ├── 通信错误：真机下 enable_default 关节位置恰好为 0.0（编码器不应返回精确零）
  └── 位置越界：position < min_pos 或 position > max_pos
    ↓
全部通过 → self_check_state = PASSED
任一失败 → 进入 FAULT（含关节名和具体值）
```

## 故障检测

| 故障来源 | 检测条件 | 适用范围 |
|----------|----------|----------|
| **驱动故障** | `driver_status.fault_active` 或 `status == STATUS_FAULT` | BOOTING/INIT/SHUTDOWN 之外的全部状态 |
| **关节错误** | 任何 `joint_error` 不是 `NONE` 或 `MOTOR_WARN` | 同上 |
| **RL 推理错误** | `rl_manager_status.inference_error` | 仅 RL_ACTIVE |
| **RL 未就绪** | 模型未加载或未就绪 | 仅 RL_ACTIVE |
| **自检失败** | 通信错误或位置越界 | IDLE 自检阶段 |

**注意**：`JOINT_ERR_MOTOR_WARN`（电机过温/过流/过压/欠压）不会触发故障，只触发语音播报。

## 故障复位条件

```
状态必须为 FAULT
∧ driverHealthy()     ← 驱动在线且无故障
∧ jointsHealthy()     ← 无致命关节错误
→ 回到 IDLE，fault_active=false，self_check_state 重置为 PENDING（需重新自检）
```

## 辅助函数速查

| 函数 | 条件 |
|------|------|
| `driverHealthy()` | 有驱动状态 ∧ online ∧ !fault_active ∧ status != STATUS_FAULT |
| `jointsHealthy()` | 所有关节 joint_error 为 NONE 或 MOTOR_WARN |
| `rlManagerReady()` | online ∧ model_loaded ∧ model_ready ∧ !inference_error ∧ missing_inputs 为空 |
| `readyConditionsMet()` | config_loaded ∧ !fault_active ∧ self_check == PASSED ∧ driverHealthy ∧ jointsHealthy |

## 提示音播报（v1.2.11 起）

从 TTS 文本合成迁移为**预录制 WAV 提示音**：通过 `robot_common::AudioClient`（封装 `AudioPlay` 消息）播放 `config/data/tips.yaml` 中定义的提示音文件。键值由 `scripts/audio/tips.py` 生成为 `robot_common::audio_tips` 常量。

### 电机告警提示音键

| motor_error | 提示音键 |
|-------------|----------|
| 1 | `MOTOR_OVER_TEMP_WARN`（电机过温警告） |
| 2 | `MOTOR_OVER_CURRENT_WARN`（电机过流警告） |
| 3 | `MOTOR_OVER_VOLTAGE_WARN`（电机过压警告） |
| 4 | `MOTOR_UNDER_VOLTAGE_WARN`（电机欠压警告） |
| 其他 | `MOTOR_WARN`（电机告警） |

状态变更提示音：IDLE→"待机模式"，READY→"准备就绪"，RL_ACTIVE→"行走模式"，FAULT→"系统发生故障"，自检成功→"自检成功"，自检失败→"自检失败，请检查机器人状态"。
