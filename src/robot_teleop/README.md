# robot_teleop

遥控输入节点。支持 **F710 手柄**和**键盘**两种输入方式，将原始输入转换为统一的 `RobotIntent` 消息发布。

## 转换管线

```
F710 硬件
  → sensor_msgs/Joy (/joy)
    → TeleopJoyMapper.map_joy()          ← 按钮/轴解析，死区，长按检测，组合键匹配
      → IntentSpec                       ← 中间数据类
        → RobotTeleopNode._handle_joy()  ← 连续运动缓存 / 一次性事件入队
          → 发布定时器 20 Hz             ← 超时检查 / 优先一次性事件
            → IntentBuilder.build()      ← RobotIntent 组装
              → input/intent
```

## F710 按键映射

| 按钮 | 索引 | 功能 |
|------|------|------|
| A | 0 | LB+RB+A：`status/reinit`（回到待机） |
| B | 1 | LB+RB+B：`status/ready`（准备站立） |
| X | 2 | LB+RB+X：`status/walk`（授权运动） |
| Y | 3 | LB+RB+Y：`status/damp`（阻尼模式） |
| LB | 4 | LB+DPad：技能触发（左侧列） |
| RB | 5 | RB+DPad：技能触发（右侧列） |
| LT | 6 | 按住 + RT 三击：配置菜单 |
| RT | 7 | 按住 + LT 三击：选项切换 |
| Back | 8 | 长按 1s：退出循环 / 按住+Start 3s：运动停止 |
| Start | 9 | 长按 1s：故障复位 |

| 轴 | 索引 | 功能 |
|----|------|------|
| 左摇杆水平 | 0 | 横向运动速度 |
| 左摇杆垂直 | 1 | 前后运动速度 |
| 右摇杆水平 | 2 | 偏航角速度 |
| DPad 水平 | 4 | 方向选择（±1） |
| DPad 垂直 | 5 | 方向选择（±1） |

## 技能快捷键（人形）

**LB + DPad：**

| DPad | LB |
|------|-----|
| 上 | `t4_clap` |
| 下 | `t4_squat_down` |
| 左 | `t4_squat_up` |
| 右 | `t4_rhello` |

**RB + DPad：**

| DPad | RB |
|------|-----|
| 上 | `t4_salute` |
| 下 | `t4_high_five` |
| 左 | `t4_handshake` |
| 右 | `t4_heart` |

**LB+RB + DPad（普通模式）：**

| DPad | 技能 |
|------|------|
| 上 | `t4_taichi_w` |
| 下 | `t4_xqxa123_dance` |
| 左 | `t4_mj_dance` |
| 右 | `t4_taichi` |

**LB+RB + DPad（Spec 模式）：**

| 方向 | 技能 |
|------|------|
| 左 | `t4_long_jump` |
| 右 | `t4_side_flip` |

## 配置菜单

LT 按住 + RT 三击：循环主菜单 `getup → speed → nav → mode`
RT 按住 + LT 三击：切换当前菜单选项（如 `speed: std → high`、`mode: rl → mix → spec`）

## 安全机制

- **运动授权门**：上电后 `_motion_enabled = false`，必须按 LB+RB+X 才启用运动输出
- **摇杆回中**：授权后三个运动轴必须全部进入死区一次才输出非零速度（防止手柄缓存值被误读）
- **校准故障锁存**：三轴同时超过 0.98 绝对值 → 锁存零速度，直到全部回死区自动恢复
- **边缘触发**：所有组合键只触发一次，按住不动不重复
- **Deadman 开关**：手柄信号丢失超过 `joy_timeout_sec`（0.3s）且在 RL_ACTIVE 下 → 强制输出零速度
- **长按检测**：Back/Start 长按 1s 才触发，Back+Start 长按 3s 才触发

## ROS 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `source_name` | `"teleop"` | 意图来源标识 |
| `publish_frequency_hz` | `20.0` | 意图发布频率 |
| `max_linear_x_mps` | `1.0` | 正常前进速度上限 |
| `max_linear_y_mps` | `0.5` | 正常横向速度上限 |
| `max_linear_x_high_mps` | `3.0` | 高速前进上限 |
| `max_linear_y_high_mps` | `1.0` | 高速横向上限 |
| `max_angular_z_radps` | `1.5` | 偏航角速度上限 |
| `enable_joy` | `true` | 订阅 /joy 话题 |
| `joy_deadzone` | `0.08` | 摇杆死区 |
| `dpad_threshold` | `0.5` | DPad 方向阈值 |
| `robot_type` | `"humanoid"` | 机器人类型（影响快捷键映射） |
| `joy_timeout_sec` | `0.3` | 手柄信号超时 |

## 键盘遥控（`robot_keyboard_node`）

基于 pynput 全局按键监听，不抢占终端。通过 `enable_keyboard:=true` 启动。

### 按键映射

| 按键 | 功能 |
|------|------|
| `B` | `status/ready` 起立 |
| `R` | `status/walk` 启用行走 |
| `Y` | `status/damp` 阻尼/坐下 |
| `X` | `status/reinit` 重新初始化 |
| `/` | `model/finish_loop` 结束循环 |
| `1`~`0` | 技能切换（`t4_hello`/`clap`/`squat_up`/`squat_down`/`heart`/`salute`/`handshake`/`high_five`/`long_jump`/`standby`） |

按键事件在 pynput 回调线程入队，ROS 定时器主线程出队并发布。

## 话题

| 类型 | 话题 | 说明 |
|------|------|------|
| Sub | `/joy` | 原始手柄输入（手柄节点） |
| Sub | `state` | 系统状态（超时只在 RL_ACTIVE 下生效） |
| Pub | `input/intent` | 统一意图输出（手柄和键盘共用） |
