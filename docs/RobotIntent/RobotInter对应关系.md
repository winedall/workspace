# 罗技 F710 与 RobotIntent 对应关系

本文说明 Logitech F710 遥控器输入如何转换为 `robot_interfaces/msg/RobotIntent`。

RobotIntent 的合法字段组合以 `docs/RobotIntent说明.xlsx` 为唯一权威；本文负责描述遥控器按键、`sensor_msgs/msg/Joy` 数据与这些合法组合之间的对应关系。所有遥控输入都应先转换为 RobotIntent，经 `robot_router` 准入后再交给状态机或控制模块处理，不应直接发布 JointCommands。

RobotIntent 中与本映射直接相关的核心字段为：

```yaml
source: "teleop"
intent_type: ""
command: ""
action: ""
linear_x_mps: 0.0
linear_y_mps: 0.0
angular_z_radps: 0.0
```

其中 `action` 已统一替代旧版的 `target_mode`、`model_name` 和 `action_name`。

## 1. 罗技 F710 与 `sensor_msgs/msg/Joy` 基础映射

### 1.1 buttons 数组映射

| 遥控器按键 | Joy 数据 | 当前用途 |
|---|---:|---|
| X 蓝色键 | `buttons[0]` | 与 LB、RB 组合，发送 `status/reinit` |
| A 绿色键 | `buttons[1]` | 与 LB、RB 组合，发送 `status/ready` |
| B 红色键 | `buttons[2]` | 与 LB、RB 组合，发送 `status/walk` |
| Y 黄色键 | `buttons[3]` | 与 LB、RB 组合，发送 `status/damp` |
| LB | `buttons[4]` | 状态组合键或模型组合键 |
| RB | `buttons[5]` | 状态组合键或组合模型键 |
| LT | `buttons[6]` | D 模式下作为配置菜单按键 |
| RT | `buttons[7]` | D 模式下作为配置菜单按键 |
| Back | `buttons[8]` | 单独长按退出循环动作；与 Start 同时长按切换运动停止/恢复（motion_stop） |
| Start | `buttons[9]` | 单独长按故障复位；与 Back 同时长按切换运动停止/恢复（motion_stop） |
| Mode / Logitech | `buttons[10]` | 不参与业务控制，使用时应保持模式指示灯熄灭 |
| 左摇杆按下 | `buttons[11]` | 当前未使用 |
| 右摇杆按下 | `buttons[12]` | 当前未使用 |

`buttons[i] == 1` 表示按下，`buttons[i] == 0` 表示释放。LT、RT 的输入形式与 F710 模式有关，本文和当前代码均按 D 模式 button 输入处理。

### 1.2 axes 数组映射

| 遥控器输入 | Joy 数据 | 当前控制含义 |
|---|---:|---|
| 左摇杆左右 | `axes[0]` | 横移速度：左为正、右为负 |
| 左摇杆前后 | `axes[1]` | 前后速度：前为正、后为负 |
| 右摇杆左右 | `axes[2]` | 偏航角速度：左转为正、右转为负 |
| 右摇杆前后 | `axes[3]` | 当前未使用 |
| 方向键左右 | `axes[4]` | 左为正、右为负 |
| 方向键上下 | `axes[5]` | 上为正、下为负 |

方向键判断阈值默认为 `0.5`，摇杆死区默认为 `0.08`。不同驱动或 F710 模式可能导致轴符号变化，首次接入应使用 `ros2 topic echo /joy` 实测；如实际符号相反，应在输入映射层统一反转，不能修改 RobotIntent 语义。

---

## 2. 状态切换类组合按键 -> status 类 RobotIntent

状态组合键负责驱动系统状态机，不负责选择动作模型，也不携带连续速度。

| 遥控器组合键 | Joy 判断条件 | 功能 | RobotIntent |
|---|---|---|---|
| LB + RB + A | `buttons[4]==1 && buttons[5]==1 && buttons[1]==1` | 进入准备流程 | `status / ready / ""` |
| LB + RB + B | `buttons[4]==1 && buttons[5]==1 && buttons[2]==1` | 进入行走控制状态 | `status / walk / ""` |
| LB + RB + Y | `buttons[4]==1 && buttons[5]==1 && buttons[3]==1` | 进入阻尼状态 | `status / damp / ""` |
| LB + RB + X | `buttons[4]==1 && buttons[5]==1 && buttons[0]==1` | 从 DAMP 重新初始化到 IDLE | `status / reinit / ""` |

示例：

```yaml
source: "teleop"
intent_type: "status"
command: "ready"
action: ""
```

处理逻辑：

- `status/ready` 请求状态机进入准备流程，准备动作完成后到达 READY。
- `status/walk` 只触发 `READY -> RL_STANDBY -> RL_ACTIVE`，同时在 teleop 内部开启摇杆 motion 输出授权。
- `status/damp` 请求进入 DAMP，并关闭 teleop 的 motion 输出授权。
- `status/reinit` 只在 DAMP 状态下回到 IDLE，并关闭 motion 输出授权。
- 同一组合键持续按住只触发一次；释放后再次按下才允许重新触发。

`status/walk` 与 `model/switch_model/t4_walk` 含义不同：前者切换系统状态，后者选择名为 `t4_walk` 的动作模型。

---

## 3. 运动停止/恢复组合键 -> model 类 RobotIntent

Back+Start 长按用于在 `stop`（停止运动，切回 t4_walk 零速站立）和 `resume`（恢复运动）之间循环切换，同时播报语音提示。

| 遥控器组合键 | Joy 判断条件 | 功能 | RobotIntent |
|---|---|---|---|
| Back + Start 长按 3 秒 | `buttons[8]==1 && buttons[9]==1`，持续时间 `>= 3.0 s` | 在 stop/resume 间循环切换 | `model / motion_stop / stop\|resume` |

示例：

```yaml
source: "teleop"
intent_type: "model"
command: "motion_stop"
action: "stop"
```

注意：

- 第一次有效长按发送 `action="stop"`，下一次有效长按发送 `action="resume"`，之后循环。
- 长按未满 3 秒不发布 RobotIntent。
- 同一次长按只发布一次；释放 Back 或 Start 后才重新计时。
- 输入时间戳回退时重新开始计时，避免异常时间戳导致误触发。
- `stop` 会强制切回 t4_walk 模型并进入零速站立状态；`resume` 恢复正常的模型运动和摇杆控制。
- 执行 `stop` 或 `resume` 时会播报"运动已停止"或"运动已恢复"语音提示。

---

## 4. 摇杆运动控制 -> motion 类 RobotIntent

只有成功触发过 `status/walk` 后，teleop 才允许摇杆产生 motion 意图。左摇杆控制前后与横移，右摇杆左右控制偏航。

| 遥控器输入 | Joy 数据 | RobotIntent 字段 |
|---|---:|---|
| 左摇杆前推/后拉 | `axes[1]` | `linear_x_mps = axes[1] * max_linear_x_mps` |
| 左摇杆左推/右推 | `axes[0]` | `linear_y_mps = axes[0] * max_linear_y_mps` |
| 右摇杆左推/右推 | `axes[2]` | `angular_z_radps = axes[2] * max_angular_z_radps` |
| 右摇杆前后 | `axes[3]` | 当前忽略 |
| 摇杆回中 | 三个有效轴进入死区 | 发布零速度 `motion / walk / ""` |

示例：

```yaml
source: "teleop"
intent_type: "motion"
command: "walk"
action: ""
linear_x_mps: 0.3
linear_y_mps: 0.0
angular_z_radps: 0.1
```

运动控制约束：

- `abs(axis) < deadzone` 时按 `0` 处理，默认死区为 `0.08`。
- `config/speed/std` 使用标准速度比例，`config/speed/high` 使用高速比例。
- 摇杆回中和 Joy 超时都使用零速度 `motion/walk` 表示停止。
- 新版协议不定义 `motion/stop` 或 `motion/stand`。
- 进入 `ready/damp/reinit` 等非 walk 状态后停止周期发布 motion。
- 检测到多个运动轴同时卡在极值附近时锁存校准错误并输出零速度，避免 F710 校准异常产生误运动。

---

## 5. 单动作交互组合键 -> model 类 RobotIntent

说明书中的单动作交互组合键包括打招呼、鼓掌、蹲起、下蹲、比心、敬礼、握手和击掌。按键布局保持与旧版一致，LB 负责左侧四个动作，RB 负责右侧四个动作。

所有动作模型统一采用新版消息结构：

```text
intent_type="model", command="switch_model", action=<动作模型名>
```

| 遥控器组合键 | Joy 判断条件 | 功能 | 目标 RobotIntent | 当前状态 |
|---|---|---|---|---|
| LB + 方向键左 | `buttons[4]==1 && axes[4]>0.5` | 打招呼 | `model / switch_model / t4_hello` | 已实现 |
| LB + 方向键上 | `buttons[4]==1 && axes[5]>0.5` | 鼓掌 | `model / switch_model / t4_clap` | 已实现 |
| LB + 方向键右 | `buttons[4]==1 && axes[4]<-0.5` | 蹲起 | `model / switch_model / t4_squat_up` | 已实现 |
| LB + 方向键下 | `buttons[4]==1 && axes[5]<-0.5` | 下蹲 | `model / switch_model / t4_squat_down` | 已实现 |
| RB + 方向键左 | `buttons[5]==1 && axes[4]>0.5` | 比心 | `model / switch_model / t4_heart` | 已实现 |
| RB + 方向键上 | `buttons[5]==1 && axes[5]>0.5` | 敬礼 | `model / switch_model / t4_salute` | 已实现 |
| RB + 方向键右 | `buttons[5]==1 && axes[4]<-0.5` | 握手 | `model / switch_model / t4_handshake` | 已实现 |
| RB + 方向键下 | `buttons[5]==1 && axes[5]<-0.5` | 击掌 | `model / switch_model / t4_high_five` | 已实现 |

已支持动作示例：

```yaml
source: "teleop"
intent_type: "model"
command: "switch_model"
action: "t4_hello"
```

循环动作示例：

```yaml
source: "teleop"
intent_type: "model"
command: "switch_model"
action: "t4_clap"
```

单动作交互的所有按键组合均已映射到注册模型名。RobotIntent 的 action 必须是注册模型名；未注册的模型名不会被 `robot_router` 拦截，而是由 RL manager 在切换时拒绝（`KeyError` 并返回失败），不能作为已实现功能使用。

---

## 6. 舞蹈/组合表演组合键 -> model 类 RobotIntent

说明书中的舞蹈和组合表演包括动太极、静太极、滑步舞蹈和相亲相爱舞蹈。LB + RB + DPad 的解析优先级高于 LB + DPad，避免组合表演被误判为单动作。

> 特技模式（spec）：通过模式菜单切换到 `config/mode/spec` 后，LB+RB+方向键左=侧空翻 `t4_side_flip`、右=跳远 `t4_long_jump`、上/下切换到 `t4_none`；正常模式保持下表映射。

| 遥控器组合键 | Joy 判断条件 | 功能 | 目标 RobotIntent | 当前状态 |
|---|---|---|---|---|
| LB + RB + 方向键左 | `buttons[4]==1 && buttons[5]==1 && axes[4]>0.5` | 动太极舞蹈 | `model / switch_model / t4_taichi` | 已实现 |
| LB + RB + 方向键上 | `buttons[4]==1 && buttons[5]==1 && axes[5]>0.5` | 静太极舞蹈 | `model / switch_model / t4_taichi_w` | 已实现 |
| LB + RB + 方向键右 | `buttons[4]==1 && buttons[5]==1 && axes[4]<-0.5` | 滑步舞蹈 | `model / switch_model / t4_mj_dance` | 已实现 |
| LB + RB + 方向键下 | `buttons[4]==1 && buttons[5]==1 && axes[5]<-0.5` | 相亲相爱舞蹈 | `model / switch_model / t4_xqxa123_dance` | 已实现 |

示例：

```yaml
source: "teleop"
intent_type: "model"
command: "switch_model"
action: "t4_mj_dance"
```

动太极与静太极分别使用 `t4_taichi` 和 `t4_taichi_w` 两个独立注册模型，RobotIntent 可通过 action 区分两者。

RobotIntent 中的 action 是模型注册名，不是模型文件路径。即使 action 已纳入 Excel，也必须在 `config/humanoid/rl_models.yaml` 中注册后才能实际执行；未注册模型会由 RL manager 拒绝并报告错误。

---

## 7. 配置菜单组合键 -> config 类 RobotIntent

配置菜单通过 LT、RT 的三次上升沿触发，菜单操作期间不产生 motion 意图。

| 遥控器组合键 | Joy 判断条件 | 功能 |
|---|---|---|
| LT 按住 + RT 连按 3 次 | `buttons[6]==1`，检测 `buttons[7]` 三次上升沿 | 切换一级菜单并发布菜单提示 |
| RT 按住 + LT 连按 3 次 | `buttons[7]==1`，检测 `buttons[6]` 三次上升沿 | 切换当前菜单的二级选项 |

一级菜单按以下顺序循环：

```text
getup -> speed -> nav -> mode -> getup
```

一级菜单切换完成后，`command` 和 `action` 使用同名值：

| 一级菜单 | 功能 | RobotIntent |
|---|---|---|
| `getup` | 起身配置菜单 | `config / getup / getup` |
| `speed` | 速度配置菜单 | `config / speed / speed` |
| `nav` | 导航配置菜单 | `config / nav / nav` |
| `mode` | 控制模式配置菜单 | `config / mode / mode` |

二级菜单选项变化时，command 保持当前菜单名，action 使用配置值：

| 配置菜单 | 二级选项 | 功能 | RobotIntent |
|---|---|---|---|
| `getup` | `stand` | 站立准备姿态 | `config / getup / stand` |
| `getup` | `lie` | 躺姿准备姿态 | `config / getup / lie` |
| `getup` | `face` | 趴姿准备姿态 | `config / getup / face` |
| `speed` | `std` | 标准速度 | `config / speed / std` |
| `speed` | `high` | 高速速度 | `config / speed / high` |
| `nav` | `manual` | 遥控模式 | `config / nav / manual` |
| `nav` | `auto` | 自动导航模式 | `config / nav / auto` |
| `nav` | `follow` | 跟随模式 | `config / nav / follow` |
| `mode` | `rl` | 纯 RL 模式 | `config / mode / rl` |
| `mode` | `mix` | 混合模式 | `config / mode / mix` |
| `mode` | `spec` | 特技模式（仅影响遥控器按键映射） | `config / mode / spec` |

一级菜单提示示例：

```yaml
source: "teleop"
intent_type: "config"
command: "speed"
action: "speed"
```

二级配置变化示例：

```yaml
source: "teleop"
intent_type: "config"
command: "speed"
action: "high"
```

配置意图只修改运行参数或发布菜单提示，不直接触发状态跳转或动作执行。`config/getup` 的新选择在下一次准备流程中生效，避免站立过程中直接改变目标姿态。

---

## 8. Back 长按优雅退出循环动作

对于 t4_hello、t4_clap 等持续循环播放参考运动的模型，通过 `model/finish_loop` 让当前动作播完最后一帧后自动回退到 t4_walk，避免突然中断运动。

| 遥控器组合键 | Joy 判断条件 | 功能 | RobotIntent |
|---|---|---|---|
| Back 单独长按 1 秒 | `buttons[8]==1 && buttons[9]==0`，持续时间 `>= 1.0 s` | 优雅退出当前循环动作 | `model / finish_loop / ""` |

示例：

```yaml
source: "teleop"
intent_type: "model"
command: "finish_loop"
action: ""
```

注意：

- Back 与 Start 同时按下时属于 motion_stop 组合，不得同时触发 finish_loop。
- 一次长按只发布一次，释放 Back 后才允许重新计时。
- 只有当前模型支持循环（如 t4_hello、t4_clap）时才有实际效果；非循环模型收到后无副作用。
- 与直接发送 `model/switch_model/t4_walk` 不同，finish_loop 让当前运动帧自然播放完毕再回退，效果更平滑。

---

## 9. 故障复位与输入解析优先级

### 9.1 Start 长按故障复位

| 遥控器组合键 | Joy 判断条件 | 功能 | RobotIntent |
|---|---|---|---|
| Start 单独长按 1 秒 | `buttons[9]==1 && buttons[8]==0`，持续时间 `>= 1.0 s` | 请求故障复位 | `reset / reset_fault / ""` |

示例：

```yaml
source: "teleop"
intent_type: "reset"
command: "reset_fault"
action: ""
```

Start 与 Back 同时按下时属于 motion_stop 组合，不得同时触发 reset。一次长按只发布一次，释放 Start 后才允许重新计时。

新版 RobotIntent 还定义了 `reset / reset_fault`，通过 Start 长按触发。

### 8.2 建议解析优先级

当前 `teleop_joy.py` 按以下顺序解析 `/joy`，优先级用于解决组合键重叠：

```text
1. Back + Start 长按：model / motion_stop
2. Start 单独长按：reset / reset_fault
3. Back 单独长按：model / finish_loop
4. LB + RB + A/B/X/Y：status
5. LB + RB + DPad：组合 model
6. LB/RB + DPad：单动作 model
7. LT/RT 三连击：config
8. 摇杆连续输入：motion / walk
```

关键约束：

- `LB + RB + DPad` 必须先于 `LB + DPad` 判断。
- Back 或 Start 参与长按判断时，不继续解析普通动作和 motion。
- LT/RT 配置操作期间不落入零速度 motion。
- status、model、reset 和 config 属于事件型意图；同一次持续按键不重复发布。
- motion 属于周期型意图，只在 `status/walk` 授权后持续发布。
