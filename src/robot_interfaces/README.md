# robot_interfaces

机器人系统全域 ROS 2 接口定义包。定义了所有节点之间通信的消息（`.msg`）和服务（`.srv`），是系统的 **API 契约层**——所有其他包都依赖此包，任何字段变更都会影响下游。

## 消息定义（14 个）

### 控制管线消息

| 消息 | 发布者 | 用途 |
|------|--------|------|
| `JointCommand` | 单关节命令（名称、位置、速度、力矩、Kp/Kd、使能） | RL/遥控/外部控制输出 |
| `JointCommands` | 多关节命令帧（含 trace_id、sequence、source、stage、type） | 控制管线全链路追踪 |
| `JointState` | 单关节反馈（位置、速度、力矩、温度、故障码） | 驱动层硬件反馈 |
| `JointStates` | 多关节反馈帧（含序列号、最后执行命令的 trace_id） | 驱动层周期性发布 |

**JointCommands.type 枚举：**
- `TYPE_NONE=0` — 常规执行命令
- `TYPE_DAMPING=2` — 阻尼命令
- `TYPE_PREPARE=3` — 准备插补命令
- `TYPE_RUN=4` — 模型推理输出

**JointState.joint_error 枚举：**
- `JOINT_ERR_NONE=0` — 正常
- `JOINT_ERR_MOTOR_WARN=1` — 电机驱动告警
- `JOINT_ERR_INVALID_ANGLE_STATE=2` — 反馈角度异常（超限或跳变）
- `JOINT_ERR_INVALID_ANGLE_CMD=3` — 目标角度指令异常
- `JOINT_ERR_ZERO_ABSENCE=4` — 零点缺失或状态异常

### 系统状态消息

| 消息 | 发布者 | 用途 |
|------|--------|------|
| `RobotState` | `robot_state` | 系统生命周期状态（BOOTING → INIT → IDLE → READY → RL_ACTIVE 等 13 个状态） |
| `RobotIntent` | `robot_teleop` / API / 外部控制 | 统一高层控制意图（运动速度、模式切换、模型切换） |
| `DriverStatus` | `robot_driver` | 驱动状态（ONLINE/OFFLINE/DEGRADED/FAULT + 故障关节列表） |
| `RlManagerStatus` | `robot_rl_manager` | RL 管理器状态（模型加载、推理状态、延迟、IMU 可用性） |
| `SystemStatus` | `robot_monitor` | 系统健康监控（CPU/内存/磁盘/电池/音频 + 阈值告警） |

**RobotState.state 枚举：**
| 常量 | 值 | 说明 |
|------|-----|------|
| `STATE_BOOTING` | 0 | 启动中 |
| `STATE_INIT` | 1 | 初始化 |
| `STATE_IDLE` | 2 | 空闲 |
| `STATE_DO_READY` | 3 | 执行站立/准备插补 |
| `STATE_READY` | 4 | 就绪，可接受高层控制 |
| `STATE_RL_STANDBY` | 5 | RL 模型就绪但未输出 |
| `STATE_RL_ACTIVE` | 6 | RL 控制运行中 |
| `STATE_FAULT` | 10 | 故障状态 |
| `STATE_DAMP` | 11 | 阻尼状态 |
| `STATE_SHUTDOWN` | 12 | 关机中 |

### 外设/工具消息

| 消息 | 发布者 | 用途 |
|------|--------|------|
| `BatteryInfo` | `robot_battery` | BMS 电池状态（电压/电流/SOC/温度/MOS 状态等字段） |
| `HeadControl` | 外部控制 | 头部 yaw/pitch 角度指令（度，支持相对/绝对模式） |
| `VoiceText` | 任意节点 | TTS 语音文本 + 优先级 |
| `AudioPlay` | `robot_rl_manager` / 任意节点 | 预录制 WAV 音频播放（文件路径 + 优先级 + 中断开关） |
| `SpeakStatus` | `robot_audio` | TTS 播放状态（IDLE/PLAYING/PAUSED） |
| `VersionInfo` | `robot_monitor` | 系统版本号 + 发布日期 |

## 服务定义（12 个）

### 控制类服务

| 服务 | 用途 |
|------|------|
| `SetRobotMode` | 设置机器人运行模式（idle/ready/rl/rule/debug/fault_recovery） |
| `ResetFault` | 故障复位（需提供复位原因） |
| `FollowControl` | 进入/退出跟随模式（ENTER_TRACK_MODE / EXIT_TRACK_MODE） |

### RL 模型管理

| 服务 | 用途 |
|------|------|
| `LoadRlModel` | 加载 RL 模型（指定模型路径、类型、配置文件、适配器） |
| `SwitchRlModel` | 切换到已加载的模型 |
| `ListRlModels` | 查询当前平台、默认模型、当前模型、home 模型和已注册 RL 模型信息 |

### 音频控制

| 服务 | 用途 |
|------|------|
| `Speak` | TTS 语音播报 |
| `SetVolume` | 设置系统音量（0.0~100.0） |
| `SetMute` | 设置静音状态 |

### Encos 电机维护（低频管理命令）

| 服务 | 用途 |
|------|------|
| `EncosGetMotorId` | 按 `joint_name` 读取 Encos 电机 ID |
| `EncosGetMotorParam` | 读取电机参数（按 Encos 协议参数码） |
| `EncosSetMotorId` | 修改电机 ID |
| `EncosSetMotorZero` | 设置当前电机位置为零点 |
| `MixQueryJoint` | 按 `joint_name` 查询混装 CANFD 关节状态 |
| `MixSetZero` | 按 `joint_name` 设置混装 CANFD 电机零点 |
| `MixQueryMotorId` | 按 `joint_name` 查询混装 CANFD 电机 ID |
| `MixSetMotorId` | 按 `joint_name` 修改混装 CANFD 电机 ID |

## 构建

```bash
# 此包是纯接口包，使用 ament_cmake + rosidl 构建
colcon build --packages-select robot_interfaces
```

依赖：`builtin_interfaces`、`std_msgs`、`action_msgs`。

## 使用约定

1. **所有外部输入必须转换为 `RobotIntent`**，不得绕过 `robot_router` 直接发送 `JointCommands`。
2. **`JointCommands` 的 `trace_id` 贯穿整个控制管线**（RL → Router → Control → Driver），用于全链路追踪和丢帧检测。
3. **关节名称必须与 `robot_joints.yaml` 中定义的一致**，否则会被路由/控制层丢弃。
4. **Encos 服务是低频管理接口**，仅在维护模式下使用，不得在高频控制循环中调用。
