# robot_recorder

旁路式实时数据记录节点。订阅控制管线各阶段话题和 IMU 数据，支持持续落盘和事件窗口两种记录模式，以 CSV 格式保存运行数据。

## 架构

```
高频 ROS 话题 (200 Hz)
  │  JointCommands × 3 + JointStates + NavAll + RobotState
  ↓
ROS 回调线程
  ├── continuous: tryPush → RingBuffer × 4 → 写线程周期 drain → CSV
  └── event_window: push → TimeWindowBuffer × 4，仅故障事件触发后写出窗口数据
```

默认 `event_window` 模式面向日常运行：平时只在内存保留最近 `pre_event_window_sec`
秒高频数据，检测到故障/告警后写出故障前窗口，并继续记录
`post_event_window_sec` 秒。需要专项调试完整正常数据时，可切到 `continuous` 模式。

## 输出文件

`continuous` 模式下，文件位于 `output_dir`（默认 `recordings/`）：

| 文件 | 内容 |
|------|------|
| `joint_states.csv` | 关节状态（位置/速度/力矩/使能/错误码/电机故障/温度）+ IMU（角速度/欧拉角） |
| `desired_commands.csv` | RL 期望命令（来自 `rl_manager/desired_commands`） |
| `accepted_commands.csv` | 路由器准入命令（来自 `router/accepted_commands`） |
| `joint_commands.csv` | 最终硬件命令（来自 `control/hardware_commands`） |
| `events.csv` | 故障/告警事件记录（含触发状态/关节名/错误码/时间戳） |

`event_window` 模式下，每次事件写入独立目录：

```text
<output_dir>/event_0001/
  events.csv
  joint_states.csv
  joint_commands.csv
```

若启用了 desired/accepted 命令记录，同一事件目录下也会生成对应 CSV。

### CSV 列格式

**状态文件**（宽表，~197 列）：`recorder_time_ms, msg_stamp_ms, sequence, last_command_trace_id, last_command_sequence, <各关节>_pos, ..., ---, <各关节>_vel, ..., ---, <各关节>_tor, ..., ---, <各关节>_err, ..., ---, <各关节>_merr, ..., ---, <各关节>_temp, ..., imu_angular_vel_{x,y,z}, imu_euler_{roll,pitch,yaw}`

**命令文件**：`recorder_time_ms, msg_stamp_ms, trace_id, sequence, parent_sequence, source, parent_source, stage, type, <各关节>_pos, ..., ---, <各关节>_vel, ..., ---, <各关节>_tor, ..., ---, <各关节>_kp, ..., ---, <各关节>_kd, ..., ---`

关节名通过 `jointNameAlias()` 映射为简短别名（如 `left_knee → leg_l_4`、`right_shoulder_pitch → arm_r_1`），共 30 个映射。

## ROS 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `buffer_capacity` | int | `2000` | 环形缓冲区容量 |
| `output_dir` | string | `"recordings"` | CSV 输出目录 |
| `robot_joints_config` | string | `""` | 关节配置 YAML 路径，用于固定 CSV 关节列顺序 |
| `flush_interval_ms` | double | `100.0` | 写线程 drain+flush 间隔（ms） |
| `recording_mode` | string | `"event_window"` | 记录模式：`event_window` / `continuous` |
| `pre_event_window_sec` | double | `180.0` | 事件触发前保留并写出的窗口长度 |
| `post_event_window_sec` | double | `60.0` | 事件触发后继续写出的窗口长度 |
| `record_states` | bool | `true` | 记录关节状态 |
| `record_commands` | bool | `true` | 记录硬件命令 |
| `record_desired` | bool | `true` | 记录 RL 期望命令 |
| `record_accepted` | bool | `true` | 记录路由准入命令 |
| `record_imu` | bool | `true` | 记录 IMU 数据 |
| `event_cooldown_sec` | double | `2.0` | 同类型事件冷却时间 |

## 订阅话题

| 话题 | 类型 | QoS | 记录内容 |
|------|------|-----|----------|
| `rl_manager/desired_commands` | `JointCommands` | 高频 | RL 原始输出 |
| `router/accepted_commands` | `JointCommands` | 高频 | 准入后命令 |
| `control/hardware_commands` | `JointCommands` | 高频 | 最终硬件命令 |
| `control/hardware_states` | `JointStates` | 高频 | 关节反馈 |
| `imu/nav_all` | `NavAll` | 可靠(depth=10) | IMU 角速度/欧拉角 |
| `state` | `RobotState` | 可靠(depth=10) | 故障检测 |

## 事件记录

检测两种事件的上升沿（从无到有）。在 `event_window` 模式下，事件触发会创建
一次窗口录制；在 `continuous` 模式下，事件仅追加到 `events.csv`：

1. **系统故障**：`system_state.fault_active` 从 false 变为 true
2. **电机告警**：任意关节的 `joint_error` 变为 `JOINT_ERR_MOTOR_WARN`

事件独立冷却，默认 2 秒内不重复触发。事件完成后立即写入 `events.csv` 并 flush。

## 性能特征

| 特征 | 说明 |
|------|------|
| **回调非阻塞** | `tryPush` 在满时直接丢弃新记录，ROS 回调永不因 I/O 阻塞 |
| **写线程解耦** | 所有 CSV 序列化和文件 I/O 在独立线程，与实时回调隔离 |
| **环形缓冲** | 容量 2000，按 200Hz/100ms 周期计算，提供 ~20x 余量 |
| **CSV 转义** | 含逗号/引号/换行的字段自动加引号转义 |
| **优雅关闭** | 停止写线程 → 清空剩余缓冲区 → 刷新并关闭所有文件 |

## 环境变量

| 变量 | 用途 |
|------|------|
| `ZL_ROBOT_RECORDER_DIR` | 覆盖 `output_dir` 参数 |

## 依赖

| 依赖 | 说明 |
|------|------|
| `rclcpp` | ROS 2 C++ 客户端库 |
| `Eigen3` | IMU 数据矩阵运算 |
| `robot_common` | 高频 QoS 配置 |
| `robot_interfaces` | 关节命令/状态消息 |
| `yesense_interface` | NavAll IMU 消息 |
