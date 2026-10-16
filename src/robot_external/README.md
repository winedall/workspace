# robot_external

外部控制接口节点。接收头部控制角度指令（度），转换为弧度后通过线性插补平滑输出关节命令。

## 管线

```
HeadControl (度, 相对/绝对)
  → math.radians() 转换
  → 相对模式：当前实际位置 + 增量
  → 钳位到 [-3.14, 3.14] rad
  → 线性插补 (默认 100 步 @ 200 Hz = 0.5s)
  → JointCommands (rad) 发布到 external/desired_commands
```

## 节点

**名称**：`robot_external_node`（Python）

## ROS 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `interpolation_steps` | int | `100` | 插补步数（≥1，`max(1, value)` 保护） |

## 话题

| 类型 | 话题 | 说明 |
|------|------|------|
| **Sub** | `external/head_control` | 头部角度指令 `HeadControl`（度） |
| **Sub** | `control/hardware_states` | 仅缓存头部关节实际位置（用于相对运动和插补起点） |
| **Pub** | `external/desired_commands` | 插补后的头部关节命令 `JointCommands`（rad，200 Hz） |

## HeadControl 消息

| 字段 | 类型 | 说明 |
|------|------|------|
| `yaw` | `float32` | 目标偏航角（度，-180~180） |
| `pitch` | `float32` | 目标俯仰角（度，-180~180） |
| `is_relative` | `bool` | `true`=相对角度（当前位置 + 增量），`false`=绝对角度 |

## 输出 JointCommands

每条 `JointCommands` 包含两个 `JointCommand`：
- `head_yaw`：kp=20.0, kd=2.0, enable=true
- `head_pitch`：kp=20.0, kd=2.0, enable=true

帧元数据：`source="external"`, `stage="desired"`。

## 插补行为

- 定时器默认取消，收到 `HeadControl` 后 `reset()` 启动
- 每个新 `HeadControl` 指令重置插补计数器到 `interpolation_steps`
- 起始位置快照为当前缓存的头部关节实际位置
- 线性插值（lerp）到目标位置
- 完成最后一帧后 `cancel()` 定时器，停止发布

## 关节状态缓存

`_handle_hardware_states` 回调仅缓存 `head_yaw` 和 `head_pitch` 两个关节的位置值，不再遍历全量关节。

## 依赖

| 依赖 | 说明 |
|------|------|
| `rclpy` | ROS 2 Python 客户端库 |
| `robot_interfaces` | `HeadControl`、`JointCommand`、`JointCommands`、`JointStates` |
| `math` | 标准库，角度转换 |
