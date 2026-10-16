# robot_injector

面向 `robot_recorder` 的简单事件注入工具，用于模拟 recorder 关心的两类事件：

- `system_fault`：发布 `/state` 系统故障
- `joint_warning`：发布 `control/hardware_states` 关节告警
- `both`：在同一轮中发布系统故障和关节告警，用于测试同一事件窗口内的多事件记录

## 用法

```bash
ros2 launch robot_injector fault_inject.launch.py mode:=system_fault
ros2 launch robot_injector fault_inject.launch.py mode:=joint_warning joint_name:=waist_yaw
ros2 launch robot_injector fault_inject.launch.py mode:=both order:=warning_first
ros2 launch robot_injector fault_inject.launch.py mode:=both order:=fault_first
```

也可以用脚本：

```bash
ros2 run robot_injector inject_fault.sh both waist_yaw warning_first 3
```

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `mode` | `""` | `system_fault` / `joint_warning` / `both` |
| `order` | `warning_first` | `both` 模式下的发布顺序：`warning_first` / `fault_first` |
| `joint_name` | `waist_yaw` | 关节告警使用的关节名 |
| `motor_error` | `2` | 关节告警使用的底层电机错误码 |
| `delay_sec` | `0.0` | 启动后延迟发布时间 |
| `period_ms` | `100.0` | 重复发布间隔 |
| `count` | `3` | 发布轮数；`<=0` 表示持续发布 |

## 触发 recorder 的内容

`system_fault` 会发布：

```text
/state
driver/status
```

`joint_warning` 会发布：

```text
control/hardware_states
```

在 `event_window` 模式下，`both` 应该让同一个 `event_xxxx/events.csv` 同时出现 `system_fault` 和 `joint_warning` 两行。
