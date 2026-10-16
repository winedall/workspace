# RobotIntent 发布命令速查

所有命令发布到 topic `input/intent`，消息类型 `robot_interfaces/msg/RobotIntent`。

查看消息定义：

```bash
ros2 interface show robot_interfaces/msg/RobotIntent
```

查看实时消息：

```bash
ros2 topic echo /robot/input/intent
```

---

## status（状态切换）

| command | 含义 |
|---------|------|
| `ready` | 进入准备流程 |
| `walk` | 从 READY 进入行走控制状态 |
| `damp` | 进入阻尼状态 |
| `reinit` | 从 DAMP 回到 IDLE |

```bash
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'status', command: 'ready', priority: 0}" --once

ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'status', command: 'walk', priority: 0}" --once

ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'status', command: 'damp', priority: 0}" --once

ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'status', command: 'reinit', priority: 0}" --once
```

---

## model（切换动作模型）

| command | action | 含义 |
|---------|--------|------|
| `switch_model` | `t4_walk` | 行走模型 |
| `switch_model` | `t4_heart` | 比心 |
| `switch_model` | `t4_taichi` / `t4_taichi_w` | 动太极 / 静太极 |
| `switch_model` | `t4_squat_down` / `t4_squat_up` | 下蹲 / 蹲起 |
| `switch_model` | `t4_hello` | 打招呼 |
| `switch_model` | `t4_mj_dance` / `t4_xqxa123_dance` | 滑步舞蹈 / 相亲相爱舞蹈 |
| `finish_loop` | (空) | 优雅退出当前循环动作 |
| `motion_stop` | `stop` / `resume` | 停止/恢复当前模型运动 |

```bash
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'model', command: 'switch_model', action: 't4_walk', priority: 0}" --once

ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'model', command: 'switch_model', action: 't4_heart', priority: 0}" --once

ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'model', command: 'switch_model', action: 't4_taichi', priority: 0}" --once

# 优雅退出循环动作
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'model', command: 'finish_loop', priority: 0}" --once

# 运动停止
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'model', command: 'motion_stop', action: 'stop', priority: 0}" --once

# 运动恢复
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'model', command: 'motion_stop', action: 'resume', priority: 0}" --once

```

---

## motion（连续运动）

`command` 固定为 `walk`，通过三个速度字段控制。停止也用相同命令，速度全置 0 即可。**建议用 `--rate 10` 持续发布**，停止发布后机器人因 `joy_timeout_sec` 参数过期自动停止。

```bash
# 向前 0.4 m/s
ros2 topic pub --rate 10 /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'motion', command: 'walk', linear_x_mps: 0.4, linear_y_mps: 0.0, angular_z_radps: 0.0, priority: 0}"

# 停止（速度全零）
ros2 topic pub --rate 10 /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'motion', command: 'walk', linear_x_mps: 0.0, linear_y_mps: 0.0, angular_z_radps: 0.0, priority: 0}"

# 右转 + 前进
ros2 topic pub --rate 10 /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'motion', command: 'walk', linear_x_mps: 0.3, linear_y_mps: 0.0, angular_z_radps: -0.5, priority: 0}"

# 横向平移
ros2 topic pub --rate 10 /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'motion', command: 'walk', linear_x_mps: 0.0, linear_y_mps: 0.2, angular_z_radps: 0.0, priority: 0}"
```
