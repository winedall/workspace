# zl_robot 命令速查手册

> 适用于 ROS 2 Humble，所有命令需在 `source /opt/ros/humble/setup.bash && source install/setup.bash` 后的终端中执行。

---

## 1. RobotIntent 发布命令

所有意图发布到 topic **`input/intent`**，消息类型 **`robot_interfaces/msg/RobotIntent`**。

```bash
# 查看消息定义
ros2 interface show robot_interfaces/msg/RobotIntent

# 实时监听
ros2 topic echo /robot/input/intent
```

### 1.1 status（状态切换）

| command | 含义 | 前置状态 | 目标状态 |
|---------|------|---------|---------|
| `ready` | 进入准备流程 | IDLE | DO_READY → READY |
| `walk` | 进入行走控制 | READY | RL_STANDBY → RL_ACTIVE |
| `damp` | 进入阻尼状态 | 任意 | DAMP |
| `reinit` | 回到空闲态 | DAMP | IDLE |

```bash
# 准备状态
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'status', command: 'ready', priority: 0}" --once

# 行走状态
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'status', command: 'walk', priority: 0}" --once

# 阻尼状态
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'status', command: 'damp', priority: 0}" --once

# 待机状态
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'status', command: 'reinit', priority: 0}" --once
```

### 1.2 model（切换动作模型）

| command | action | 含义 |
|---------|--------|------|
| `switch_model` | `t4_walk` | 行走（默认模型） |
| `switch_model` | `t4_heart` | 比心 |
| `switch_model` | `t4_taichi` / `t4_taichi_w` | 动太极 / 静太极 |
| `switch_model` | `t4_squat_down` / `t4_squat_up` | 下蹲 / 蹲起 |
| `switch_model` | `t4_hello` | 打招呼 |
| `switch_model` | `t4_mj_dance` / `t4_xqxa123_dance` | 滑步舞蹈 / 相亲相爱舞蹈 |
| `switch_model` | `t4_side_flip` / `t4_long_jump` | 侧空翻 / 跳远（特技模型，遥控器 spec 模式映射） |
| `finish_loop` | (空) | 优雅退出当前循环动作，播完当前周期后回退到 t4_walk |
| `motion_stop` | `stop` | 停止当前模型运动，强制切回 t4_walk 零速站立 |
| `motion_stop` | `resume` | 恢复当前模型运动 |

```bash
# 切换模型 - 行走
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'model', command: 'switch_model', action: 't4_walk', priority: 0}" --once

# 切换模型 - 比心
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'model', command: 'switch_model', action: 't4_heart', priority: 0}" --once

# 切换模型 - 太极
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

### 1.3 motion（连续运动）

`command` 固定为 `walk`，通过三个速度字段控制。停止也用相同命令，速度全置 0。
**建议用 `--rate 10` 持续发布**，停止发布后机器人因 `joy_timeout_sec` 参数过期自动停止。

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

### 1.4 config（配置切换）

`intent_type` 固定为 `config`，通过 `command` + `action` 组合切换机器人运行配置。

| command | action | 含义 |
|---------|--------|------|
| `getup` | `getup` / `stand` / `lie` / `face` | 起身姿态选择 |
| `nav` | `nav` / `auto` / `manual` / `follow` | 导航模式切换 |
| `speed` | `speed` / `std` / `high` | 速度模式切换 |
| `mode` | `mode` / `rl` / `mix` / `spec` | 控制模式切换（spec=特技模式，仅影响遥控器按键映射） |

```bash
# 起身姿态选择
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'config', command: 'getup', action: 'getup', priority: 0}" --once
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'config', command: 'getup', action: 'stand', priority: 0}" --once
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'config', command: 'getup', action: 'lie', priority: 0}" --once
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'config', command: 'getup', action: 'face', priority: 0}" --once

# 导航模式切换
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'config', command: 'nav', action: 'nav', priority: 0}" --once
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'config', command: 'nav', action: 'auto', priority: 0}" --once
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'config', command: 'nav', action: 'manual', priority: 0}" --once
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'config', command: 'nav', action: 'follow', priority: 0}" --once

# 速度模式切换
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'config', command: 'speed', action: 'speed', priority: 0}" --once
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'config', command: 'speed', action: 'std', priority: 0}" --once
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'config', command: 'speed', action: 'high', priority: 0}" --once

# 控制模式切换
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'config', command: 'mode', action: 'rl', priority: 0}" --once
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'config', command: 'mode', action: 'mix', priority: 0}" --once
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'config', command: 'mode', action: 'spec', priority: 0}" --once
```

### 1.5 头部控制（外部接口）

通过 `external/head_control` 发送头部控制指令，消息类型 `HeadControl`。支持绝对角度和相对角度两种模式，角度值以 **度** 为单位。

```bash
# 查看消息定义
ros2 interface show robot_interfaces/msg/HeadControl

# 绝对角度：头部转到 yaw=30°, pitch=15°
ros2 topic pub --once /robot/external/head_control robot_interfaces/msg/HeadControl \
  "{yaw: 30.0, pitch: 15.0, is_relative: false}"

# 相对角度：在当前基础上右转 10°，抬头 5°
ros2 topic pub --once /robot/external/head_control robot_interfaces/msg/HeadControl \
  "{yaw: 10.0, pitch: 5.0, is_relative: true}"

# 回正：yaw=0°, pitch=0°
ros2 topic pub --once /robot/external/head_control robot_interfaces/msg/HeadControl \
  "{yaw: 0.0, pitch: 0.0, is_relative: false}"
```

系统会自动将角度转换为弧度，限幅到关节活动范围，并做 50 帧平滑插值。

### 1.6 reset（故障复位）

| intent_type | command | 含义 |
|-------------|---------|------|
| `reset` | `reset_fault` | 故障复位 |

```bash
# 故障复位
ros2 topic pub /robot/input/intent robot_interfaces/msg/RobotIntent \
  "{source: 'teleop', intent_type: 'reset', command: 'reset_fault', priority: 0}" --once
```

---

## 2. MQTT 桥接测试命令

MQTT 桥接节点默认不启动，需要通过 `enable_mqtt:=true` 打开。默认 broker 和 topic 映射来自 `robot_bringup/config/mqtt_params.yaml`。

```bash
# 启动带 MQTT 桥接的机器人系统
ros2 launch robot_bringup zl_robot.launch.py enable_mqtt:=true

# 当前默认 MQTT broker
export MQTT_HOST=127.0.0.1
export MQTT_PORT=1883
```

### 2.1 MQTT topic 映射

| 方向 | MQTT topic | ROS topic | ROS 类型 |
|------|------------|-----------|----------|
| MQTT → ROS | `/mqtt/robot/intent` | `input/intent` | `robot_interfaces/msg/RobotIntent` |
| ROS → MQTT | `/mqtt/robot/monitor/system_status` | `monitor/system_status` | `robot_interfaces/msg/SystemStatus` |
| ROS → MQTT | `/mqtt/robot/state` | `state` | `robot_interfaces/msg/RobotState` |
| ROS → MQTT | `/mqtt/robot/driver/status` | `driver/status` | `robot_interfaces/msg/DriverStatus` |

### 2.2 订阅机器人状态

```bash
# 订阅所有 robot_mqtt 上报 topic
mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" -t "/mqtt/robot/#" -v

# 只看状态机状态
mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" -t "/mqtt/robot/state" -v

# 只看系统监控状态
mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" -t "/mqtt/robot/monitor/system_status" -v

# 只看驱动状态
mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" -t "/mqtt/robot/driver/status" -v
```

### 2.3 通过 MQTT 发布 RobotIntent

MQTT 下发的 payload 是 JSON，对应 `robot_interfaces/msg/RobotIntent` 字段。建议 MQTT/API/云端来源使用 `priority: 50~99`，避免抢占遥控器 `priority: 0`。

```bash
# 准备状态
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -q 1 -t "/mqtt/robot/intent" \
  -m '{"source":"mqtt","intent_type":"status","command":"ready","priority":50}'

# 行走状态
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -q 1 -t "/mqtt/robot/intent" \
  -m '{"source":"mqtt","intent_type":"status","command":"walk","priority":50}'

# 阻尼状态
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -q 1 -t "/mqtt/robot/intent" \
  -m '{"source":"mqtt","intent_type":"status","command":"damp","priority":50}'

# 故障复位
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -q 1 -t "/mqtt/robot/intent" \
  -m '{"source":"mqtt","intent_type":"reset","command":"reset_fault","priority":50}'

# 切换模型
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -q 1 -t "/mqtt/robot/intent" \
  -m '{"source":"mqtt","intent_type":"model","command":"switch_model","action":"t4_heart","priority":50}'

# 发布一次前进速度
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -q 1 -t "/mqtt/robot/intent" \
  -m '{"source":"mqtt","intent_type":"motion","command":"walk","linear_x_mps":0.3,"linear_y_mps":0.0,"angular_z_radps":0.0,"priority":50}'

# 持续发布前进速度，Ctrl-C 停止
while true; do
  mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -q 1 -t "/mqtt/robot/intent" \
    -m '{"source":"mqtt","intent_type":"motion","command":"walk","linear_x_mps":0.3,"linear_y_mps":0.0,"angular_z_radps":0.0,"priority":50}'
  sleep 0.1
done

# 停止运动
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -q 1 -t "/mqtt/robot/intent" \
  -m '{"source":"mqtt","intent_type":"motion","command":"walk","linear_x_mps":0.0,"linear_y_mps":0.0,"angular_z_radps":0.0,"priority":50}'
```

### 2.4 MQTT 与 ROS 联调验证

```bash
# 终端 A：观察 MQTT 下发是否进入 ROS RobotIntent
ros2 topic echo /robot/input/intent

# 终端 B：通过 MQTT 发布一条准备指令
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -q 1 -t "/mqtt/robot/intent" \
  -m '{"source":"mqtt","intent_type":"status","command":"ready","priority":50}'

# 终端 C：观察 ROS 状态是否被 robot_mqtt 上报到 MQTT
mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" -t "/mqtt/robot/state" -v
```

### 2.5 断线恢复观察

`robot_mqtt` 使用异步连接和 1~30 秒退避重连。网络或 broker 恢复后，节点会自动重连，并重新订阅 `/mqtt/robot/intent`。断线期间 ROS → MQTT 的周期状态会上报失败并被丢弃，不阻塞 ROS 回调。

```bash
# 查看 robot_mqtt 节点是否在线
ros2 node list | grep robot_mqtt

# 查看 robot_mqtt 日志中是否出现断线/重连记录
ros2 launch robot_bringup zl_robot.launch.py enable_mqtt:=true

# broker 恢复后，重新订阅状态确认数据恢复
mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" -t "/mqtt/robot/#" -v
```

---

## 3. Service 调用命令

### 3.1 状态控制服务（robot_state_node）

```bash
# 故障复位
ros2 service call /robot/state/reset_fault robot_interfaces/srv/ResetFault \
  "{reason: 'fault_cleared'}"
```

### 3.2 RL 模型服务（robot_rl_manager_node）

```bash
# 查询当前平台已注册模型
ros2 service call /robot/rl_manager/list_models robot_interfaces/srv/ListRlModels "{}"

# 加载模型（路径相对于 robot_rl_manager 的 share 目录）
ros2 service call /robot/rl_manager/load_model robot_interfaces/srv/LoadRlModel \
  "{model_name: 't4_walk', model_path: 'model/t4_walk.onnx', model_type: 'onnx', config_path: 'config/t4_walk.yaml', adapter_path: 'policy/adapters/t4_walk.py'}"

# 切换模型（需先行加载或已在 config/humanoid/rl_models.yaml 注册）
ros2 service call /robot/rl_manager/switch_model robot_interfaces/srv/SwitchRlModel \
  "{model_name: 't4_heart'}"
```

`list_models` 返回当前 `runtime_platform`（如 `x86_64` / `aarch64`）、`default_model_name`、`active_model_name`、`home_model_name`，以及当前平台实际注册成功的模型列表。每个模型包含 `name`、`alias`、`category`、`backend`、`duration_sec`、`has_duration`。

### 3.3 电机维护服务（robot_driver_node）——配置模式专用

通过 `driver/tienkung/*`、`driver/encos/*` 和 `driver/mix/*` admin service 实现电机低频维护命令。关节级配置命令统一使用 `joint_name` 定位关节，底层 `slave_id`、`passage`、`canfd.bus` 等拓扑字段只由硬件 YAML 配置解析。

> **模式说明**：
> - **config_mode=true**（zl_robot_driver.launch.py）：只启动 admin srv，不启动 worker 线程， `control/hardware_commands` 失效
> - **config_mode=false**（zl_robot.launch.py）：正常运行，worker 线程处理力位混控，admin srv 失效或返回 "not in config mode"

> **配置模式专用 srv（通过 zl_robot_driver.launch.py 启动）**：
>
> | service | request | response |
> |---------|---------|----------|
> | `driver/tienkung/query_status` | `joint_name`: string, `timeout_sec`: float32 | `success`, `message`, `motor_id`, `feedback_code`, `position`, `velocity`, `effort`, `motor_error`, `motor_temp`, `mos_temp` |
> | `driver/tienkung/set_zero` | `joint_name`: string, `timeout_sec`: float32 | `success`, `message`, `motor_id`, `feedback_code`, `zero_position_rad` |
> | `driver/tienkung/modify_id` | `joint_name`: string, `new_motor_id`: uint8, `timeout_sec`: float32 | `success`, `message`, `motor_id`, `new_motor_id`, `feedback_code` |
> | `driver/tienkung/query_id` | `timeout_sec`: float32 | `success`, `message`, `motor_id`, `feedback_code` |
> | `driver/tienkung/reset_id` | `timeout_sec`: float32 | `success`, `message`, `feedback_code` |
>
> | service | request | response |
> |---------|---------|----------|
> | `driver/encos/get_motor_id` | `joint_name`: string, `timeout_sec`: float32 | `success`, `message`, `motor_id`, `feedback_code` |
> | `driver/encos/set_motor_id` | `joint_name`: string, `new_motor_id`: uint16, `timeout_sec`: float32 | `success`, `message`, `motor_id`, `new_motor_id`, `feedback_code` |
> | `driver/encos/set_motor_zero` | `joint_name`: string, `timeout_sec`: float32 | `success`, `message`, `motor_id`, `feedback_code` |
> | `driver/encos/get_motor_param` | `joint_name`: string, `param_code`: uint8, `timeout_sec`: float32 | `success`, `message`, `motor_id`, `param_code`, `value`, `value_min`, `value_max` |
>
> | service | request | response |
> |---------|---------|----------|
> | `driver/mix/query_joint` | `joint_name`: string, `timeout_sec`: float32 | `success`, `message`, `vendor`, `motor_id`, `feedback_code`, `position`, `velocity`, `effort`, `motor_error`, `motor_temp`, `mos_temp` |
> | `driver/mix/set_zero` | `joint_name`: string, `timeout_sec`: float32 | `success`, `message`, `vendor`, `motor_id`, `feedback_code`, `zero_position_rad` |
> | `driver/mix/query_motor_id` | `joint_name`: string, `timeout_sec`: float32 | `success`, `message`, `vendor`, `motor_id`, `feedback_code` |
> | `driver/mix/set_motor_id` | `joint_name`: string, `new_motor_id`: uint16, `timeout_sec`: float32 | `success`, `message`, `vendor`, `motor_id`, `new_motor_id`, `feedback_code` |

> `driver/mix/query_motor_id` 中的 `joint_name` 用于从 `hw_mix.yaml` 定位厂家协议和 `canfd.bus`；底层查询 ID 指令是 bus 级事务，不依赖配置中的目标 `motor_id` 做响应匹配。`driver/mix/set_motor_id` 会使用 `hw_mix.yaml` 中该关节当前配置的 `motor_id` 发送修改命令，修改成功后需要同步更新 `hw_mix.yaml`，否则下次启动或后续配置命令仍会按旧 ID 通讯。
>
> `driver/mix/set_motor_id` 会按 `vendor` 自动选择厂家协议。`vendor=tienkung` 时 `new_motor_id` 实际范围为 `1~255`；`vendor=encos` 时使用 `uint16` ID。修改 ID 前建议确保同一路 `canfd.bus` 上只有目标电机响应，避免广播/配置维护过程中误改其他电机。
>
>
> **config_mode 标志状态查询**（通过 driver/status 查看）：
>
> ```bash
> # 查看驱动状态，其中 config_mode 字段指示当前模式
> ros2 topic echo /robot/driver/status
> # 关键输出示例：
> # ...
> # config_mode: true      # 当前为配置模式
> # config_mode: false     # 当前为运行模式
> # status_message: "encos config mode ready"
> # 或
> # status_message: "tienkung motion write ready"
> ```

### 3.4 新增：电机配置 srv 速查表

| service file | 服务名称 | 典型用途 |
|-------------|----------|----------|
| `TienkungQueryStatus.srv` | `driver/tienkung/query_status` | 查询指定关节电机状态（位置、速度、扭矩） |
| `TienkungSetZero.srv` | `driver/tienkung/set_zero` | 将关节当前位置设为电机内部零点 |
| `TienkungModifyID.srv` | `driver/tienkung/modify_id` | 修改指定关节的电机 ID |
| `TienkungQueryID.srv` | `driver/tienkung/query_id` | **广播**查询所有电机 ID |
| `TienkungResetID.srv` | `driver/tienkung/reset_id` | **广播**重置所有电机 ID 为出厂默认值 |
| `EncosGetMotorId.srv` | `driver/encos/get_motor_id` | 按 `joint_name` 读取 Encos 电机 ID |
| `EncosSetMotorId.srv` | `driver/encos/set_motor_id` | 按 `joint_name` 修改 Encos 电机 ID |
| `EncosSetMotorZero.srv` | `driver/encos/set_motor_zero` | 按 `joint_name` 设置 Encos 电机零位 |
| `EncosGetMotorParam.srv` | `driver/encos/get_motor_param` | 按 `joint_name` 读取 Encos 厂家参数 |
| `MixQueryJoint.srv` | `driver/mix/query_joint` | 按 `joint_name` 查询混装 CANFD 关节状态 |
| `MixSetZero.srv` | `driver/mix/set_zero` | 按 `joint_name` 设置混装 CANFD 电机零位 |
| `MixQueryMotorId.srv` | `driver/mix/query_motor_id` | 按 `joint_name` 查询混装 CANFD 电机 ID |
| `MixSetMotorId.srv` | `driver/mix/set_motor_id` | 按 `joint_name` 修改混装 CANFD 电机 ID |

### 3.5 Service 调用范例（以 config_mode=true 为例）

```bash
# 1. 查询天工关节 'left_knee' 的电机状态
ros2 service call /robot/driver/tienkung/query_status \
  robot_interfaces/srv/TienkungQueryStatus \
  "{joint_name: 'left_knee', timeout_sec: 2.0}"

# 2. 设置天工关节 'left_knee' 零点
ros2 service call /robot/driver/tienkung/set_zero \
  robot_interfaces/srv/TienkungSetZero \
  "{joint_name: 'left_knee', timeout_sec: 3.0}"

# 3. 修改天工关节 'left_knee' 电机 ID 为 5
ros2 service call /robot/driver/tienkung/modify_id \
  robot_interfaces/srv/TienkungModifyID \
  "{joint_name: 'left_knee', new_motor_id: 5, timeout_sec: 2.0}"

# 4. 广播查询所有电机 ID（无需指定关节名）
ros2 service call /robot/driver/tienkung/query_id \
  robot_interfaces/srv/TienkungQueryID \
  "{timeout_sec: 1.0}"

# 5. 广播重置所有电机 ID
ros2 service call /robot/driver/tienkung/reset_id \
  robot_interfaces/srv/TienkungResetID \
  "{timeout_sec: 2.0}"

# 6. 读取 Encos 关节 'left_wrist_pitch' 的电机 ID
ros2 service call /robot/driver/encos/get_motor_id \
  robot_interfaces/srv/EncosGetMotorId \
  "{joint_name: 'left_wrist_pitch', timeout_sec: 1.0}"

# 7. 设置 Encos 关节 'left_wrist_pitch' 的电机 ID 为 3
ros2 service call /robot/driver/encos/set_motor_id \
  robot_interfaces/srv/EncosSetMotorId \
  "{joint_name: 'left_wrist_pitch', new_motor_id: 3, timeout_sec: 1.0}"

# 8. 设置 Encos 关节 'left_wrist_pitch' 的电机零点
ros2 service call /robot/driver/encos/set_motor_zero \
  robot_interfaces/srv/EncosSetMotorZero \
  "{joint_name: 'left_wrist_pitch', timeout_sec: 1.0}"

# 9. 读取 Encos 关节 'left_wrist_pitch' 的厂家参数
ros2 service call /robot/driver/encos/get_motor_param \
  robot_interfaces/srv/EncosGetMotorParam \
  "{joint_name: 'left_wrist_pitch', param_code: 1, timeout_sec: 1.0}"

# 10. 查询 mix 混装 CANFD 关节状态
ros2 service call /robot/driver/mix/query_joint \
  robot_interfaces/srv/MixQueryJoint \
  "{joint_name: 'left_wrist_pitch', timeout_sec: 2.0}"

# 11. 设置 mix 混装 CANFD 关节零点
ros2 service call /robot/driver/mix/set_zero \
  robot_interfaces/srv/MixSetZero \
  "{joint_name: 'left_wrist_pitch', timeout_sec: 3.0}"

# 12. 查询 mix 混装 CANFD 关节对应电机 ID
# joint_name 用于定位 hw_mix.yaml 中的 vendor 和 canfd.bus
ros2 service call /robot/driver/mix/query_motor_id \
  robot_interfaces/srv/MixQueryMotorId \
  "{joint_name: 'left_wrist_pitch', timeout_sec: 2.0}"

# 13. 修改 mix 混装 CANFD 关节电机 ID
# 会按 joint_name 对应的 vendor 自动选择 Encos 或 Tienkung 协议
# 修改成功后需要同步更新 config/hardware/hw_mix.yaml 中该关节的 motor_id
ros2 service call /robot/driver/mix/set_motor_id \
  robot_interfaces/srv/MixSetMotorId \
  "{joint_name: 'left_wrist_pitch', new_motor_id: 16, timeout_sec: 2.0}"
```

### 3.6 Mode Switching in Launch

| launch 文件 | config_mode | 适用场景 |
|------------|------------|----------|
| `zl_robot.launch.py` | `false`（默认） | 正常运行：启动 worker 线程、力位混控、所有运动控制生效 |
| `zl_robot_driver.launch.py` | `true` | 配置模式：仅启动 admin srv、禁止运动命令、适用于电机配置、故障诊断、参数录入 |

```bash
# 启动正常运行模式
ros2 launch robot_bringup zl_robot.launch.py

# 启动配置模式（仅用于电机参数配置、故障排查）
ros2 launch robot_bringup zl_robot_driver.launch.py
```


| launch 文件 | config_mode | 适用场景 |
|------------|------------|----------|
| `zl_robot.launch.py` | `false`（默认） | 正常运行：启动 worker 线程、力位混控、所有运动控制生效 |
| `zl_robot_driver.launch.py` | `true` | 配置模式：仅启动 admin srv、禁止运动命令、适用于电机配置、故障诊断、参数录入 |

```bash
# 启动正常运行模式
ros2 launch robot_bringup zl_robot.launch.py

# 启动配置模式（仅用于电机参数配置、故障排查）
ros2 launch robot_bringup zl_robot_driver.launch.py
```

### 3.7 交互式电机配置工具 (motor.sh)

`motor.sh` 是基于 `zl_robot_driver` config_mode 的交互式电机配置脚本，支持 `tienkung`、`encos` 和 `mix` 三种硬件配置，**运行时直接解析 YAML 配置**，无需构建步骤，单脚本即可运行。

> **前置条件**：需安装 `python3-yaml` (`sudo apt install python3-yaml`)，脚本会自动后台启动 config_mode driver 并等待服务就绪。

#### 3.7.1 依赖安装

```bash
# 安装 Python YAML 解析库
sudo apt install python3-yaml

# 确保已 source ROS 2 环境
source /opt/ros/humble/setup.bash
source install/setup.bash
```

#### 3.7.2 使用方法

```bash
# 仅 Tienkung 关节
./scripts/motor.sh --vendor tienkung

# 仅 Encos 关节
./scripts/motor.sh --vendor encos

# 混装 CANFD 关节（解析 config/<type>/hw_mix.yaml）
./scripts/motor.sh --vendor mix

# 指定机器人类型和配置路径
./scripts/motor.sh --type humanoid --vendor mix --config /path/to/config
```

#### 3.7.3 菜单功能速查

| 选项 | 功能 | 说明 |
|------|------|------|
| 1 | 📍 读取关节位置 | 选择关节，自动调用对应厂商 srv 查询位置/速度/力矩 |
| 2 | 🔧 关节标零 | 选择关节，显示标零前位置 → 执行标零 → 显示标零后位置 |
| 3 | 📋 批量读取所有位置 | 顺序读取所有关节位置，适合快速巡检 |
| 4 | 🔄 批量标零 | 需输入 YES 确认，对所有关节顺序执行标零 |
| 5 | 🆔 读取电机 ID | Tienkung: 广播查询；Encos/Mix: 按关节名查询 |

#### 3.7.4 交互示例

```bash
$ ./scripts/motor.sh --vendor mix

=============================================
        zl_robot 电机配置工具
        Robot: humanoid | Vendor: MIX
=============================================

  1. 📍 读取关节位置
  2. 🔧 关节标零
  3. 🆔 读取电机 ID
  4. 📋 批量读取所有关节位置
  5. 🔄 批量标零 (需确认)
  q. 退出程序
=============================================
请选择功能: 1

=============================================
        关节选择
=============================================
   1. head_yaw           (TIENKUNG slave=1 bus=1 slot=0)
   2. head_pitch         (TIENKUNG slave=1 bus=1 slot=1)
   3. left_shoulder_pitch (TIENKUNG slave=1 bus=0 slot=0)
   ...
  31. right_ankle_roll   (TIENKUNG canfd.bus=3 slave=1 bus=0 slot=5)
=============================================
输入关节号: 4

=============================================
       读取 left_shoulder_roll (TIENKUNG) 位置
=============================================
[服务调用输出]
...
按回车继续...
```

#### 3.7.5 配置文件要求

脚本运行时直接解析以下配置文件（需放在 `$ZL_ROBOT_CONFIG_ROOT/<robot_type>/`）：

| 文件 | 用途 |
|------|------|
| `robot_joints.yaml` | 关节名称、分组、单位、限制 |
| `hw_tienkung.yaml` | Tienkung 关节硬件映射 (slave_id, bus, slot, motor_id, kt 等) |
| `hw_encos.yaml` | Encos 关节硬件映射 (slave_id, passage, motor_id, kt 等) |
| `hw_mix.yaml` | Encos/Tienkung 混装 CANFD 映射；`canfd.bus + motor_id` 决定通讯通道，`vendor` 决定厂家协议 |

> **注意**：脚本运行时直接读取 YAML，配置修改即时生效，无需重新构建。脚本会自动匹配 `robot_joints.yaml` 中的关节名与硬件配置中的映射，未在硬件配置中定义的关节会被跳过。`--vendor mix` 只读取 `hw_mix.yaml`，所有维护命令走 `/robot/driver/mix/*`。

---

### 3.7b 单电机运动调试工具 (motor_motion.sh)

`motor_motion.sh` + `motor_motion.py` 是 `motor.sh` 的运动补充版：以**运控模式** (`config_mode=false`) 单独拉起 `robot_driver`，按编号选择一个关节，直接向 `/robot/control/hardware_commands` 下发 `JointCommands` 全量帧，绕过 router/control 对单个电机做台架运动测试。

> **⚠ 安全警告**
> - 同一时刻**只使能一个关节**，其余关节每帧显式 `enable=false` 断使能 —— **站立的机器人会立即倾倒**，必须吊机/台架固定、脚不承重。
> - 目标位置按限速 (默认 0.3 rad/s) 平滑爬升，避免命令阶跃被 kp 放大成冲击；默认 kp=30/kd=1，不建议超过 50。
> - 退出 (q / Ctrl+C / EOF) 前会自动发布全关帧；driver 缓存的最后一帧是全 disable，不会残留运动命令。
> - 需要 `robot_driver` 二进制已 setcap (`sudo setcap cap_sys_nice=ep install/robot_driver/lib/robot_driver/robot_driver_node`)，脚本会检测并告警。

#### 使用方法

```bash
# tienkung 真机单电机调试（自动拉起运控模式 driver，交互前需吊机确认）
./scripts/motor_motion.sh

# 指定硬件配置 / 机器人类型 / 命令帧频率
./scripts/motor_motion.sh --profile encos --rate 100

# 复用已运行的 robot_driver，不自行拉起
./scripts/motor_motion.sh --no-driver

# 无硬件走一遍交互菜单
./scripts/motor_motion.sh --dry-run
```

#### 子菜单命令速查

| 命令 | 功能 |
|------|------|
| `<编号>` | 选择关节（自动断开上一个使能中的关节），编号与 motor.sh 一致，来自 robot_joints_*.yaml 顺序 |
| `e` | 使能选中关节并保持在当前反馈位置（无反馈时拒绝使能） |
| `g <rad>` | 绝对位置运动，自动限幅到 min_pos/max_pos，未使能时先自动使能 |
| `r <rad>` | 相对位置运动 |
| `s <rad/s>` | 设置运动速度（上限受关节 max_vel 限制） |
| `p <kp> <kd>` | 设置 PD 增益 |
| `d` | 断使能选中关节 |
| `c` / `l` | 查看反馈 / 关节列表 |
| `q` | 退出（先发全关帧） |

> **实现要点**：环境与 run.sh 对齐（ROS_DOMAIN_ID=10、CycloneDDS localhost、`/robot` namespace）；QoS 与 driver 的 SensorDataQoS 匹配 (best_effort/keep_last=2)；位置命令使用逻辑关节空间 rad，单位换算/方向/零位由 robot_driver 后端处理。首次收到命令时 tienkung 后端会对 enable_default=true 的关节做一次使能握手再按帧清槽，属预期行为。

---

### 3.8 音频播报（robot_audio_node）

`robot_audio_node` 通过 topic **`audio/request`** 接收统一音频请求，消息类型 **`robot_interfaces/msg/AudioRequest`**。

| kind | 含义 | name 字段 |
|------|------|-----------|
| `1` / `KIND_TIP` | 系统提示文本 | `config/data/tips.yaml` 中的提示文本 key |
| `2` / `KIND_FILE` | 动作音频文件 | `config/data/actions` 下的文件名，或绝对路径 |
| `3` / `KIND_TEXT` | 文本 TTS | 文本内容；需 `tts_enabled=true`，默认开启 |

| 字段 | 说明 |
|------|------|
| `kind` | 请求类型，只支持 `KIND_TIP=1`、`KIND_FILE=2`、`KIND_TEXT=3`；其他值会被忽略并打印告警 |
| `name` | 请求内容；节点会先去掉首尾空白，空字符串直接忽略 |
| `priority` | 播放优先级，数值越大越先播放；队列满时新请求会被拒绝 |
| `interrupt` | 是否允许中断当前播放；`interrupt=true` 或 `priority>=200` 都会触发中断逻辑 |

```bash
# 查看消息定义
ros2 interface show robot_interfaces/msg/AudioRequest

# 播放系统提示音：待机模式
ros2 topic pub /robot/audio/request robot_interfaces/msg/AudioRequest \
  "{kind: 1, name: 'state_idle', priority: 100, interrupt: false}" --once

# 播放系统提示音：系统发生故障，高优先级会中断当前播放
ros2 topic pub /robot/audio/request robot_interfaces/msg/AudioRequest \
  "{kind: 1, name: 'state_fault', priority: 200, interrupt: true}" --once

# 播放动作音频文件；相对文件名由 robot_audio 在 config/data/actions 下查找
ros2 topic pub /robot/audio/request robot_interfaces/msg/AudioRequest \
  "{kind: 2, name: '我在这_16k.wav', priority: 120, interrupt: false}" --once

# 临时文本 TTS 播报；系统固定播报建议优先使用 KIND_TIP
ros2 topic pub /robot/audio/request robot_interfaces/msg/AudioRequest \
  "{kind: 3, name: '请注意，机器人即将进入调试模式', priority: 100, interrupt: false}" --once
```

系统提示文本 key 统一维护在 `config/data/tips.yaml`，`KIND_TIP` 会走 TTS 并复用 `config/data/temps/tts_cache` 缓存；动作模型绑定音频位于 `config/data/actions`，`robot_rl_manager` 只发布文件名。
TTS 默认开启，使用 `sherpa_onnx` Matcha 后端，模型和音量归一化等参数在 `audio_params.yaml` 中配置。

```bash
# 校验系统提示文本配置
python3 scripts/audio/tips.py check

# 根据 tips.yaml 生成 C++/Python 常量
python3 scripts/audio/tips.py generate

# 预生成系统提示文本的 TTS 缓存，避免首次播报等待
# 默认从已安装的 share/robot_audio/models/matcha-icefall-zh-baker 读取模型，需先 source install/setup.bash
python3 scripts/audio/tips.py warmup
```

`generate` 会更新源码中的 C++/Python tip key 常量；`warmup` 不更新源码，只在 `config/data/temps/tts_cache` 生成系统提示文本的缓存 wav。

### 3.8 监控服务（robot_monitor_node）

```bash
# 设置音量
ros2 service call /robot/monitor/set_volume robot_interfaces/srv/SetVolume \
  "{volume_percent: 80.0}"

# 设置静音
ros2 service call /robot/monitor/set_mute robot_interfaces/srv/SetMute \
  "{muted: true}"
```

### 3.9 数据记录（robot_recorder_node）

数据记录节点持续将控制命令、关节状态、IMU 与状态事件写入 `logs/` 目录下的 CSV 文件。

> 说明：recorder 节点当前不再提供 `ClearRecorderLogs` 清理服务，日志清理由文件系统或运维脚本管理。

### 3.10 故障注入（robot_injector）

```bash
# 注入关节电机错误（持续发布，直到节点关闭或故障被捕获）
ros2 launch robot_injector fault_inject.launch.py fault_type:=joint_motor

# 指定关节名称
ros2 launch robot_injector fault_inject.launch.py fault_type:=joint_motor joint_name:=waist_yaw

# 只注入一次（默认持续发布）
ros2 launch robot_injector fault_inject.launch.py fault_type:=joint_motor repeat_count:=1

# 交互式菜单选择
ros2 run robot_injector inject_fault.sh

# 故障恢复（与普通故障恢复相同）
ros2 run robot_injector inject_fault.sh --reset
```

支持的故障类型：

| 类型 | 说明 | 触发条件 |
|------|------|----------|
| `joint_motor` | 关节电机错误 | joint_error=1 |
| `joint_invalid_angle_state` | 关节反馈角度非法 | joint_error=2 |
| `joint_invalid_angle_cmd` | 关节目标命令非法 | joint_error=3 |
| `joint_zero_absence` | 关节零位缺失 | joint_error=4 |
| `driver_fault` | 驱动状态故障 | status=FAULT |
| `driver_fault_active` | 驱动故障激活 | fault_active=true |
| `rl_inference_error` | RL推理错误 | inference_error=true |
| `rl_not_ready` | RL未就绪 | online/model_loaded/model_ready=false |

---

## 4. Topic 实时观察命令

### 4.1 控制链观察

```bash
# 输入意图
ros2 topic echo /robot/input/intent

# 活跃意图（已准入）
ros2 topic echo /robot/router/active_intent

# 被拒意图（调试用）
ros2 topic echo /robot/router/rejected_intent

# RL 候选关节命令
ros2 topic echo /robot/rl_manager/desired_commands

# 外部控制指令（低频，BEST_EFFORT）
ros2 topic echo /robot/external/desired_commands

# 外部头部控制指令
ros2 topic echo /robot/external/head_control

# 外部控制关节硬件状态反馈
ros2 topic echo /robot/external/hardware_states

# 已准入关节命令
ros2 topic echo /robot/router/accepted_commands

# 最终硬件命令（发给驱动/仿真）
ros2 topic echo /robot/control/hardware_commands

# 被拒关节命令（调试用）
ros2 topic echo /robot/control/rejected_commands

# 准备插值进度 (0.0 ~ 1.0)
ros2 topic echo /robot/control/preparation_progress
```

### 4.2 反馈链观察

```bash
# 驱动原始关节状态（并联空间）
ros2 topic echo /robot/driver/joint_states

# 控制层关节状态（当前模型期望空间，供上层使用）
ros2 topic echo /robot/control/hardware_states
```

### 4.3 状态与诊断

```bash
# 系统状态（状态机权威输出）
ros2 topic echo /robot/state

# 驱动状态
ros2 topic echo /robot/driver/status

# RL 管理器状态
ros2 topic echo /robot/rl_manager/status

# 系统监控状态
ros2 topic echo /robot/monitor/system_status

# 版本信息
ros2 topic echo /robot/version

# 电池信息
ros2 topic echo /robot/battery/battery_info
```

### 4.4 传感器

```bash
# IMU 完整导航数据
ros2 topic echo /robot/imu/nav_all

# IMU 标准 ROS 消息
ros2 topic echo /robot/imu/imu_data_ros

# 外部速度指令（导航输入）
ros2 topic echo /cmd_vel

# 深度图（RL 观测输入）
ros2 topic echo /robot/camera/depth/image_rect_raw
```

### 4.5 音频

```bash
# 音频播报请求
ros2 topic echo /robot/audio/request

# 音频播报状态
ros2 topic echo /robot/audio/status
```

### 4.6 手柄输入

```bash
# 原始手柄数据
ros2 topic echo /joy
```

---

## 5. 接口定义查询

```bash
# 消息定义
ros2 interface show robot_interfaces/msg/RobotIntent
ros2 interface show robot_interfaces/msg/RobotState
ros2 interface show robot_interfaces/msg/JointCommands
ros2 interface show robot_interfaces/msg/JointCommand
ros2 interface show robot_interfaces/msg/JointStates
ros2 interface show robot_interfaces/msg/JointState
ros2 interface show robot_interfaces/msg/DriverStatus
ros2 interface show robot_interfaces/msg/RlManagerStatus
ros2 interface show robot_interfaces/msg/BatteryInfo
ros2 interface show robot_interfaces/msg/SystemStatus
ros2 interface show robot_interfaces/msg/AudioRequest
ros2 interface show robot_interfaces/msg/SpeakStatus
ros2 interface show robot_interfaces/msg/VersionInfo

# 服务定义
ros2 interface show robot_interfaces/srv/SetRobotMode
ros2 interface show robot_interfaces/srv/ResetFault
ros2 interface show robot_interfaces/srv/LoadRlModel
ros2 interface show robot_interfaces/srv/SwitchRlModel
ros2 interface show robot_interfaces/srv/EncosGetMotorId
ros2 interface show robot_interfaces/srv/EncosSetMotorId
ros2 interface show robot_interfaces/srv/EncosSetMotorZero
ros2 interface show robot_interfaces/srv/EncosGetMotorParam
ros2 interface show robot_interfaces/srv/SetVolume
ros2 interface show robot_interfaces/srv/SetMute

# 第三方接口
ros2 interface show yesense_interface/msg/NavAll
ros2 interface show realsense2_camera_msgs/msg/IMUInfo
ros2 interface show realsense2_camera_msgs/msg/Extrinsics
ros2 interface show realsense2_camera_msgs/msg/Metadata
```

---

## 6. 节点与系统命令

### 6.1 启动

```bash
# 完整启动（真实硬件默认）
ros2 launch robot_bringup zl_robot.launch.py

# MuJoCo 仿真启动
ros2 launch robot_bringup zl_robot.launch.py enable_mujoco_sim:=true

# 启动 OTA 升级服务
ros2 launch robot_bringup zl_robot_ota.launch.py

# 启动 MQTT 桥接
ros2 launch robot_bringup zl_robot.launch.py enable_mqtt:=true
```

### 6.2 Launch 参数速查

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_sim_time` | `false` | 使用仿真时钟 |
| `enable_mujoco_sim` | `false` | 使用 MuJoCo 仿真（代替真实硬件） |
| `enable_joy_node` | `true` | 启动手柄驱动 |
| `enable_teleop` | `true` | 启动遥控意图节点 |
| `enable_rl` | `true` | 启动 RL 管理节点 |
| `enable_audio` | `true` | 启动音频提示节点 |
| `enable_recorder` | `true` | 启动数据记录节点 |
| `enable_battery` | `false` | 启动电池节点 |
| `enable_monitor` | `true` | 启动系统监控节点 |
| `enable_mqtt` | `false` | 启动 MQTT 桥接节点 |
| `enable_camera` | `false` | 启动 RealSense 相机 |
| `enable_external` | `true` | 启动外部控制接口节点 |
| `enable_domain_bridge` | `true` | 启动 DDS 域桥接（白名单放行话题到外部网口） |
| `domain_bridge_config` | `config/domain_bridge_robot.yaml` | 域桥接白名单配置文件路径 |
| `cyclonedds_config` | `config/cyclonedds_bridge_whitelist.xml` | 域桥接使用的 CycloneDDS 网卡绑定配置 |

### 6.3 构建

```bash
# 全量构建
colcon build --symlink-install

# 按包构建
colcon build --symlink-install --packages-select robot_control

# 按包测试
colcon test --packages-select robot_control && colcon test-result --verbose
```

### 6.4 运行脚本

```bash
# 生产启动脚本（由 systemd 调用）
./scripts/run.sh

# 停止脚本
./scripts/stop.sh
```

### 6.5 调试工具

```bash
# 查看节点列表
ros2 node list

# 查看 Topic 列表
ros2 topic list

# 查看 Service 列表
ros2 service list

# 查看计算图
rqt_graph

# 查看 TF 树
ros2 run tf2_tools view_frames.py

# 查看带宽使用
ros2 topic bw /robot/control/hardware_commands

# 查看发布频率
ros2 topic hz /robot/control/hardware_states

# 录制数据包
ros2 bag record -o robot_bag state control/hardware_states driver/joint_states

# 回放数据包
ros2 bag play robot_bag
```

### 6.6 日志与配置路径

| 内容 | 路径 |
|------|------|
| 控制链 CSV 记录 | `logs/` |
| 关节逻辑配置 | `config/robot_joints.yaml` |
| 仿真关节配置 | `config/robot_joints_sim.yaml` |
| 硬件映射配置 | `config/hw_encos.yaml` |
| RL 模型注册 | `config/humanoid/rl_models.yaml`（人形）/ `config/dog/rl_models.yaml`（四足） |
| 各节点参数 | `robot_bringup/config/*.yaml` |
| 系统运行日志 | `log/` |

---

## 7. RobotIntent 字段速查

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `header` | std_msgs/Header | — | 意图产生时间戳，用于超时判断 |
| `source` | string | — | 来源标识：`teleop` / `http` / `mqtt` / `sdk` / `cloud` / `test` |
| `intent_type` | string | — | 意图类别：`status` / `model` / `motion` / `config` / `reset` |
| `command` | string | — | 类别内具体操作 |
| `action` | string | — | model/config 的操作对象/参数 |
| `linear_x_mps` | float32 | m/s | 前后线速度（motion 时使用） |
| `linear_y_mps` | float32 | m/s | 左右线速度（motion 时使用） |
| `angular_z_radps` | float32 | rad/s | 偏航角速度（motion 时使用） |
| `priority` | uint8 | — | 准入优先级，数值越小优先级越高：teleop 固定 `0`（最高），`app` 取 `1~49`，`api/http/mqtt/cloud` 取 `50~99` |

---

## 8. 状态机状态速查

| 枚举值 | 状态名 | 说明 |
|-------|--------|------|
| 0 | BOOTING | 启动中 |
| 1 | INIT | 初始化 |
| 2 | IDLE | 空闲 |
| 3 | DO_READY | 准备动作插值中 |
| 4 | READY | 就绪 |
| 5 | RL_STANDBY | RL 待命 |
| 6 | RL_ACTIVE | RL 活跃（推理/运动） |
| 7 | RULE_STANDBY | 规则控制待命（预留） |
| 8 | RULE_ACTIVE | 规则控制活跃（预留） |
| 9 | TRANSITIONING | 状态转换中 |
| 10 | FAULT | 故障 |
| 11 | DAMP | 阻尼（放松） |
| 12 | SHUTDOWN | 关机中 |

```bash
# 实时查看状态
ros2 topic echo /robot/state --field state
```

---

## 9. 关节命令类型速查

| 枚举值 | 类型 | 说明 |
|-------|------|------|
| 0 | TYPE_NONE | 无类型 |
| 1 | TYPE_ZERO_MOMENT | 零力矩 |
| 2 | TYPE_DAMPING | 阻尼模式 |
| 3 | TYPE_PREPARE | 准备插值帧（DO_READY 阶段） |
| 4 | TYPE_RUN | 正常运行帧 |

---

## 10. 驱动状态速查

| 枚举值 | 状态名 | 说明 |
|-------|--------|------|
| 0 | STATUS_UNKNOWN | 未知 |
| 1 | STATUS_OFFLINE | 离线 |
| 2 | STATUS_ONLINE | 在线 |
| 3 | STATUS_DEGRADED | 降级 |
| 4 | STATUS_FAULT | 故障 |

---

## 11. 关节错误码速查

| 枚举值 | 错误名 | 说明 |
|-------|--------|------|
| 0 | JOINT_ERR_NONE | 无错误 |
| 1 | JOINT_ERR_MOTOR_WARN | 电机或驱动器底层告警 |
| 2 | JOINT_ERR_INVALID_ANGLE_STATE | 反馈角度/位置非法 |
| 3 | JOINT_ERR_INVALID_ANGLE_CMD | 目标角度命令非法 |
| 4 | JOINT_ERR_ZERO_ABSENCE | 零位缺失 |

---

## 12. DDS 域桥接白名单（domain_bridge）

机器人内部 DDS 流量（域 10）默认只走本地回环 `lo`，外部调试/导航网络（域 100）走指定的有线调试网卡。域桥接（domain_bridge）负责在两个域之间**按白名单转发话题**，白名单外的任何话题都不会跨网络暴露。白名单由两层配置共同决定：

- **网口层**：`config/cyclonedds_bridge_whitelist.xml`。把域 10 绑到 `lo`、域 100 绑到 有线调试口。文件名中的 "whitelist" 指限制域 100 只走指定网口，且明确不绑到 EtherCAT 网卡。
- **话题层**：`config/domain_bridge_robot.yaml`。`topics:` 列表里写出的话题才会在域 10 ↔ 100 之间转发，由 `zl_robot.launch.py` 通过标准 `ros2 run domain_bridge domain_bridge` 加载。文件名中的 "whitelist" 指只放行列表内的话题。

```bash
# 启动带域桥接白名单的机器人系统（默认已开启，可显式关闭）
ros2 launch robot_bringup zl_robot.launch.py enable_domain_bridge:=true
ros2 launch robot_bringup zl_robot.launch.py enable_domain_bridge:=false

# 用自定义白名单 / 网卡配置启动
ros2 launch robot_bringup zl_robot.launch.py \
  domain_bridge_config:=/path/to/your_bridge.yaml \
  cyclonedds_config:=/path/to/your_cyclonedds.xml
```

### 12.1 白名单话题与方向

当前 `domain_bridge_robot.yaml` 转发的话题（含方向 `from_domain → to_domain`）：

| 话题 | 消息类型 | 方向 | 说明 |
|------|----------|------|------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 100 → 10 | 外部速度指令 |
| `input/intent` | `robot_interfaces/msg/RobotIntent` | 100 → 10 | 外部意图指令 |
| `external/head_control` | `robot_interfaces/msg/HeadControl` | 100 → 10 | 外部头部控制 |
| `audio/request` | `robot_interfaces/msg/AudioRequest` | 100 → 10 | 外部音频请求 |
| `state` | `robot_interfaces/msg/RobotState` | 10 → 100 | 机器人状态 |
| `driver/status` | `robot_interfaces/msg/DriverStatus` | 10 → 100 | 驱动状态 |
| `driver/joint_states` | `robot_interfaces/msg/JointStates` | 10 → 100 | 关节状态 |
| `rl_manager/status` | `robot_interfaces/msg/RlManagerStatus` | 10 → 100 | RL 管理状态 |
| `version` | `robot_interfaces/msg/VersionInfo` | 10 → 100 | 版本信息 |
| `human_follow/follow_control` | `robot_interfaces/msg/FollowControl` | 10 → 100 | 跟随控制 |

### 12.2 QoS 配置与注意事项

`domain_bridge_robot.yaml` 中每个话题都**必须显式写 QoS**，因为域桥接会在源域建订阅、在目标域建发布，**该 QoS 同时用于桥接两端的订阅与发布**。QoS 不匹配会导致连不上或丢数据：

- **reliability（可靠性）**：桥接订阅端 QoS 必须**兼容源发布者的 offered QoS**。源是 `reliable` 时桥接订阅不能写成 `best_effort`（否则连不上、收不到）；反过来 `reliable` 订阅可接收 `best_effort` 发布。桥接发布端 QoS 也必须**兼容下游订阅者**的 requested QoS，否则下游收不到。当前白名单里控制/状态类低频关键话题（intent、head_control、audio、version、follow_control）用 `reliable`，高频状态反馈类（state、driver/*、rl_manager/status）用 `best_effort`。
- **history / depth（历史与深度）**：桥接只缓冲 `depth` 条消息。高频话题（如 `joint_states` 用 `depth: 1` + `best_effort`）为降低延迟会丢弃旧帧；若自定义高频话题照搬 `depth: 1` 可能丢数据，需按实际需求调大 `depth`。
- **durability / liveliness 等**：未在白名单中显式给出时取 ROS 2 默认值，一般无需调整；若对接的系统要求 `transient_local` 等，需在 yaml 中补齐对应字段。

### 12.3 新增自定义话题白名单

yaml 是通用的，支持任意话题，**无需改代码**，只需在 `topics:` 下新增一条并填全字段：

```yaml
  /your/custom_topic:
    type: your_msgs/msg/YourType      # 必须是已 build 且该消息包在 bridge 进程环境中 source 到的完整类型名
    from_domain: 10                   # 源域
    to_domain: 100                   # 目标域
    qos:
      reliability: reliable           # 需同时兼容源发布者与下游订阅者
      history: keep_last
      depth: 5
```

注意：

- 若自定义话题使用第三方/自研 `.msg`，**定义该消息的包必须已构建并在 domain_bridge 启动时 source 到环境**，否则报 "unknown type" 无法反序列化。
- 网口绑定是全局的（域 100 → `对外网口`），所有放行话题共用该网卡；若需某话题走另一块网卡，标准做法是不改代码地再起一个 domain_bridge 实例 + 另一份 cyclonedds xml，而非在同一份 yaml 内区分网口。
- 标准 `domain_bridge` 的 `topic` 字段支持正则，配合单一 `type` 可一次放行一批同类型话题。
