# ZL Robot 控制系统设计说明

> 本文描述当前仓库中已经实现的系统，而不是早期规划或推荐架构。内容以 `robot_bringup/launch/zl_robot.launch.py`、各节点源码、`robot_interfaces` 接口定义和现行 YAML 配置为准。
>
> 对尚未接入运行链路的包或接口，本文会明确标为“预留”或“未使用”。

## 1. 系统范围

ZL Robot 是基于 Ubuntu 22.04 和 ROS 2 Humble 的人形机器人控制系统。系统支持两种互斥的执行后端：

- 真实硬件：`robot_driver_node` 通过 Encos/EtherCAT 驱动执行器，Yesense 节点提供 IMU 数据；
- MuJoCo 仿真：`sim_t4_std` 替代真实驱动和真实 IMU，对外保持相同的关节状态、驱动状态和 IMU 接口。

当前主控制能力包括：

- 遥控意图输入；
- 系统状态机、开机自检和故障管理；
- 控制意图与候选关节命令准入；
- RL 模型加载、切换、准备动作和推理；
- 关节命令过滤、平滑和踝关节串并联转换；
- Encos 电机运动控制与低频维护服务；
- 运行数据记录、电池读取、系统资源监控和语音提示；
- systemd 启停和 CycloneDDS 本机通信部署。

`robot_api` 和 `robot_tools` 当前只有包骨架，没有可执行节点，也没有接入 bringup。HTTP、MQTT、SDK、规则控制器和通用动作服务器不属于当前已实现运行能力。

## 2. 运行入口与部署基线

生产部署入口为：

```text
zl_robot.service
  └─ scripts/run.sh
       ├─ 加载 ROS 2 Humble 与工作区环境
       ├─ 选择 rmw_cyclonedds_cpp
       ├─ 设置 ROS_LOCALHOST_ONLY=0，并由 CycloneDDS 配置绑定内部/外部网口
       ├─ 配置 EtherCAT 可执行文件权限
       └─ ros2 launch robot_bringup zl_robot.launch.py
```

`scripts/stop.sh` 由 systemd 的 `ExecStop` 调用；`scripts/shutdown.sh` 用于开发环境中的强制停止和残留进程清理。

### 2.1 Launch 参数

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `use_sim_time` | `false` | 是否使用 ROS 仿真时钟 |
| `robot_joints_config` | 按 `enable_mujoco_sim` 选择 | 关节逻辑配置路径（sim/real 自动切换） |
| `hw_encos_config` | `config/hw_encos.yaml` | Encos 硬件映射配置路径 |
| `enable_mujoco_sim` | `false` | 选择 MuJoCo 或真实硬件路径 |
| `enable_joy_node` | `true` | 启动 ROS `joy_node` |
| `enable_teleop` | `true` | 启动遥控意图节点 |
| `enable_rl` | `true` | 启动 RL 管理节点 |
| `enable_audio` | `true` | 启动语音节点 |
| `enable_recorder` | `true` | 启动数据记录节点 |
| `enable_battery` | `true` | 启动电池节点 |
| `enable_monitor` | `true` | 启动系统监控节点 |
| `enable_camera` | `true` | 启动 RealSense 相机 |
| `enable_external` | `true` | 启动外部控制接口节点 |

代码内参数默认值通常偏保守，而 `robot_bringup/config/*.yaml` 是当前部署覆盖值。判断实际运行行为时必须同时检查 launch 和 YAML，不能只看代码默认值。

### 2.2 ROS 命名规则

自研节点源码、bringup 参数文件和 `domain_bridge_robot.yaml` 中的内部 ROS topic/service 均使用相对名，例如 `input/intent`、`control/hardware_commands`、`driver/status`。`zl_robot.launch.py` 在整机启动时统一注入 `robot` namespace，因此部署后的完整名称解析为 `/robot/input/intent`、`/robot/control/hardware_commands` 等。

例外项：

- ROS 系统话题 `/rosout`、`/parameter_events` 保持系统绝对名；
- 外部标准入口 `/cmd_vel` 保持根级绝对名；
- MQTT topic 如 `/mqtt/robot/state` 不属于 ROS namespace 规则；
- 第三方维护包按其上游约定处理，不在本文的相对 topic 约束范围内。

## 3. 包与组件

| 包 | 当前职责 | Bringup 状态 |
|---|---|---|
| `robot_interfaces` | 控制系统自定义消息、服务和动作定义 | 基础依赖 |
| `robot_common` | 关节配置、限位、YAML、踝关节转换和高频 QoS | 基础依赖 |
| `robot_state` | 系统状态机和安全状态权威 | 始终启动 |
| `robot_router` | RobotIntent 与候选 JointCommands 准入 | 始终启动 |
| `robot_control` | 过滤、平滑、准备插值、串并联转换 | 始终启动 |
| `robot_driver` | Encos/EtherCAT 真实硬件适配 | 非 MuJoCo 时启动 |
| `robot_rl_manager` | 模型注册、适配、推理和候选命令生成 | 可选，默认启动 |
| `robot_teleop` | F710 Joy 到 RobotIntent 的转换 | 可选，默认启动 |
| `robot_bringup` | 全系统 launch 和参数装配 | 启动入口 |
| `robot_audio` | 本地 TTS、播报队列和状态 | 可选，默认启动 |
| `robot_recorder` | 高频控制链 CSV 记录与故障窗口记录 | 可选，默认启动 |
| `robot_battery` | USB-CAN BMS 数据读取 | 可选，默认启动 |
| `robot_monitor` | CPU、温度、内存、磁盘和电池监控 | 可选，默认启动 |
| `yesense_interface` | Yesense 自定义消息 | 真实 IMU 依赖 |
| `yesense_std_ros2` | Yesense 串口采集与解码 | 非 MuJoCo 时启动 |
| `sim_t4_std` / `mj_sim` | MuJoCo 机器人仿真及 ROS 适配 | MuJoCo 时启动 |
| `robot_camera` | RealSense 深度相机驱动与 ROS 桥接 | 可选，默认启动 |
| `robot_external` | 外部控制接口：头部控制指令转换与插值 | 可选，默认启动 |
| `robot_injector` | 故障注入工具：模拟关节/驱动/RL故障，测试状态机响应 | 未接入 bringup，独立使用 |
| `robot_api` | 外部 API 预留包 | 未接入，无可执行节点 |

## 4. 总体架构与数据流

### 4.1 高层意图链

```text
joy_node (/joy)
  → robot_teleop_node
  → input/intent [RobotIntent]
  → robot_router_node
  ├→ router/active_intent
  │    ├→ robot_state_node
  │    └→ robot_rl_manager_node
  └→ router/rejected_intent
```

外部输入表达状态切换、模型、配置和速度意图，不直接发布执行级关节命令。

### 4.2 关节命令链

```text
robot_rl_manager_node
  → rl_manager/desired_commands       候选命令，串联关节空间
  → robot_router_node                              合并外部指令 + 来源/状态/序号/白名单准入
  → router/accepted_commands
  → robot_control_node                             过滤、限幅、平滑、插值
  → control/hardware_commands               最终命令，踝关节为并联空间
  → robot_driver_node 或 sim_t4_std
  → 电机或 MuJoCo

外部控制源
  → external/head_control                  头部控制指令（绝对/相对角度）
  → robot_external_node                           相对/绝对转换 + 限幅 + 插值
  → external/desired_commands              外部关节命令，低频 BEST_EFFORT
  → robot_router_node                             缓存后与 RL 指令合并（enable AND 运算）
  → external/hardware_states               外部控制关节的硬件状态反馈
```

`robot_rl_manager` 不允许绕过 router 和 control 直接访问执行后端。外部控制源通过 `external/desired_commands` 输入，router 在 mix 模式下将其与 RL 指令合并（position/kp/kd 覆盖，enable 取 OR）。

### 4.3 关节反馈链

```text
robot_driver_node 或 sim_t4_std
  → driver/joint_states                     原始反馈，真实机为并联空间
  → robot_control_node
       环境输出空间（仿真=串联，实体=并联）
       模型期望空间（由当前模型 ankle_in_series 决定）
       空间不匹配时转换到模型期望空间
  → control/hardware_states                 匹配当前模型期望空间
  ├→ robot_state_node
  ├→ robot_rl_manager_node
  └→ robot_recorder_node
```

因此，上层算法应订阅 `control/hardware_states`，不能把 `driver/joint_states` 当作统一算法空间接口。注意 `hardware_states` 的空间不固定——串联模型（heart/walk 等）得到串联空间，并联模型（long_jump）得到并联空间。

### 4.4 IMU 与速度输入

```text
Yesense 或 sim_t4_std → imu/nav_all ─────┐
                                                ├→ robot_rl_manager_node → PolicyContext
/cmd_vel（auto 导航模式）─────────────────────────┤
router/active_intent（manual 遥控模式）────┘
```

RL 节点根据 `config/nav` 意图在 `manual` 和 `auto` 间切换。manual 使用已准入 RobotIntent 的速度；auto 使用第三方 `/cmd_vel`，并执行超时和模型速度上限保护。

### 4.5 旁路监控链

- `robot_recorder_node` 旁路订阅 desired、accepted、hardware commands、hardware states 和 RobotState，不参与闭环控制；
- `robot_battery_node` 发布 `battery/battery_info`；
- `robot_monitor_node` 汇总系统资源和电池信息到 `monitor/system_status`；
- state、RL 和 monitor 可向 `audio/voice_text` 发布语音文本，audio 节点负责播放。

## 5. 核心接口

### 5.1 关节命令与状态

`JointCommand` 表达一个逻辑关节的目标位置、速度、力矩/推力、`kp`、`kd` 和使能请求。名称必须来自 `robot_joints.yaml`，消息中不包含 motor ID、从站号或通道号。

`JointCommands` 是执行链的一帧命令，关键字段为：

- `trace_id`：贯穿候选、准入和控制输出的关联标识；
- `sequence`：当前发布者输出序号；
- `parent_sequence` / `parent_source`：直接上游；
- `source` / `stage`：当前生产者和处理阶段；
- `type`：`TYPE_NONE`、`TYPE_ZERO_MOMENT`、`TYPE_DAMPING`、`TYPE_PREPARE` 或 `TYPE_RUN`；
- `commands`：本帧关节命令列表。

`JointState` 包含位置、速度、力矩/推力、使能和关节错误码。`JointStates` 还包含反馈序号，以及最后应用命令的 `trace_id` 和 sequence，用于命令—反馈关联。

### 5.2 RobotIntent

`RobotIntent` 是高层输入统一格式，包含：

- 来源、意图类型和命令；
- `linear_x_mps`、`linear_y_mps`、`angular_z_radps`；
- model 或 config 使用的统一 `action` 字段；
- 优先级。

当前 teleop 会产生以下主要意图：

- `status`: `ready`、`walk`、`damp`、`reinit`；
- `motion`: 周期性 `walk` 速度；
- `model`: 模型选择；
- `config`: 起身姿态、标准/高速、manual/auto 导航；
- `reset`: `reset_fault`。

Joy 信号超过 `joy_timeout_sec` 未更新后，teleop 将连续速度归零。一次性事件优先于周期 motion，避免状态或配置事件被速度帧覆盖。

### 5.3 状态与诊断

- `RobotState`：状态机唯一权威输出，含当前状态、模式、控制源、活动模型、硬件使能和故障信息；
- `DriverStatus`：执行后端在线、使能、故障和关节统计；
- `RlManagerStatus`：模型、推理、输入完备性、频率和错误状态；
- `BatteryInfo`：BMS 原始电池数据；
- `SystemStatus`：主机资源、电池换算值和阈值告警；
- `VoiceText` / `SpeakStatus`：语音请求与播放状态。

## 6. 系统状态机

状态枚举定义在 `RobotState.msg`，实现位于 `robot_state`：

```text
BOOTING → INIT → IDLE
                   │ ready（健康条件满足）
                   ↓
                DO_READY ──准备插值完成──→ READY
                   │                         │ walk
                   │ damp                    ↓
                   └──────────────────────→ DAMP   RL_STANDBY → RL_ACTIVE
                                                ↑ reinit    ↑ walk
                                                │           │
                                               IDLE ←──────┘ (通过 damp→reinit→IDLE→ready→walk)

任意运行状态 ──故障──→ FAULT
DAMP ──reinit──→ IDLE
```

消息中还保留 `RULE_STANDBY`、`RULE_ACTIVE`、`TRANSITIONING` 和 `SHUTDOWN`。当前 bringup 没有规则控制器，也没有完整使用 transitioning/shutdown 运行流程。

### 6.1 READY 前置条件

进入准备流程需要同时满足：

- 关节配置已加载；
- 开机自检通过；
- 无故障；
- DriverStatus 在线；
- JointStates 已接收且没有阻断级错误。

开机自检只检查 `enable_default=true` 的关节。状态数据稳定后，验证位置不是精确零值且位于配置限位内。温度告警允许继续运行；其他已上报的关节错误进入 FAULT。

### 6.2 准备动作

IDLE 收到 `ready` 后进入 DO_READY。RL 节点首次生成 `TYPE_PREPARE` 目标帧，后续周期帧使用 `TYPE_RUN`；control 以首次 PREPARE 为触发，以 `prepare_interpolation_cycles` 指定的周期数从当前反馈插值到目标姿态，并发布 `control/preparation_progress`。插值期间 control 丢弃非 PREPARE 帧，进度达到 1.0 后 state 进入 READY。

准备期间重复收到相同目标不会重启插值；目标变化会重新开始。离开 DO_READY 会取消未完成插值，防止其阻塞后续 DAMP 命令。

### 6.3 RL 运行

READY 收到 `walk` 后进入 RL_STANDBY。只有当 RL 状态在线、模型已加载且 ready、无推理错误、必需输入齐全时，状态机才进入 RL_ACTIVE。RL_ACTIVE 中模型状态异常（离线、未加载或未就绪）或推理错误会触发 FAULT。

### 6.4 FAULT

- 驱动故障、关节错误和 RL 活动期故障进入 FAULT；
- `reset_fault` 只有在底层输入恢复健康时才成功，恢复后回到 IDLE，不直接进入 READY；

`RobotState.fault_reason` 记录首次进入 FAULT 的原因，格式示例：

| 故障类型 | fault_reason 示例 |
|----------|------------------|
| 关节错误 | `joint state error: waist_yaw(error=1), left_knee(error=3)` |
| 驱动故障 | `driver status fault: status=4, fault_active=true` |
| RL推理错误 | `rl manager inference error: <error_message>` |
| RL未就绪 | `rl manager not ready: online=false, model_loaded=true, model_ready=false` |
| 开机自检 | `self-check: joint waist_yaw position=0.0 (通讯异常)` |

## 7. 命令路由

`robot_router_node` 是高层意图和候选关节命令的统一准入点，同时负责外部指令合并与控制模式切换。

### 7.1 意图准入

- FAULT 时阻断所有运动意图；
- DO_READY 期间拒绝模型切换，允许 damp 打断；
- 高优先级可以覆盖低优先级，低优先级不能抢占当前意图；
- stop 高于普通 teleop，teleop 高于 API 类来源。

消息显式 `priority` 与来源默认优先级共同决定结果。当前没有自动释放 active intent 的独立定时流程，连续运动依赖 teleop 周期发布及其 Joy 超时归零。

### 7.2 控制模式切换

router 支持两种控制模式，通过 RobotIntent 切换：

| intent_type | command | action | 效果 |
|---|---|---|---|
| `config` | `mode` | `rl` | 纯 RL 模式，不合并外部指令 |
| `config` | `mode` | `mix` | 混合模式，合并外部指令 |

- **rl 模式**：RL 指令透传，不合并外部指令。
- **mix 模式**：将缓存的外部指令与 RL 指令合并，position/kp/kd 按关节名覆盖，enable 取 OR 运算。无外部指令时回退到 RL 指令。

### 7.3 JointCommands 准入

router 拒绝以下候选命令：

- 空命令；
- FAULT 中的命令；
- 硬件未使能且未显式允许仿真绕过；
- 同一 source 的 sequence 重复或回退；
- 当前状态不允许的来源；
- 未知或重复的关节名。

通过后，router 保留 trace，更新 parent 字段，将自身 source 写为 `command_router`、stage 写为 `accepted`，并使用自己的输出 sequence。

router 默认要求驱动明确在线且无故障才放行关节命令，判断依据是 `robot_driver` 或仿真节点上报的 `DriverStatus.online`、`fault_active` 和 `status` 字段。

### 7.4 外部指令合并

router 维护外部指令缓存，通过 `external/desired_commands` 接收并缓存最新一帧。控制模式决定合并行为：

| 模式 | 行为 |
|------|------|
| `rl` | 不合并，RL 指令透传 |
| `mix` | 合并外部指令：按关节名覆盖 position/kp/kd，enable = RL.enable OR external.enable |

mix 模式下无外部指令时回退到 RL 指令。外部指令使用 BEST_EFFORT QoS，兼容各种外部来源（HTTP API、WebSocket 等）。

router 同时从 `control/hardware_states` 过滤出外部控制关节的状态，发布到 `external/hardware_states`，供外部控制源获取反馈。

## 8. RL 管理器

`robot_rl_manager_node` 由三部分组成：

- model registry：读取 `config/rl_models.yaml`；
- policy backend：模型使用 ONNX 后端（推理模型）、TorchScript 后端（PyTorch 推理模型）或 Pipeline 后端（硬编码插值，不依赖神经网络）；
- policy adapter/runtime：把关节状态、IMU 和速度意图转换为模型观测，再把动作转换为 `JointCommands`。

模型按功能类型分为行走类、参考动作类和 Pipeline 类（硬编码插值），每个条目关联模型配置、模型文件和适配器。

三个周期相互分离：

| 周期 | 当前部署频率 | 用途 |
|---|---:|---|
| 推理 | 50 Hz | 更新策略动作 |
| 命令发布 | 200 Hz | 发布最新候选关节命令 |
| 状态发布 | 1 Hz | 发布 `RlManagerStatus` |

命令行为由 RobotState 控制：DO_READY 产生一次 prepare 帧，DAMP 产生阻尼命令，RL_ACTIVE 执行推理并发布 run 命令。IMU 默认新鲜度阈值为 0.2 s，外部 `/cmd_vel` 默认超时为 0.3 s。

## 9. 控制安全层

`robot_control_node` 是执行后端之前的最后一个运动安全处理节点。

处理顺序为：

1. 检查 RobotState 是否阻断运动；
2. 对 PREPARE 执行专用多周期插值；
3. 对普通帧执行关节白名单、NaN/Inf 和物理边界过滤；
4. 踝关节空间转换：模型输出空间（`ankle_in_series`）适配环境空间（`ankle_env_parallel`），空间不匹配时做串↔并转换；
5. 发布 `control/hardware_commands`。

当前部署关键参数：控制频率 200 Hz。

被拒绝帧可发布到 `control/rejected_commands`。该 topic 由参数控制，当前部署启用。

## 10. 驱动与仿真后端

### 10.1 真实驱动

`robot_driver` 内部分层为：

```text
robot_driver_node
  → HardwareBackend 抽象
  → EncosBackend
  → EncosProtocol
  → EthercatMaster / SoemMaster
  → Encos 执行器
```

`robot_joints.yaml` 描述逻辑关节、限位和默认增益；`hw_encos.yaml` 描述 `motor_id`、`slave_id`、`passage`、方向、偏置、力矩系数和发送队列。上层模块不依赖硬件映射字段。

驱动订阅最终 hardware commands，以 200 Hz 发布关节状态，以 10 Hz 发布 DriverStatus。低频 Encos 管理服务与周期运动通道分离，避免维护请求阻塞实时控制回调。

### 10.2 MuJoCo

`enable_mujoco_sim=true` 时不启动 `robot_driver_node` 和真实 Yesense 节点。`sim_t4_std`：

- 订阅 `control/hardware_commands`；
- 发布 31 关节统一视图到 `driver/joint_states`；
- 发布在线、无故障的 `driver/status`；
- 发布 `imu/nav_all`；
- 默认使用串联踝关节空间（MuJoCo 模型为串联关节）。串联模型（heart/walk 等）输出匹配，跳过转换；并联模型（long_jump）输出为并联空间，control 做 P2S（并联→串联）匹配仿真环境。

MuJoCo 模型包含 29 个身体关节，对外补齐两个头部关节以维持系统统一的 31 关节接口。

## 11. Topic 拓扑

### 11.1 控制与状态主链

| Topic | 类型 | 发布者 | 订阅者 | 典型频率/QoS |
|---|---|---|---|---|
| `joy` | `sensor_msgs/Joy` | `joy_node` | teleop | 输入设备频率，depth 10 |
| `input/intent` | `RobotIntent` | teleop | router | 20 Hz 或事件触发，depth 10 |
| `router/active_intent` | `RobotIntent` | router | state、RL | 系统默认 QoS |
| `router/rejected_intent` | `RobotIntent` | router | 调试消费者 | 系统默认 QoS |
| `rl_manager/desired_commands` | `JointCommands` | RL | router、recorder | 200 Hz，高频 QoS |
| `external/head_control` | `HeadControl` | 外部控制源 | external | 事件触发，depth 10；角度制（度），节点内转弧度 |
| `external/desired_commands` | `JointCommands` | external | router | 外部频率，SensorDataQoS |
| `external/hardware_states` | `JointStates` | router | external | 200 Hz，SensorDataQoS |
| `router/accepted_commands` | `JointCommands` | router | control、recorder | 高频 QoS |
| `control/hardware_commands` | `JointCommands` | control | driver 或 simulator、recorder | 200 Hz，高频 QoS |
| `driver/joint_states` | `JointStates` | driver 或 simulator | control | 200 Hz，高频 QoS |
| `control/hardware_states` | `JointStates` | control | state、RL、recorder | 200 Hz，高频 QoS |
| `control/rejected_commands` | `JointCommands` | control | 调试消费者 | 高频 QoS，可关闭 |
| `control/preparation_progress` | `std_msgs/Float32` | control | state | 准备期间，系统默认 QoS |
| `driver/status` | `DriverStatus` | driver 或 simulator | state、router | 真实驱动 10 Hz；仿真 5 Hz |
| `state` | `RobotState` | state | router、control、RL、recorder | 20 Hz，系统默认 QoS |
| `rl_manager/status` | `RlManagerStatus` | RL | state | 1 Hz，depth 10 |
| `imu/nav_all` | `yesense_interface/NavAll` | Yesense 或 simulator | RL | depth 10 |
| `camera/depth/image_rect_raw` | `sensor_msgs/Image` | RealSense 相机 | RL | depth 10 |
| `rl_manager/finish_loop` | `std_msgs/Empty` | teleop 或外部 | RL | 事件触发 |
| `rl_manager/motion_stop` | `std_msgs/String` | teleop 或外部 | RL | 事件触发 |
| `/cmd_vel` | `geometry_msgs/Twist` | 外部导航 | RL | 外部输入，depth 10 |

高频 QoS 统一为 `rclcpp::SensorDataQoS`（KeepLast(5)、BEST_EFFORT、VOLATILE）。原先的自定义 helper `robot_common::highFrequencyControlQoS()` 已移除，代码直接复用 ROS 内置的 `SensorDataQoS`；与旧实现相比仅 depth 由 1 变为 5，reliability/durability 保持不变。DDS 兼容性只看 reliability/durability，因此与 `SystemDefaultsQoS`（RELIABLE）发布端及仿真器仍兼容。外部 topic 同样使用 SensorDataQoS（KeepLast(5)、BEST_EFFORT、VOLATILE）以兼容各种外部来源。

跨域桥接 `config/domain_bridge_robot.yaml` 中：`best_effort`（传感器/外部）类 topic 已统一对齐到 depth 5，与 `SensorDataQoS` 一致；`reliable`（系统默认）类 topic（如 `input/intent`、`external/head_control`、`audio/request`、`version`、`human_follow/follow_control`）已统一对齐到 depth 10，与 `rclcpp::SystemDefaultsQoS()`（RELIABLE、KeepLast(10)、VOLATILE）一致。`version` 的发布端（`robot_monitor_node.cpp` 使用 `SystemDefaultsQoS()`，即 depth 10）与桥接读取端 depth 10 完全匹配。

### 11.2 辅助链路

| Topic | 类型 | 发布者 | 订阅者 |
|---|---|---|---|---|
| `version` | `VersionInfo` | monitor | 外部消费者 |
| `audio/voice_text` | `VoiceText` | state、RL、monitor | audio |
| `audio/status` | `SpeakStatus` | audio | 外部消费者 |
| `battery/battery_info` | `BatteryInfo` | battery | monitor |
| `monitor/system_status` | `SystemStatus` | monitor | 外部消费者 |

语音文本链使用 RELIABLE、TRANSIENT_LOCAL，depth 10，使晚加入的 audio 节点可以取得最近的提示。

## 12. Service 与 Action

| Service | 类型 | 服务端 | 用途 |
|---|---|---|---|---|
| `state/reset_fault` | `ResetFault` | state | 健康条件恢复后的故障复位 |
| `rl_manager/load_model` | `LoadRlModel` | RL | 加载模型元数据、配置和后端 |
| `rl_manager/switch_model` | `SwitchRlModel` | RL | 切换已注册模型 |
| `driver/encos/get_motor_id` | `EncosGetMotorId` | driver | 读取电机 ID |
| `driver/encos/set_motor_id` | `EncosSetMotorId` | driver | 修改电机 ID |
| `driver/encos/set_motor_zero` | `EncosSetMotorZero` | driver | 设置电机零位 |
| `driver/encos/get_motor_param` | `EncosGetMotorParam` | driver | 读取厂家参数 |
| `audio/speak` | `Speak` | audio | 请求语音播放 |
| `recorder/clear_logs` | `ClearRecorderLogs` | recorder | 按保留天数清理日志 |
| `monitor/set_volume` | `SetVolume` | monitor | 设置系统音量 |
| `monitor/set_mute` | `SetMute` | monitor | 设置系统静音 |

Encos 写操作会改变电机身份或标定结果。接口文件注释声明真实运动写入模式应拒绝危险维护操作，但当前 service callback 和 Encos backend 中没有实现这项状态检查。因此这些服务目前只能依靠调用权限和工程流程隔离，不能把接口注释视为已生效的运行保护。

## 13. 配置边界

### 13.1 `config/robot_joints.yaml`

包含 31 个逻辑关节，分为 head、left_arm、right_arm、waist、left_leg、right_leg。每个关节定义分组、默认使能、单位、位置/速度/力矩边界和默认控制增益。

使用者包括 common、state、router、control、driver 和 RL。它是逻辑控制配置，不保存厂家总线细节。

### 13.2 `config/hw_encos.yaml`

包含逻辑关节到 Encos/EtherCAT 的硬件映射，只由驱动路径使用。逻辑配置与硬件配置通过稳定的 joint name 对齐。

### 13.3 `config/rl_models.yaml` 与模型配置

模型注册表关联模型名、模型配置、ONNX 文件和 adapter。模型自己的观测、动作维度、关节映射和增益位于 `robot_rl_manager/config/*.yaml`。

### 13.4 Bringup 参数

`robot_bringup/config/*.yaml` 保存节点运行频率、超时、限幅、记录策略、设备路径和告警阈值。launch 只负责选择节点、选择真实/仿真后端并传入配置路径。

## 14. 安全机制与边界

系统现有安全机制分布在多层：

1. teleop：摇杆死区、速度上限、事件锁存、Joy 丢失后速度归零；
2. router：优先级、状态阻断、sequence、来源和关节白名单；
3. state：开机自检、关节错误、RL 就绪、FAULT 恢复约束；
4. control：物理限幅、增益钳位、单步跳变、准备插值、状态阻断；
5. driver：协议错误映射、维护/运动通道隔离和驱动状态上报；
6. recorder/monitor：故障前后数据留存和资源、电池告警。

需要注意的当前边界：

- router 的部署配置允许硬件未使能时放行，主要服务仿真；真实部署应审查该覆盖值；
- 温度告警当前只记录并允许继续运行，没有实现自动降额；
- `RULE_*` 状态存在，但规则控制器尚未接入；
- API接口尚未实现；
- Encos 设置 ID、设置零位服务尚未按接口注释实现“真实运动写入时拒绝”的运行检查；
- `use_sim_time` 与 `enable_mujoco_sim` 是独立参数，默认启动 MuJoCo 但 `use_sim_time=false`；当前仿真节点使用系统运行节拍，不应假设一定存在 `/clock`；
- Topic 名称主要硬编码在节点中，新增或重命名接口时必须同步更新代码、测试、launch 和本文档。

## 15. 设计结论

当前系统的核心边界如下：

```text
teleop               只产生高层意图
robot_router         统一准入意图和候选关节命令，合并外部指令，控制模式切换
robot_state          是系统状态唯一权威
robot_rl_manager     管理策略并产生候选命令（空间由模型 ankle_in_series 决定）
robot_control        完成最终运动安全处理和串并联空间转换
driver/simulator     是可互换的执行与反馈后端
recorder/monitor     只旁路观察，不参与控制决策
```

所有新控制源应先接入 `RobotIntent` 或受控的候选 `JointCommands` 入口，不能直接发布 `control/hardware_commands`。所有上层状态消费者应使用 `control/hardware_states`，后者已按当前模型空间做转换，不应直接依赖 `driver/joint_states`。

外部控制源可通过 `external/desired_commands` 输入关节命令，router 在 mix 模式下合并到控制链中，并通过 `external/hardware_states` 获取对应关节的硬件状态反馈。
