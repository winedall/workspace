# robot_bringup

系统启动编排包。提供启动文件（launch）和参数文件（YAML config），负责按正确顺序拉起所有节点并注入运行参数。

## 命名规则

`zl_robot.launch.py` 会为整机节点统一注入 `robot` namespace。自研节点代码和 bringup 配置中的内部 ROS topic/service 写相对名，例如 `state`、`control/hardware_commands`、`driver/status`；运行时会解析成 `/robot/state`、`/robot/control/hardware_commands`、`/robot/driver/status`。

ROS 系统话题 `/rosout`、`/parameter_events`、外部标准入口 `/cmd_vel` 以及 MQTT topic `/mqtt/robot/...` 不按内部相对 topic 规则改写。

## 快速启动

```bash
# 真机模式（需预先设置 ZL_ROBOT_CONFIG_ROOT）
ros2 launch robot_bringup zl_robot.launch.py

# 仿真模式
ros2 launch robot_bringup zl_robot.launch.py enable_mujoco_sim:=true parallel_ankle:=false

# 仅 OTA 服务
ros2 launch robot_bringup zl_robot_ota.launch.py
```

## 启动参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_sim_time` | `false` | 使用仿真时钟 |
| `robot_type` | `humanoid` | 机器人形态：`humanoid` / `dog` |
| `config_root` | `$ZL_ROBOT_CONFIG_ROOT` | 配置文件根目录 |
| `arg_file_robot_joints` | `{config_root}/{robot_type}/robot_joints_<profile>.yaml` | 关节配置文件路径；mix profile 使用 `robot_joints_mix.yaml` |
| `arg_hardware_profile` | `mix_canfd` | 硬件配置 profile：`encos` / `tienkung` / `mix_canfd` / `mix_ethercat` |
| `arg_file_hardware` | `{config_root}/{robot_type}/hw_{hardware_profile}.yaml` | 硬件配置文件路径 |
| `file_rl_models` | `{config_root}/{robot_type}/rl_models.yaml` | RL 模型目录配置 |
| `file_action_audio` | `{config_root}/{robot_type}/action_audio.yaml` | 动作音频编排配置 |
| `file_prepare_profiles` | `""` | 准备姿态配置文件（空则用内置默认） |
| `dir_recorder` | `logs` | 记录器输出目录 |
| `enable_mujoco_sim` | `false` | 使用 MuJoCo 仿真（替代真实 IMU + 驱动） |
| `parallel_ankle` | `true` | 人形仿真踝环境空间：`true` 为并联仿真，`false` 为串联仿真；真机人形固定并联 |
| `enable_teleop` | `true` | 启用手柄遥控 |
| `enable_keyboard` | `false` | 启用键盘遥控（与手柄并存） |
| `enable_joy_node` | `true` | 启用手柄驱动节点（`joy_node`） |
| `enable_rl` | `true` | 启用 RL 管理器 |
| `enable_audio` | `true` | 启用 TTS 语音 |
| `enable_recorder` | `true` | 启用数据记录器 |
| `enable_battery` | `false` | 启用电池监控（需 BMS 硬件） |
| `enable_monitor` | `true` | 启用系统健康监控 |
| `enable_camera` | `false` | 启用 RealSense 相机 |
| `enable_external` | `true` | 启用外部控制接口 |

## 启动的节点

系统按以下顺序启动 15 个节点：

```
 1. IMU 驱动 (yesense_std_ros2)          — 仅真机模式
 2. MuJoCo 仿真 (sim_*)                  — 仅仿真模式
 3. joy_node                              — 条件：enable_joy_node
 4. robot_driver_node                     — 仅真机模式
 5. robot_state_node                      — 始终启动 (respawn)
 6. robot_router_node                     — 始终启动 (respawn)
 7. robot_control_node                    — 始终启动 (respawn)
 8. robot_rl_manager_node                 — 条件：enable_rl (respawn)
 9. robot_teleop_node                     — 条件：enable_teleop (respawn)
10. robot_keyboard_node                   — 条件：enable_keyboard (respawn)
11. robot_external_node                   — 条件：enable_external
12. robot_audio_node                      — 条件：enable_audio
12. robot_recorder_node                   — 条件：enable_recorder
13. robot_battery_node                    — 条件：enable_battery
14. robot_monitor_node                    — 条件：enable_monitor
```

核心节点（state、router、control、rl_manager、teleop）设置了 `respawn=True, respawn_delay=1.0`，崩溃后自动重启。

## 控制管线数据流

```
joy_node → robot_teleop ──→ RobotIntent ──→ robot_router ──→ robot_control ──→ robot_driver → 硬件
                                ↑                ↑                                  ↑
                         robot_external    robot_rl_manager                   JointStates 反馈
```

- **robot_state** 发布系统状态（无命令输出），router 根据状态做准入判断。
- **robot_monitor** 发布 `SystemStatus`，independently 监控健康指标。
- **robot_recorder** 旁路监听控制话题，写入 CSV（不介入控制回路）。

## YAML 配置文件（13 个）

所有配置遵循 ROS 2 参数文件格式：顶层 key 为节点名，其下为 `ros__parameters`。

| 文件 | 对应节点 | 关键参数 |
|------|----------|----------|
| `state_params.yaml` | `robot_state_node` | 状态发布频率 |
| `router_params.yaml` | `robot_router_node` | 优先级配置、调试开关 |
| `control_params.yaml` | `robot_control_node` | 控制频率、PD 限幅、插补步数 |
| `driver_params.yaml` | `robot_driver_node` | 状态发布频率、故障去抖参数 |
| `rl_manager_params.yaml` | `robot_rl_manager_node` | 推理频率、默认模型、ONNX 线程配置、超时 |
| `teleop_params.yaml` | `robot_teleop_node` | 最大速度、死区、手柄超时 |
| `monitor_params.yaml` | `robot_monitor_node` | 告警阈值、音频守护 |
| `battery_params.yaml` | `robot_battery_node` | 设备路径、发布频率 |
| `recorder_params.yaml` | `robot_recorder_node` | 缓冲区容量、输出目录、记录通道 |
| `audio_params.yaml` | `robot_audio_node` | TTS 语速/音量、队列深度 |
| `camera_params.yaml` | `camera` | 序列号、深度/彩色流开关 |
| `external_params.yaml` | `robot_external_node` | 插补步数 |
| `ota_params.yaml` | `robot_ota_node` | 端口、密钥 |

## 环境变量

| 变量 | 用途 |
|------|------|
| `ZL_ROBOT_CONFIG_ROOT` | 配置文件根目录（必须设置） |
| `ZL_ROBOT_RECORDER_DIR` | 记录器 CSV 输出目录 |
| `ZL_ROBOT_RUN_ID` | 本次运行的唯一标识（用于日志目录命名） |
| `ZL_ROBOT_DDS_MODE` | DDS 模式：`localhost`/`bridge`（默认 `localhost`） |
| `ZL_ROBOT_ROBOT_TYPE` | 机器人类型：`humanoid`/`dog` |
| `ZL_ROBOT_MUJOCO_SIM` | 仿真开关（默认 `true`） |
| `ZL_ROBOT_ANKLE_ENV_PARALLEL` | 人形仿真踝环境空间（默认 `true`） |

## 运行模式

### 真机模式（默认）
```bash
export ZL_ROBOT_CONFIG_ROOT=/path/to/config
ros2 launch robot_bringup zl_robot.launch.py
```
- 启动真实 IMU（`yesense_std_ros2`）+ `robot_driver_node`
- 不启动仿真或相机节点（可手动开启）
- 默认使用 `mix_canfd` 硬件 profile；如需天工或 Encos，可传入 `arg_hardware_profile:=tienkung` 或 `arg_hardware_profile:=encos`

### 仿真模式
```bash
ros2 launch robot_bringup zl_robot.launch.py enable_mujoco_sim:=true parallel_ankle:=false
```
- 用 MuJoCo 仿真替代 IMU + 驱动
- 人形串联踝仿真启动 `sim_t4_std_add_head`，人形并联踝仿真启动 `sim_t4_std_parallel`，狗形启动 `sim_d4`

### 最小模式（调试用）
```bash
ros2 launch robot_bringup zl_robot.launch.py \
  enable_teleop:=false \
  enable_rl:=false \
  enable_audio:=false \
  enable_recorder:=false \
  enable_battery:=false \
  enable_monitor:=false \
  enable_external:=false
```
仅保留 state、router、control、driver 核心管线。

## 日志

所有节点统一添加 `--ros-args --disable-external-lib-logs` 参数，禁用单节点独立日志文件，将所有输出合并到统一的 run-log 中。运行日志由外部脚本 `scripts/logs.sh` 管理（提供 `cleanup` 和 `list` 子命令）。
