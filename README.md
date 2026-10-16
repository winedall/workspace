# zl_robot

双足/四足机器人全身运动控制系统。基于 ROS 2 Humble + MuJoCo 仿真 + ONNX 强化学习推理，实现真机与仿真环境统一的控制管线。

**当前版本：v1.2.11**

## 系统架构

```
┌──────────────────────────────────────────────────────────────┐
│  遥 控 层                                                     │
│  ┌──────────┐  ┌──────────────┐  ┌─────────────────────────┐ │
│  │ F710手柄  │  │ HTTP/API/云  │  │ 外部头部控制              │ │
│  └────┬─────┘  └──────┬───────┘  └───────────┬─────────────┘ │
│       │               │                      │                │
│  ┌────┴───────────────┴──────────────────────┴─────────────┐ │
│  │                 robot_router (命令仲裁中枢)                │ │
│  │        优先级仲裁 + 安全准入 + RL/外部合并                 │ │
│  └────────────────────────┬────────────────────────────────┘ │
│                           │                                   │
│  ┌────────────────────────┴────────────────────────────────┐ │
│  │               robot_control (安全过滤层)                  │ │
│  │    关节限幅 + 步长平滑 + 踝关节变换 + 准备插补             │ │
│  └────────────────────────┬────────────────────────────────┘ │
│                           │                                   │
│  ┌────────────────────────┴────────────────────────────────┐ │
│  │                robot_driver (硬件抽象层)                  │ │
│  │    Mock Backend / Encos Backend (EtherCAT + SOEM)        │ │
│  └─────────────────────────────────────────────────────────┘ │
│                           │                                   │
│                     ┌─────┴─────┐                             │
│                     │  真机硬件   │                             │
│                     │  or 仿真   │                             │
│                     └───────────┘                             │
└──────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────┐
│  智 能 层                                                     │
│  ┌──────────────────────────────────────────────────────────┐│
│  │  robot_rl_manager (RL 策略管理)                           ││
│  │  42 种技能策略 + ONNX/TorchScript 推理 + 参考动作回放      ││
│  └──────────────────────────────────────────────────────────┘│
│  ┌──────────────────────────────────────────────────────────┐│
│  │  robot_state (系统状态机)                                  ││
│  │  13 状态生命周期 + 自检 + 故障检测 + 意图处理              ││
│  └──────────────────────────────────────────────────────────┘│
└──────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────┐
│  基 础 设 施                                                  │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌───────────────┐  │
│  │IMU 驱动   │ │BMS 驱动   │ │TTS 语音   │ │健康监控        │  │
│  │robot_imu  │ │robot_bat  │ │robot_audio│ │robot_monitor   │  │
│  └──────────┘ └──────────┘ └──────────┘ └───────────────┘  │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌───────────────┐  │
│  │旁路记录    │ │故障注入    │ │OTA 更新   │ │仿真框架        │  │
│  │robot_rec  │ │robot_inj  │ │robot_ota  │ │robot_sim       │  │
│  └──────────┘ └──────────┘ └──────────┘ └───────────────┘  │
└──────────────────────────────────────────────────────────────┘
```

## 快速开始

### 系统依赖

- Ubuntu 22.04
- ROS 2 Humble
- MuJoCo 3.1.1（vendored，无需系统安装）
- SOEM (EtherCAT)、ONNX Runtime、Piper TTS、espeak-ng、ALSA

### 编译

```bash
cd ~/zl_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 启动

```bash
# 仿真模式（人形 T4，含头部和相机）
ros2 launch robot_bringup zl_robot.launch.py arg_enable_mujoco_sim:=true arg_robot_type:=humanoid

# 仿真模式（四足 D4）
ros2 launch robot_bringup zl_robot.launch.py arg_enable_mujoco_sim:=true arg_robot_type:=dog

# 真机模式（需硬件连接，默认配置）
ros2 launch robot_bringup zl_robot.launch.py

# 仅 OTA 管理服务
ros2 launch robot_bringup zl_robot_ota.launch.py
```

更多启动参数见 [`robot_bringup`](src/robot_bringup/README.md)。

### 系统配置（首次部署）

```bash
cd scripts
sudo bash install.sh   # 安装 udev 规则、配置动态库路径
```

## 包清单

### 控制管线（核心）

| 包 | 语言 | 说明 |
|----|------|------|
| [`robot_interfaces`](src/robot_interfaces/README.md) | ROS msg/srv | 系统全域 API 契约（14 消息 + 12 服务） |
| [`robot_bringup`](src/robot_bringup/README.md) | Python | 启动编排（14 节点 + 13 配置） |
| [`robot_router`](src/robot_router/README.md) | C++ | 命令仲裁中枢（优先级 + 准入 + 合并） |
| [`robot_control`](src/robot_control/README.md) | C++ | 安全控制层（滤波 + 平滑 + 踝变换） |
| [`robot_driver`](src/robot_driver/README.md) | C++ | 硬件抽象层（Mock/Encos + EtherCAT） |
| [`robot_state`](src/robot_state/README.md) | C++ | 系统状态机（13 状态 + 故障检测） |

### 智能层

| 包 | 语言 | 说明 |
|----|------|------|
| [`robot_rl_manager`](src/robot_rl_manager/README.md) | Python | RL 策略管理（42 技能 + ONNX/TorchScript） |
| [`robot_teleop`](src/robot_teleop/README.md) | Python | 遥控输入节点（F710 手柄 + 键盘） |
| [`robot_external`](src/robot_external/README.md) | Python | 外部头部控制接口 |

### 外设驱动

| 包 | 语言 | 说明 |
|----|------|------|
| [`robot_imu`](src/robot_imu/README.md) | C++ | YESENSE IMU 驱动（串口协议 + NavAll 发布） |
| [`robot_battery`](src/robot_battery/README.md) | C++ | BMS 电池驱动（USB-CAN） |
| [`robot_audio`](src/robot_audio/README.md) | Python | 离线中英 TTS（Piper ONNX + eSpeak） |
| [`robot_camera`](src/robot_camera/README.md) | C++/Python | RealSense D435 相机 + MQTT 桥接 |

### 系统工具

| 包 | 语言 | 说明 |
|----|------|------|
| [`robot_monitor`](src/robot_monitor/README.md) | C++ | 系统健康监控（CPU/内存/磁盘/电池 + 语音告警 + 音频守护） |
| [`robot_recorder`](src/robot_recorder/README.md) | C++ | 旁路数据记录（环形缓冲 + CSV 输出） |
| [`robot_injector`](src/robot_injector/README.md) | C++ | 故障注入工具（8 种故障类型） |
| [`robot_ota`](src/robot_ota/README.md) | Python | 固件 OTA 更新平台（FastAPI 服务端 + systemd Agent） |

### 仿真

| 包 | 语言 | 说明 |
|----|------|------|
| [`robot_sim`](src/robot_sim/mj_sim/README.md) | C++ | MuJoCo 仿真框架（4 种机器人 + 自动代码生成） |

### 基础库

| 包 | 语言 | 说明 |
|----|------|------|
| [`robot_common`](src/robot_common/README.md) | C++ | 共享基础库（关节配置 + 限幅 + 踝运动学 + QoS） |

## 数据流

```
遥控/App/API → Intent ──→ router (仲裁) ──→ state (状态机)
                                │
              ┌─────────────────┴─────────────────┐
              ↓                                   ↓
        rl_manager                            external
        (RL 推理)                            (头部控制)
              │                                   │
              └─────────────────┬─────────────────┘
                                ↓
                         router (合并 + 准入)
                       ↓
                  control (安全 + 平滑)
                       ↓
              ↓
          driver
       (全身关节)
              ↓
         硬件/仿真
              ↓
        JointStates 反馈 ←──
```

### 关键技术参数

- 控制频率：200 Hz
- RL 推理频率：50 Hz
- 命令发布频率：200 Hz（含插值）
- EtherCAT 周期：2 ms（可配）
- 准备插补时长：5 秒（默认 1000 周期）

## 控制模式

| 模式 | 说明 |
|------|------|
| **RL** | 纯强化学习控制，42 种技能策略（行走/舞蹈/跳跃/爬行/姿态） |
| **MIX** | 外部命令按关节覆盖 RL 输出（头部控制等场景） |
| **RULE** | 规则控制（预留） |

## 机器人支持

| 类型 | 代号 | 关节数 | 仿真模型 | 技能数 |
|------|------|--------|----------|--------|
| 人形双足（串联踝） | T4 | 29+2（头部） | sim_t4_std_add_head | 50 |
| 人形双足（并联踝） | T4 parallel | 29+2（头部） | sim_t4_std_parallel | 50 |
| 四足机器狗 | D4 | 12 | sim_d4 | 2 |

通过 `arg_robot_type` 参数选择：`humanoid`（人形）/ `dog`（四足），通过 `arg_parallel_ankle` 选择人形仿真踝关节环境。

## DDS 通信模式（v1.2.10/v1.2.11）

CycloneDDS 支持两种配置模式，通过 `ZL_ROBOT_DDS_MODE` 环境变量切换：

| 模式 | 配置文件 | 适用场景 |
|------|----------|----------|
| `localhost` | `cyclonedds_localhost.xml` | 默认。本机话题绑定 lo 网口，避免网络变化影响 |
| `bridge` | `cyclonedds_bridge_whitelist.xml` + `domain_bridge_robot.yaml` | 白名单形式将内部话题与外部网络隔离 |

## 安全机制

- **自检**：开机后自动校验关节通信和位置限幅
- **故障检测**：驱动故障、关节错误、RL 推理异常的多层检测
- **故障恢复**：手动阻尼 → 故障复位 → 重新自检
- **安全默认**：上电 BOOTING → 无运动输出 → 需显式授权
- **Deadman 开关**：遥控信号丢失 0.3s 自动零速
- **校准故障检测**：摇杆异常自动锁存零速
- **OTA 升级安全**：仅在非运动状态允许升级 + Ed25519 签名校验 + 自动回滚

## 文档

各包的详细文档见对应 `src/<包名>/README.md`。

顶层设计文档见 `docs/`：
- `zl_robot控制系统设计说明.md` — 控制系统整体设计
- `RobotIntent/` — 意图消息设计说明
- `Simulation/` — 仿真环境说明
- `OTASystem/` — OTA 系统设计
- `zl_robot命令速查手册.md` — 命令快速参考

## 编译与运行脚本

| 脚本 | 用途 |
|------|------|
| `scripts/build.sh` | 编译项目 |
| `scripts/run.sh` | 启动机器人（前台） |
| `scripts/stop.sh` | 停止机器人 |
| `scripts/install.sh` | 系统环境配置（udev/动态库/权限） |
| `scripts/release.sh` | 发布版本打包 |
| `scripts/ota.sh` | OTA 管理 |
| `scripts/logs.sh` | 日志管理（cleanup/list） |
| `scripts/injector.sh` | 故障注入工具 |

## 环境变量

| 变量 | 用途 |
|------|------|
| `ZL_ROBOT_CONFIG_ROOT` | 配置文件根目录（默认 `config/`） |
| `ZL_ROBOT_LOG_ROOT` | 日志根目录（默认 `logs/ros`） |
| `ZL_ROBOT_RUN_ID` | 本次运行唯一标识 |
| `ZL_ROBOT_RECORDER_DIR` | 数据记录输出目录 |
| `ZL_ROBOT_AGENT_ROOT` | OTA Agent 安装根目录（默认 `~/.zl_robot`） |
| `ZL_ROBOT_DDS_MODE` | DDS 模式：`localhost`/`bridge`（默认 `localhost`） |
| `ZL_ROBOT_ROBOT_TYPE` | 机器人类型：`humanoid`/`humanoid_parallel`/`dog` |
| `ZL_ROBOT_MUJOCO_SIM` | 仿真开关（默认 `true`） |
