# robot_control

控制管线安全层。位于 `robot_router` 和 `robot_driver` 之间，对上接收已准入的关节命令，执行安全过滤、踝关节坐标变换和准备插补，对下输出硬件安全的关节命令。

## 管线位置

```
RL Manager / 遥控 / 外部控制
        ↓
   robot_router (优先级仲裁 + 状态准入)
        ↓
 → robot_control (本包：安全过滤 + 踝变换 + 准备插补) ←
        ↓
   robot_driver (EtherCAT/CAN 硬件输出)
```

## 架构

```
RobotControlNode
├── JointCommandFilter     ← 第 1 层：白名单、NaN/Inf、位置/速度/力矩钳位
├── 踝关节坐标变换          ← 串联空间 ↔ 并联空间（FK + IK + Jacobian）
└── 准备插补控制器          ← 多周期位置插补（TYPE_PREPARE 触发）
```

## ROS 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `robot_joints_config` | string | `""` | 关节配置 YAML 路径 |
| `control_frequency_hz` | double | `200.0` | 控制循环频率（≥1 Hz） |
| `prepare_interpolation_cycles` | int | `1000` | 准备插补周期数（200Hz × 1000 = 5s） |
| `enable_rejected_command_topic` | bool | `true` | 发布被拒绝的命令帧 |
| `enable_mujoco_sim` | bool | `true` | 仿真模式 |
| `ankle_env_parallel` | bool | `false` | 环境踝空间：`true` 为并联，`false` 为串联；人形真机由 bringup 固定为并联 |
| `ankle_ff_mode` | string | `"off"` | 踝控制模式：`off`=等效增益；`torque`=纯力矩前馈；`hybrid`=混合前馈（见下） |
| `ankle_ff_torque_limit` | double | `30.0` | 前馈模式下单个踝电机的前馈力矩限幅（Nm） |
| `ankle_ff_scale` | double | `1.0` | 前馈权重：0=退化为等效增益，1=全量前馈；用于扫稳定性/精度折中 |
| `robot_type` | string | `"humanoid"` | 机器人类型（humanoid 启踝变换） |

## 话题

| 类型 | 话题 | 用途 |
|------|------|------|
| **Sub** | `router/accepted_commands` | 接收准入后的关节命令 |
| **Sub** | `driver/joint_states` | 接收驱动反馈 |
| **Sub** | `rl_manager/status` | 踝空间信息（ankle_in_series） |
| **Sub** | `state` | 系统状态（FAULT 阻塞判断） |
| **Pub** | `control/hardware_commands` | 输出给驱动的安全关节命令 |
| **Pub** | `control/hardware_states` | 转换后的关节状态（供上游使用） |
| **Pub** | `control/preparation_progress` | 准备插补进度 0.0~1.0 |
| **Pub** | `control/rejected_commands` | 被拒绝的命令帧（诊断用） |

## 命令处理完整流程

```
handleAcceptedCommands(JointCommands)
  │
  ├─ [1] 踝空间快进：cmd.ankle_in_series 与缓存不同 → 立即更新
  │
  ├─ [2] FAULT 阻塞：非 TYPE_DAMPING → 拒绝 + 发布到 rejected_commands
  │
  ├─ [3] TYPE_PREPARE：触发/重启动多周期插补，绕过普通 filter 路径
  │
  ├─ [4] DO_READY 守卫：非 PREPARE 帧 → 静默丢弃
  │
  ├─ [5] JointCommandFilter.filter()
  │     ├── 关节名白名单校验 → 未知关节名则整帧拒绝
  │     ├── NaN/Inf 检测（5 个 float 字段） → 任意异常则整帧拒绝
  │     ├── 位置钳位到 [min_pos, max_pos]
  │     ├── 速度钳位到 [-max_vel, +max_vel]
  │     ├── 力矩钳位到 [-max_effort, +max_effort]
  │     └── kp/kd 有限数检查通过后原样透传
  │
  └─ [6] publishHardwareCommands()
        ├── S2P 踝变换（命令路径，模型空间 → 环境空间）
        ├── 设置 parent_sequence/parent_source/source/stage 元数据
        └── 发布到 control/hardware_commands
```

## 命令类型处理策略

| type | FAULT 下 | filter | 特殊行为 |
|------|----------|--------|----------|
| `TYPE_NONE` (0) | 阻塞 | ✓ | 常规命令 |
| `TYPE_ZERO_MOMENT` (1) | 阻塞 | ✓ | - |
| `TYPE_DAMPING` (2) | **放行** | ✓ | 紧急停止通道 |
| `TYPE_PREPARE` (3) | 阻塞 | **绕过** | 触发多周期位置插补 |
| `TYPE_RUN` (4) | 阻塞 | ✓ | - |

## 安全机制对比

| 机制 | 层级 | 策略 | 失败行为 |
|------|------|------|----------|
| **Filter** | 控制输出前 | 白名单 + 范围钳位 + NaN 拒绝 | 整帧拒绝（不全量输出） |

关键设计：
- 普通运行命令只做基础安全过滤，不做逐周期平滑；平滑策略由上游模型或运控策略负责
- PREPARE 绕过普通 filter 路径（目标来自已知安全配置，插补本身提供斜坡）
- PREPARE 进行中时，所有非 PREPARE 帧被静默丢弃（防止 TYPE_RUN 覆盖插补中间值）

## 踝关节坐标变换

### 两种空间

| 空间 | 说明 | 使用场景 |
|------|------|----------|
| **串联空间** | ankle_pitch/roll 是直接电机角度 | 部分 RL 模型（walk, heart）+ 串联踝仿真 |
| **并联空间** | ankle_pitch/roll 是混合机构的输出角度 | 部分 RL 模型（long_jump）+ 真机硬件 + 并联踝仿真 |

### 变换规则

**状态路径**（驱动 → 上游）：
- 仿真 + 并联模型 → `convertToParallel`（S2P）
- 真机 + 串联模型 → `convertToSeries`（P2S）

**命令路径**（上游 → 驱动）：
- 串联模型 + 真机 → `convertToParallel`（S2P）
- 并联模型 + 仿真 → `convertToSeries`（P2S）

### 踝前馈模式（`ankle_ff_mode`）

实现在 **`ankle_command_converter.{hpp,cpp}`**（类 `AnkleCommandConverter`），与节点其余逻辑解耦：
节点只做三件事——构造时 `Configure()`、收到反馈时 `CacheEnvStates()/CacheModelStates()`、
下发前 `Convert()`。数值部分仍在 `robot_common/ankle_transform`。

仅在**模型串联 + 环境并联**（真机 / 并联踝仿真 + walk 类模型）时生效。两种模式共用
同一个期望力矩，差别在于它是否替代电机侧 PD：

```
τ_H = effort_ff + kp_H ⊙ (q*_H − q_H) + kd_H ⊙ (q̇*_H − q̇_H)     ← 串联空间，模型自带增益
τ_L = J(q_H,实测)⁻ᵀ · τ_H                                       ← 期望的并联电机力矩
```

**`torque`（纯力矩）**

```
下发：effort = τ_L（限幅），kp = kd = 0，position/velocity = 实测并联值
```

精度最高（不引入对角近似、不锁定 `r_opt` 比例、不丢弃 roll 增益，结果精确等于训练时
串联空间的 PD），但 PD 闭环从电机内部移到 200 Hz 链路：受延迟约束，可用刚度上限
≈ (φ/τ)²·J_eff；断流时电机会保持最后一帧力矩。

**`hybrid`（混合，推荐）**

```
下发：position/velocity/kp/kd = 等效增益转换结果（电机内闭合 PD，不动）
      effort = τ_L − [effort_conv + kp_L ⊙ Δq_L + kd_L ⊙ Δq̇_L]   ← 只补差额
```

电机侧实际输出 = 基线 PD + 差额 = τ_L（前馈新鲜时精确）。刚度、阻尼、断流安全仍由
电机内环承担，不受 200 Hz 延迟上限约束；前馈只承担"增益等效做不对"的那部分
（roll 增益、耦合项、姿态漂移）。前馈失效时自动退化为既有增益模式。

**安全约束（两种模式共用）**：实测状态缺失或踝关节数据不完整时整体回退到
`convertToParallel`；前馈力矩按 `ankle_ff_torque_limit` 限幅。

### 底层数学

- **FK**（`FastAnkleFk`）：串联角度 → 笛卡尔位置（MATLAB 生成闭式解）
- **IK**（`FastAnkleIk`）：笛卡尔位置 → 串联角度（Newton 迭代 + 阻尼，最大 20 次，容差 1e-5）
- **Jacobian**（`FastAnkleJacob`）：解析闭式 Jacobian
- 串联约束：pitch ∈ [-0.87, 0.51]，roll ∈ [-0.26, 0.26]
- 并联约束：sum ∈ [-0.36, 0.36]，diff ∈ [-1.2, 2.2]

每条腿 6 关节：`hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll`

## 准备插补

```
TYPE_PREPARE 到达
  → 记录目标帧 + 当前实际位置作为起点
  → preparing_ = true
  → 每周期 advancePrepareInterpolation():
      t = min(cycle / prepare_interpolation_cycles, 1.0)
      position = start + t × (target - start)
      发布进度 Float32
  → t >= 1.0: preparing_ = false
```

**中断/重启动**：新 PREPARE 目标不同 → 从当前位置重新插补；目标相同 → 忽略。

## 仿真 vs 真机

| 方面 | 串联环境 (`ankle_env_parallel=false`) | 并联环境 (`ankle_env_parallel=true`) |
|------|--------------------------------------|-------------------------------------|
| 环境空间 | 串联 | 并联 |
| 状态转换 | 并联模型时 S2P | 串联模型时 P2S |
| 命令转换 | 并联模型时 P2S | 串联模型时 S2P |
| 其他处理 | 相同 | 相同 |

`robot_type="humanoid"` 启用踝变换；其他值（如 dog）完全跳过。

## 关节错误处理

| 错误码 | 行为 |
|--------|------|
| `JOINT_ERR_NONE` (0) | 正常 |
| `JOINT_ERR_MOTOR_WARN` (1) | 仅告警（2s 节流） |
| 其他错误码 | 由 driver/state_manager 故障链路统一处理，control 不做逐关节冻结 |

## 输出帧元数据

每个 `hardware_commands` 帧携带全链路追踪信息：

```
sequence        = 自增（从 1 开始）
parent_sequence = 输入帧的 sequence
source          = "robot_control"
parent_source   = 输入帧的 source（如 "rl_manager"）
stage           = "control_output" | "prepare_interp"
```

追踪链：`rl_manager → command_router → robot_control → robot_driver`
