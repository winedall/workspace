# robot_rl_manager

强化学习策略模型管理与推理节点。负责加载 ONNX/TorchScript 模型、组装观测输入、执行推理、解码动作输出为关节命令。支持 49 种技能策略的动态切换（含迎宾 zl1~zl6 和 standby 待机姿态）。

## 架构

```
rl_manager_node.py (ROS 2 节点, 三定时器架构)
├── ModelRegistry           ← 模型元数据管理（YAML 两级索引）
├── PolicyRuntime           ← 推理编排器
│   ├── PolicyBackend       ← ONNX / TorchScript / Pipeline
│   │   ├── OnnxPolicyBackend (onnxruntime)
│   │   ├── TorchPolicyBackend (torch.jit)
│   │   └── PipelinePolicyBackend (无模型)
│   └── PolicyAdapter       ← 每技能独立 Python 类
│       ├── build_model_input()   ← 传感器 → 模型输入
│       ├── decode_action()       ← 模型输出 → 关节位置
│       └── is_motion_end()       ← 动作完成判断
└── JointCommandBuilder     ← 组装 JointCommands + PD 增益
```

## 快速开始

```bash
# 默认启动（加载 t4_walk 模型，50Hz 推理，200Hz 命令发布）
ros2 run robot_rl_manager robot_rl_manager_node

# 切换模型（运行时）
ros2 service call /robot/rl_manager/switch_model robot_interfaces/srv/SwitchRlModel "{model_name: 't4_heart'}"

# 加载新模型
ros2 service call /robot/rl_manager/load_model robot_interfaces/srv/LoadRlModel "{
  model_name: 't4_wave',
  model_path: 'model/t4_wave.onnx',
  model_type: 'onnx',
  config_path: 'config/t4_wave.yaml',
  adapter_path: 'policy/adapters/t4_wave.py'
}"
```

## ROS 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `robot_joints_config` | string | `""` | 关节配置 YAML 路径 |
| `rl_models_config` | string | `""` | 模型索引 YAML 路径 |
| `prepare_profiles_config` | string | `""` | 准备姿态配置路径 |
| `default_prepare_profile` | string | `"stand"` | 默认起立姿态 |
| `inference_frequency_hz` | float | `50.0` | 模型推理频率 |
| `command_publish_frequency_hz` | float | `200.0` | 命令发布频率（含插值） |
| `status_publish_frequency_hz` | float | `1.0` | 状态发布频率 |
| `onnx_intra_op_num_threads` | int | `2` | ONNX 单算子内部线程数（0=默认） |
| `onnx_inter_op_num_threads` | int | `1` | ONNX 算子间并行线程数（0=默认） |
| `onnx_execution_mode` | string | `sequential` | ONNX 执行模式：`sequential`/`parallel` |
| `default_model_name` | string | `"walk"` | 默认模型 |
| `robot_type` | string | `"humanoid"` | 机器人类型 |
| `nav_all_timeout_sec` | float | `0.5` | IMU 超时阈值 |
| `cmd_vel_timeout_sec` | float | `0.3` | 外部速度指令超时 |
| `default_nav_mode` | string | `"manual"` | 导航模式 |

## 三定时器架构

```
推理定时器 (50 Hz)
  → 仅在 RL_ACTIVE 状态下运行推理
  → policy_runtime.infer_commands(context)
  → 缓存到 _latest_commands

命令发布定时器 (200 Hz)
  → 重新发布缓存的最后帧命令
  → 高频重发确保控制平滑

状态定时器 (1 Hz)
  → 发布 RlManagerStatus（模型加载/推理状态/延迟/IMU 可用性）
```

### 状态机门控

| 系统状态 | RL Manager 行为 |
|----------|----------------|
| `RL_ACTIVE` | 正常运行推理 |
| `DO_READY` | 发布准备姿态命令（TYPE_PREPARE 单次 + TYPE_RUN 持续） |
| `DAMP` | 发布阻尼命令（kp=0, kd=3） |
| `IDLE` | 发布零力矩命令（kp=0, kd=0） |
| `FAULT` + 手动阻尼意图 | 发布阻尼命令 |

## 模型切换

```
意图/服务 → 校验切换规则
  ├── pairs:       双向切换组 (squat_down ↔ squat_up)
  ├── sequences:   顺序链 (stand_to_crawl → crawl)
  └── default_model_name: 切换基准与自动回退模型

→ 保存最后帧命令
→ 初始化 OutputSmoother（10 帧平滑过渡）
→ 创建新 PolicyRuntime（新后端 + 新适配器）
→ 加载模型、重置适配器状态
```

当 `is_motion_end()` 返回 True 时，自动切回 `default_model_name`。

## 适配器分类（49 个）

### A. 连续运动（walk 类）
`t4_walk`, `t4_sprint`, `t4_depth`, `t4_kd_velocity`
- 92 + 460 维观测（5 帧历史堆叠）
- `is_motion_end()` 始终返回 False

### B. 参考动作一次性技能
`t4_heart`, `t4_high_jump_02`, `t4_long_jump_l`, `t4_front_flip`（前空翻）, `t4_jd_jump`, `t4_side_flip`, `t4_taichi`, `t4_mj_dance`, `t4_yb1-4`, `t4_salute`, `t4_lay_up`, `t4_face_up`, `t4_zl1`~`t4_zl6`（迎宾动作组） 等
- 144 维观测 + 27 维动作 + `.npz` 参考动作数据
- `is_motion_end()` 在动作结束时返回 True
- 使用 IMU 四元数做偏航锚定

### C. 循环技能
`t4_hello`, `t4_clap`, `t4_handshake`, `t4_rhello`
- 含 `LOOP_START_FRAME`/`LOOP_END_FRAME`，调用 `finish_loop()` 退出循环

### D. 爬行/复杂技能
`t4_kd_crawl`, `t4_kd_leg`, `t4_kd_stand_to_crawl`, `t4_kd_php_boxclimb`
- 627 维观测（16 帧历史）

### E. 四足（D4）
`d4_flat`（270 维观测）, `d4_getup`（45 维观测）
- 12 维动作 → 12 个关节

### F. 姿态（Pipeline，无模型）
`t4_squat_down`, `t4_squat_up`, `t4_raise_down`, `t4_sit_down`
- 使用 PipelinePolicyBackend，适配器直接控制输出

### 动作音频绑定（v1.2.9，v1.2.11 重构）

模型切换时自动触发预录制 WAV 音频播放。通过 `config/humanoid/action_audio.yaml` 配置：

```yaml
action_audio:
  enabled: true
  base_dir: "../data"
  bindings:
    t4_zl1:
      - file: "哈喽初来乍到请多多关照_16k.wav"
        delay_sec: 0.0
        priority: 120
```

`rl_manager_node` 在模型切换时查找绑定配置，按 `delay_sec` 延时后发布 `AudioPlay` 消息到 `audio/play`。

**v1.2.11 起**：配置菜单语音（getup/speed/nav/mode 切换播报）从 TTS 文本合成迁移为预录制提示音——通过 `robot_audio.audio_client.AudioClient` + `audio_tips` 常量播放 WAV 文件（键值定义在 `config/data/tips.yaml`，由 `scripts/audio/tips.py` 生成代码常量）。

## 添加新技能（6 步）

### 1. 放置模型文件
```
model/t4_wave.onnx          # ONNX 模型
model/t4_wave.npz           # 参考动作数据（可选，含 joint_pos/vel + body_quat_w）
```

### 2. 创建模型配置
`config/t4_wave.yaml`：
```yaml
model:
  name: t4_wave
  control_frequency_hz: 50
  observation_dim: 144
  action_dim: 27
  ankle_in_series: true
joint_profiles:
  run:
    joints:
      - {joint_name: head_yaw, position: 0.0, kp: 20.0, kd: 1.0}
      # ... 全部 31 个关节
```

### 3. 编写适配器
`policy/adapters/t4_wave.py`，实现 `PolicyAdapter` Protocol 的 5 个方法：
- `required_inputs()` → `{"joint_states", "imu"}`
- `build_model_input(context)` → 构建观测向量
- `decode_action(raw_action, context)` → 27 维 → 31 关节位置
- `is_motion_end()` → 动作是否完成
- `reset()` → 重置内部状态

一次性的参考动作技能可参考 `t4_heart.py`，循环技能参考 `t4_hello.py`。

### 4. 注册模型
在 `rl_models.yaml` 中添加：
```yaml
models:
  t4_wave:
    alias: "挥手"
    category: "单次动作类"
    duration_sec: 0.0
    has_duration: false
    config_path: "config/t4_wave.yaml"
    adapter_path: "policy/adapters/t4_wave.py"
    artifacts:
      x86_64:
        model_path: "model/t4_wave.onnx"
        backend: onnx
      aarch64:
        model_path: "model/t4_wave.rknn"
        backend: rknn
```

### 5. 添加切换规则（可选）
```yaml
model_switch_rules:
  pairs:
    - ["t4_wave", "t4_walk"]
```

### 6. 确保 setup.py 包含新文件
```python
('share/' + package_name + '/config', glob('config/*.yaml')),
('share/' + package_name + '/model', glob('model/*')),
```

## 话题

| 类型 | 话题 | 用途 |
|------|------|------|
| **Sub** | `control/hardware_states` | 关节反馈 |
| **Sub** | `imu/nav_all` | IMU 数据 |
| **Sub** | `/cmd_vel` | 外部速度指令 |
| **Sub** | `router/active_intent` | 当前活跃意图 |
| **Sub** | `state` | 系统状态 |
| **Sub** | `camera/depth/image_rect_raw` | 深度图 |
| **Pub** | `rl_manager/desired_commands` | 策略输出关节命令 |
| **Pub** | `rl_manager/status` | 状态报告 |
| **Srv** | `rl_manager/load_model` | 加载模型 |
| **Srv** | `rl_manager/switch_model` | 切换模型 |

## 关键设计决策

- **Backend/Adapter 分离**：Backend 只做张量推理，不知机器人语义；Adapter 拥有所有机器人特定逻辑
- **PolicyContext**：单一数据类承载所有帧级数据，新增传感器不改 Adapter 签名
- **PolicyImu**：厂商无关的 IMU 表示，在节点层从 NavAll 转换
- **关节映射**：人形 29 DoF（省略 waist_pitch/roll）+ 2 头部 = 31 关节，POLICY_IDX 做 29→27 映射
- **OutputSmoother**：模型切换时 10 帧线性混合防止命令跳变（v1.2.11 起由 25 帧缩短）
