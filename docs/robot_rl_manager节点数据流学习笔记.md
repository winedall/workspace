# robot_rl_manager 节点数据流学习笔记

> 目标：用"数据从哪来 → 调用哪个函数 → 函数里怎么处理 → 处理完传给谁"的方式，看懂一次完整的控制循环。
> 所有引用均为源码路径 `src/robot_rl_manager/robot_rl_manager/` 下的 `文件:行号`。

---

## 0. 三十秒看懂这个节点

这个节点是"人形/狗形机器人的 RL 策略中间层"。它的工作可以用一句话概括：

**把机器人的传感器数据（关节角、IMU、速度指令）喂给一个训练好的神经网络模型（ONNX/Torch），
把模型输出的动作向量翻译成 `JointCommands`（每个关节的目标位置 + PD 增益），然后高频发布出去。**

它自己**不控制**电机，只负责"算出来想要的命令"并发布，真正的底层控制是别的节点做的。

模块划分（谁干什么）：

| 模块 | 文件 | 职责 |
|------|------|------|
| 节点层 | `rl_manager_node.py` | 订阅/发布 ROS 消息、编排流程、状态判断 |
| Runtime | `policy/runtime.py` | 一次推理的"总指挥"，编排 adapter + backend + builder |
| Adapter | `policy/adapters/t4_walk.py` 等 | "语义翻译"：把机器人的状态翻译成模型输入，把模型输出翻译成关节目标 |
| Backend | `policy/onnx.py`, `policy/torch.py` | "真正跑模型"：只承认 `{输入名: 浮点数组}` → 输出 `raw_action` 列表 |
| Builder | `joint_command_builder.py` | 按 `robot_joints.yaml` 的关节顺序，把位置向量组装成 `JointCommands` 消息 |
| Registry | `model_registry.py` | 管理所有模型的名字、配置、切换规则 |

---

## 1. 全局架构图（数据从哪来到哪去）

```
                        ┌───────────────────────────────────────────────────┐
                        │           RobotRlManagerNode (节点)               │
                        │                                                   │
 订阅（数据进来，只缓存）   │                                                   │
 ─────────────────────   │                                                   │
 /robot/control/hardware_states ──► _handle_joint_states ───────┐           │
 /robot/imu/nav_all ─────────────► _handle_nav_all(PolicyImu)──┤           │
 /cmd_vel ──────────────────────► _handle_external_cmd_vel ────┤           │
 /robot/router/active_intent ───► _handle_active_intent ────────┤           │
 /robot/state ──────────────────► _handle_robot_state ──────────┤  缓存到   │
 /robot/camera/depth/... ───────► _handle_depth_image ──────────┘  self._*  │
                                                                           │
 三个定时器（数据被消费）                                                 │
 ─────────────────────                                                    │
 ① 推理定时器 (~50Hz)  _handle_inference_timer                              │
        │                                                                │
        ▼  ___凑齐数据，执行一次推理__                                    │
        └──► PolicyRuntime.infer_commands(...)                           │
                │  (policy/runtime.py)                                    │
                ▼                                                        │
           1) 组装 PolicyContext（所有输入打成一个包）                    │
           2) adapter.build_model_input   → 模型输入 dict                 │
           3) backend.infer               → raw_action 列表              │
           4) adapter.decode_action       → 关节位置列表                  │
           5) builder.build               → JointCommands 消息           │
        │ 结果缓存在 self._latest_commands                               │
                                                                           │
 ② 发布定时器 (~200Hz) _handle_command_publish_timer                      │
        │                                                                │
        ▼ 把缓存的 _latest_commands 加上序号/时间戳                       │
        └──► publish ──► /robot/rl_manager/desired_commands               │
                                                                           │
 ③ 状态定时器 (~1Hz)  _handle_status_timer                                │
        ▼                                                                │
        └──► publish ──► /robot/rl_manager/status                          │
                        └───────────────────────────────────────────────────┘

 下游：/robot/rl_manager/desired_commands → command_router → robot_control(真正控制电机)
```

**核心设计：订阅回调只"存"数据，三个定时器才"用"数据。**
订阅回来的消息都是高频的，比如 `hardware_states`，回调里不处理只缓存
（`rl_manager_node.py:302` 三行就结束了），保证不阻塞。

---

## 2. 主线流程：一次完整的推理控制循环

我们从"机器人正在运行 `t4_walk` 行走模型、处于 `RL_ACTIVE` 状态"这个最常见的场景切入。

### 2.1 数据是怎么进来的（订阅回调，4 个关键来源）

一次推理需要 4 样输入，它们分别在各自的订阅回调里被缓存：

| 输入 | 来源 topic | 回调函数 | 缓存到的变量 |
|------|-----------|----------|-------------|
| 关节状态 | `/robot/control/hardware_states` | `_handle_joint_states` `rl_manager_node.py:302` | `self._latest_joint_states` |
| IMU 姿态/角速度 | `/robot/imu/nav_all` | `_handle_nav_all` `rl_manager_node.py:328` | `self._latest_policy_imu`（转成 PolicyImu） |
| 速度指令(手动) | `/robot/router/active_intent` | `_handle_cmd_vel_intent` `rl_manager_node.py:432` | `self._latest_cmd_vel` |
| 速度指令(自动) | `/cmd_vel` | `_handle_external_cmd_vel` `rl_manager_node.py:323` | `self._latest_external_cmd_vel` |
| 当前系统状态 | `/robot/state` | `_handle_robot_state` `rl_manager_node.py:261` | `self._latest_state` |

注意 `_handle_nav_all` 做的"转换"：厂家消息里是**度**，policy 需要**弧度**，所以它把
roll/pitch/yaw 和角速度统一 `math.radians()` 转成弧度，封装成厂商无关的 `PolicyImu`
（`policy/imu.py`），这样 adapter 就不依赖具体 IMU 型号了。

### 2.2 推理定时器触发（每 20ms 一次）

`main` 里 `rclpy.spin` 起来后，定时器按各自频率回调。推理定时器走
`_handle_inference_timer`（`rl_manager_node.py:710`），它是整个节点的"大脑"。

```
_handle_inference_timer(rl_manager_node.py:710)
│
├─ ① 检查基础条件 _base_command_conditions_ok() (line 834)
│      - 没收到过关节状态？→ 直接不干活
│      - 状态是 FAULT/SHUTDOWN？→ 不干活
│      - 不是 DO_READY/READY/DAMP/IDLE/RL_STANDBY/RL_ACTIVE？→ 不干活
│      - 不满足 → self._latest_commands = None，return
│
├─ ② 看看当前状态要不要"不用推理"的固定命令 _profile_commands_for_current_state() (line 858)
│      按 state 分流，全都不满足则返回 None（走下一步真正推理）：
│      - FAULT + 手动damp意图 → 阻尼命令
│      - IDLE → 零力矩命令
│      - DAMP → 阻尼命令
│      - DO_READY → prepare 命令（站立/躺下等固定姿态）★首次是 TYPE_PREPARE 只发一次
│      - READY → prepare 的 run 类型命令
│      - RL_STANDBY / RL_ACTIVE 且未发首帧 → run profile 首帧命令
│      返回非None → _set_latest_commands() 缓存，return（不推理）
│
├─ ③ 判断要不要真的推理 _should_run_inference() (line 814)
│      - 状态必须是 RL_ACTIVE（行走过程中）
│      - 不能缺输入（_missing_policy_inputs）
│
├─ ④ 真正推理 _policy_runtime.infer_commands(...) (line 735)  ★核心，见 2.3
│      组装输入：cmd_vel（来自 _resolve_cmd_vel line 1008）
│                joint_states / policy_imu / depth / now_sec
│      成功 → 得到 JointCommands
│      失败 → 记录 _inference_error，_latest_commands=None，快速发布状态
│
├─ ⑤ 缓存结果 _set_latest_commands(commands) (line 902)
│      填 trace_id、时间戳、stage="desired"，做切换平滑 smoother.filter
│
└─ ⑥ 一次性模型播完了吗？ (line 756)
       is_motion_end() → _handle_model_motion_end() (line 759)
       如果当前是一次性模型(如比心/挥手)且参考动作播完 →
       自动切回默认模型 walk，或推进序列 model sequence 的下一段
```

### 2.3 核心：PolicyRuntime.infer_commands 的一次数据旅行

这是数据"穿过"整个模型层的过程，也是你要重点看的部分（`policy/runtime.py:81`）：

```
infer_commands(runtime.py:81)
│
├─ 1) _make_context (runtime.py:164)
│      把 cmd_vel、joint_states、imu、depth、上一帧的 action/positions、
│      当前时间 now_sec、距上一帧的 dt_sec 全部打成一个 PolicyContext
│      （policy/context.py 的数据类，adapter 只认这一个"包裹"）
│
├─ 2) _validate_required_inputs (runtime.py:192)
│      问 adapter：你必需哪些输入？→ required_inputs()
│      walk 要 {"joint_states", "imu"} (t4_walk.py:117)
│      上下文缺哪个就抛错，缺输入不允许跑模型
│
├─ 3) adapter.build_model_input(context)  ★语义翻译第①步
│      walk 具体实现 t4_walk.py:119
│      做的事：
│        - 取 IMU 角速度、欧拉角投影重力方向 (t4_walk.py:137-140)
│        - 平滑速度指令（每周期最多变0.05）(t4_walk.py:141)
│        - 按 AMP 29关节顺序取关节位置/速度，减去参考姿态 (t4_walk.py:148-154)
│        - 更新步态相位 phase（走多快 → 周期0.6/0.8s→sin/cos）(t4_walk.py:263)
│        - 把 7 组特征拼成 obs[92]，并把本帧追加进5帧历史 deque → hist[460]
│      返回 {"obs": [...92个数...], "hist": [...460个数...]}
│
├─ 4) backend.infer(model_input)  ★真的让模型算
│      onnx 实现 onnx.py:72
│        - 检查输入名与模型 input 完全一致（不多不少）
│        - 每个输入 ndarray reshape 成 (1, -1)
│        - session.run → 取输出[0] → 展平成 list → raw_action(27维)
│      返回 raw_action = [动作, 动作, ...27个数]
│
├─ 5) adapter.decode_action(raw_action, context)  ★语义翻译第②步
│      walk 具体实现 t4_walk.py:175
│        - 校验是 27 维（AMP_POLICY_IDX 数量）
│        - 记下 self._last_action（下一帧观测要用）
│        - 从 run profile 取 31 关节的参考位置 (t4_walk.py:216)
│        - 27 维动作按索引映射到 29 个真实关节，加上 0.25*比例的动作偏移
│        - 输出按 robot_joints.yaml 顺序的 31 个目标位置
│
├─ 6) _validate_decoded_action_dim (runtime.py:238)
│      校验解码出的数量 == 关节配置数，不一致就是 bug
│
├─ 7) builder.build(profile, positions)  ★组装 ROS 消息
│      joint_command_builder.py:105
│        - profile 取 self._model_info.joint_profiles["run"]（t4_walk.yaml 的 kp/kd）
│        - 逐个关节：position 用算出来的值，kp/kd 用 profile 里的值
│        - 输出 JointCommands(frame.commands = [31 个 JointCommand])
│
└─ return JointCommands  ▲回给节点层的 _handle_inference_timer
```

把 2.3 画成一张更细的"数据变换链"，每一格都是数据类型变化：

```
joint_states/imu/cmd_vel/...         ← 原始 ROS 消息，可能含角度(度)、冗余状态
        │  _make_context
        ▼
PolicyContext（统一上下文包）         ← 节点/runtime 的"交接单"
        │  adapter.build_model_input
        ▼
{"obs": [92], "hist": [460]}        ← 模型的输入，纯数字
        │  backend.infer (onnx)
        ▼
raw_action = [27个数]               ← 模型的原始输出
        │  adapter.decode_action
        ▼
[31 个关节目标位置]                   ← 与 robot_joints.yaml 顺序对应
        │  builder.build
        ▼
JointCommands（31 个 JointCommand)   ← 节点发布的正式消息
```

**一句话总结这条链：`传感器 → 上下文包 → 模型输入字典 → 动作向量 → 关节位置 → ROS消息`。**
每换一层，就换一个模块负责；所以想改"模型怎么算"改 backend，想改"怎么翻译"改 adapter。

### 2.4 发布定时器：把算好的命令发出去（每 5ms 一次）

推理 50Hz 算一帧，但命令要 200Hz 发布，所以有第二个定时器重复广播同一帧
`_handle_command_publish_timer`（`rl_manager_node.py:791`）：

```
_handle_command_publish_timer (rl_manager_node.py:791)
│
├─ 能否发布？ _should_publish_commands() (line 824)
│      RL_ACTIVE 但缺输入 → 不发
│      基础条件不过 / 有推理错误 → 不发
│
├─ _latest_commands 是 None → return（没有可发的）
│
├─ sequence+1、打上当前时间戳 header.stamp
│
├─ _commands_pub.publish(_latest_commands)  → 发到 /robot/rl_manager/desired_commands
│
└─ 如果是 TYPE_PREPARE 类型（首帧站立这种只发一次的命令）
      → 清空 _latest_commands，避免下次重复发
```

为什么同一个命令发 4 次？因为下游 command_router/robot_control 可能也在高频
接收，200Hz 下哪怕漏几帧也总能收到，保证实时性。

### 2.5 状态定时器

`_handle_status_timer`（`rl_manager_node.py:804`）→ `_publish_status_now` →
`_make_status_msg`（`rl_manager_node.py:916`）。汇总模型是否 ready、缺哪些输入、
推理是否出错等等，1Hz 发布到 `/robot/rl_manager/status`，给 state_manager 判断用。

---

## 3. 另外两条分支数据流

### 3.1 不推理的那条：固定姿态命令（DO_READY 起身为例）

机器人从倒地到站立的流程，不经过模型推理，走 `_profile_commands_for_current_state`
（`rl_manager_node.py:870`）：

```
state_manager 把状态置为 DO_READY
        │ _handle_robot_state (rl_manager_node.py:265)
        ▼ 若是 DO_READY 且当前模型与起身配置不符
_sync_model_for_prepare_profile() (line 287)
        ▼ 比如起身配置是"躺下"→切换 t4_lay_up 模型（line 71 的映射表）
推理定时器 → _profile_commands_for_current_state (line 858)
        ▼ state == DO_READY
runtime.build_prepare_commands(profile_name, command_type) (runtime.py:141)
        ▼ 从 model_info.prepare_profiles 取固定姿态 → builder.build_from_profile
JointCommands（TYPE_PREPARE，只发一帧）
        ▼ 首发帧后 _prepare_frame_published=True，之后变成 TYPE_RUN 持续发布
```

### 3.2 指令数据流：RobotIntent 怎么让模型"切换动作"

第三方想让它做"挥手"动作，发一条 `RobotIntent`。回调 `_handle_active_intent`
（`rl_manager_node.py:365`）把它分发给 6 个子处理函数：

```
_handle_active_intent (line 365)
│  ├─ _handle_model_switch_intent (line 439)
│  │      intent_type=="model" && command=="switch_model"
│  │      → 校验：必须在 RL_ACTIVE、不能正在 stopped、
│  │        目标在允许范围(switch_rules) → _switch_to_model("t4_hello")
│  │
│  ├─ _handle_getup_switch_intent (line 387)   // config/getup 起身姿态
│  ├─ _handle_nav_switch_intent (line 409)     // config/nav 导航模式
│  ├─ _handle_cmd_vel_intent  (line 432)       // 提取手动速度
│  ├─ _handle_finish_loop_intent (line 491)    // model/finish_loop 结束循环
│  └─ _handle_motion_stop_intent (line 497)    // model/motion_stop 停止/恢复

_switch_to_model (line 508)  ★换模型的主路径
│  ├─ 保存旧模型最后一帧命令 → smoother.start_transition 平滑过渡
│  ├─ model_registry.switch(name)  换活动模型元数据
│  ├─ runtime.close() → 释放旧后端
│  ├─ 重建 runtime → load(新模型)（重新加载 onnx session）
│  └─ 新的推理定时器周期就用新模型算命令了
```

也注意 `_handle_robot_state`（`rl_manager_node.py:274`）：每次进入 `IDLE` 状态，
如果当前模型不是默认的 walk，会自动切回默认模型，防止一次性动作模型"残留"。

---

## 4. 关键概念速查

| 概念 | 含义 | 出现位置 |
|------|------|---------|
| PolicyContext | 一次推理的"输入包裹"，adapter 只跟它打交道 | `policy/context.py:18` |
| required_inputs | adapter 声明自己缺一不可的输入名 | `t4_walk.py:117` |
| obs / hist | walk 模型的 92 维当前观测 + 460 维 5 帧历史观测 | `t4_walk.py:170-171` |
| raw_action | 模型原始输出，27 维 | `onnx.py:111` |
| decode_action | 把动作向量翻译成关节目标位置 | `t4_walk.py:175` |
| joint_profiles | 每个关节的 kp/kd（模型不管控制增益，增益在 yaml 里） | `config/t4_walk.yaml:9` |
| TYPE_PREPARE / TYPE_RUN | 命令类型，前者只发一帧（如站立首帧） | `rl_manager_node.py:801` |
| has_input | 检查某个输入是否齐全 | `policy/context.py:65` |

---

## 5. 如果你想改点东西，从哪里下手

- **改模型文件/推理后端** → `policy/onnx.py` / `policy/torch.py` / `policy/pipeline.py`
- **加一个动作** → 新 yaml 注册进 `config/rl_models.yaml` + 新 adapter 文件
  `policy/adapters/xxx.py`（参考 `t4_heart.py` 这种一次性模型：`is_motion_end` 播完返回 True）
- **改观测特征怎么拼** → `t4_walk.py` 的 `build_model_input`
- **改动作怎么映射到关节** → 对应 adapter 的 `decode_action`
- **调整 kp/kd / 参考姿态** → 对应模型的 yaml `joint_profiles`
- **改状态→命令的分支逻辑** → `rl_manager_node.py` 的 `_profile_commands_for_current_state`

---

## 6. 建议的阅读顺序

1. `policy/context.py`（认识"交接单"）
2. `policy/adapter.py`（看接口 + 基类默认实现）
3. `policy/runtime.py` 的 `infer_commands`（看编排，这是主线）
4. `policy/onnx.py`（看模型怎么跑）
5. `policy/adapters/t4_walk.py`（看一个真实 adapter 怎么做语义翻译）
6. `rl_manager_node.py` 的 `_handle_inference_timer`（回到节点，看整条链怎么被调用）
7. `joint_command_builder.py`（最后看消息怎么拼出来）
