# robot_common

C++ 共享基础库。为所有其他 C++ 包提供关节配置加载、安全限幅、踝关节运动学变换和高频 QoS 配置。

## 功能模块

### 1. 关节配置加载（`joint_config.hpp`）

从 YAML 加载机器人关节定义：

```yaml
robot_joints:
  joints:
    - {name: left_hip_pitch, group: left_leg, enable_default: true, unit: rad, min_pos: -3.14, max_pos: 3.14, max_vel: 3.0, max_effort: 80.0}
    - {name: left_knee, group: left_leg, enable_default: true, unit: rad, min_pos: -3.14, max_pos: 3.14, max_vel: 3.0, max_effort: 80.0}
    # ...
```

**API**：
- `loadRobotJointsConfig(path)` → `RobotJointsConfig`（含有序向量 `joints` + 哈希映射 `joint_map`）
- `hasJoint(name)` / `getJoint(name)` / `jointNames()`
- 路径为空时自动从 CWD 向上搜索 `config/robot_joints.yaml`

每个关节必须显式配置 `group`、`enable_default`、`unit`、`min_pos`、`max_pos`、`max_vel`、`max_effort`；不再支持 defaults 兜底。

**校验规则**：非空、无重名、`min_pos < max_pos`、`max_vel > 0`、`max_effort > 0`。

### 2. 关节限幅（`joint_limits.hpp`）

纯计算安全钳位，不涉及 ROS 发布：

| 方法 | 行为 |
|------|------|
| `clampPosition(name, v)` | 钳位到 `[min_pos, max_pos]` |
| `clampVelocity(name, v)` | 钳位到 `[-max_vel, max_vel]` |
| `clampEffort(name, v)` | 钳位到 `[-max_effort, max_effort]` |
| `isPositionValid(name, v)` 等 | 返回 bool |

未知关节名抛出 `std::out_of_range`（不静默忽略）。

### 3. 踝关节运动学变换（`ankle_transform.hpp`）

实现串联空间 ↔ 并联空间转换。真机踝关节是并联机构，但 RL 模型在串联空间运行。每条腿 6 关节（hip_pitch/roll/yaw, knee, ankle_pitch/roll）。

```
串联角度 ←─FK─→ 笛卡尔位置 ←─IK─→ 并联角度
            (MATLAB 生成闭式解)     (Newton 迭代, max 20 次)
```

**核心类**：`FastAnkleFk`、`FastAnkleIk`（阻尼 Newton, tol=1e-5, damping=0.9）、`FastAnkleJacob`（解析闭式 Jacobian）

**高层模板函数**（header-only）：
- `convertToSeries<T>(vector<T>&)` — 并联→串联（状态路径）
- `convertToParallel<T>(vector<T>&)` — 串联→并联（命令路径）
- 自动检测 `T` 是否有 kp/kd 字段（通过 `has_kp_kd_v<T>`）

**全量变换**：位置（FK/IK）+ 速度（`J·v` 或 `pinv(J)·v`）+ 力矩（`J^{-T}·τ` 或 `J^T·τ`，功率守恒）+ Kp/Kd（对角近似）

**工作空间约束**：
- 串联：pitch ∈ [-0.87, 0.51], roll ∈ [-0.26, 0.26]
- 并联：sum ∈ [-0.36, 0.36], diff ∈ [-1.2, 2.2]
- 往返精度：~90% 点误差 < 1e-5 rad；~7.6% 边界点最大误差 ~0.2 rad（并联机构固有约束）

**力矩、速度与惯量折算（补充）**：

雅可比 `J = ∂t/∂f`（并联位置对串联角度）随姿态变化。以下系数取站立姿态（pitch=-0.2, roll=0）的 `J ≈ [-1.198, 0.700; 1.201, 0.694]`（与代码中 kp/kd 固定系数同一姿态），数值由闭式 Jacobian 验证；全工作空间内变化不大。

三者共用一套功率一致映射：**力矩经 Jᵀ**、**速度经 J⁻¹**、**惯量经 Jᵀ(·)J**。

J 的列结构决定了两个轴的特性（站立姿态）：

```
列1 (pitch) = [-1.198, +1.201]   两执行器反向 → 差动轴：力矩可叠加，速度被抵消
列2 (roll)  = [+0.700, +0.694]   两执行器同向 → 共模轴：速度可叠加，力矩不放大
```

对应并联空间约束：`sum ∈ [-0.36, 0.36]` 为共模（roll）行程，`diff ∈ [-1.2, 2.2]` 为差动（pitch）行程。

- **力矩映射**（功率守恒）：`τ_parallel = J⁻ᵀ·τ_serial`（S2P）、`τ_serial = Jᵀ·τ_parallel`（P2S）。
  设两电机输出端最大扭矩为 tor1、tor2，串联两关节单轴可达到的最大输出力矩（两电机同号叠加）：

  ```
  tor_pitch_max = |J11|·tor1 + |J21|·tor2 ≈ 1.198·tor1 + 1.202·tor2
  tor_roll_max  = |J12|·tor1 + |J22|·tor2 ≈ 0.700·tor1 + 0.694·tor2
  ```

  tor1 = tor2 = tor 时：tor_pitch_max ≈ 2.40·tor，tor_roll_max ≈ 1.39·tor。
  两轴最大值不可同时达到；要求两轴同时输出 τ_serial 时，需满足电机约束 |J⁻ᵀ·τ_serial| ≤ [tor1, tor2]。

- **速度映射**（几何关系）：`ṫ_parallel = J·ḟ_serial`（S2P）、`ḟ_serial = J⁻¹·ṫ_parallel`（P2S，代码用 `PseudoInverse(J)`）。
  设两电机输出端最大转速为 w1、w2，串联两关节单轴可达到的最大角速度（两电机同号叠加），站立姿态 `J⁻¹ ≈ [-0.4149, 0.4186; 0.7182, 0.7163]`：

  ```
  pitcḣ_max = |J⁻¹11|·w1 + |J⁻¹12|·w2 ≈ 0.415·w1 + 0.419·w2
  roll̇_max  = |J⁻¹21|·w1 + |J⁻¹22|·w2 ≈ 0.718·w1 + 0.716·w2
  ```

  w1 = w2 = w 时：pitcḣ_max ≈ 0.833·w，roll̇_max ≈ 1.435·w。
  工作空间扫描（6615 个有效位形）：pitch 0.61~0.85·w，roll 1.08~1.51·w。
  两轴最大值同样不可同时达到；要求同时输出 ḟ_serial 时，需满足电机约束 |J·ḟ_serial| ≤ [w1, w2]。

- **力矩–速度对偶**：同一组符号选择下严格满足 `τ_parallel·ṫ = τ_serial·ḟ`（功率守恒）。等速/等扭矩时的系数乘积 ≈ 执行器数量 2：

  | 轴 | 力矩系数（Jᵀ） | 速度系数（J⁻¹） | 乘积 |
  |----|----|----|----|
  | pitch（差动） | 2.40 | 0.833 | 2.00 |
  | roll（共模） | 1.39 | 1.435 | 1.99 |

  全工作空间乘积范围 1.996~2.103。即 pitch 力矩强、速度慢，roll 速度快、力矩弱。
  注意串联空间逐关节 max_vel 限幅与并联侧电机限幅**不等价**，跨空间下发速度命令时需按上式校验。

- **转子惯量折算**（动能守恒）：设两电机转子惯量（含减速器，折算到执行器输出端）为 I1、I2，折算到串联关节的等效惯量矩阵 `M_ref = Jᵀ·diag(I1, I2)·J`：

  ```
  I_pitch = J11²·I1 + J21²·I2 ≈ 1.4357·I1 + 1.4435·I2
  I_roll  = J12²·I1 + J22²·I2 ≈ 0.4902·I1 + 0.4817·I2
  I_cross = J11·J12·I1 + J21·J22·I2 ≈ −0.8389·I1 + 0.8338·I2
  ```

  I1 = I2 = I_rot 时：I_pitch ≈ 2.879·I_rot，I_roll ≈ 0.972·I_rot，I_cross ≈ −0.005·I_rot（站立姿态近似解耦）。
  惯量按 J² 叠加，区别于力矩按 |J| 线性叠加（2.879 vs 2.40）。
  转子加速对串联输出力矩的动态扣减：`τ_serial = Jᵀ·τ_parallel − (Jᵀ·I_rot·J)·f̈ − Jᵀ·I_rot·J̇·ḟ`（上一行公式为静态上限）。
  右腿镜像：对角项幅值不变，耦合项变号。

### 4. 高频 QoS（`high_frequency_qos.hpp`）

```cpp
rclcpp::QoS highFrequencyControlQoS()
```

- `KeepLast(1)` + `VOLATILE` + `RELIABLE`
- 无 deadline/lifespan（兼容仿真和 SystemDefaultsQoS 节点）
- 用于控制管线话题：`rl_manager/desired_commands`、`router/accepted_commands`、`control/hardware_commands`、`driver/joint_states`

## 消费者

以下包依赖 `robot_common`：

| 包 | 使用的头文件 |
|----|-------------|
| `robot_state` | `joint_config.hpp` |
| `robot_control` | `joint_config.hpp`, `high_frequency_qos.hpp`, `ankle_transform.hpp` |
| `robot_router` | `joint_config.hpp`, `high_frequency_qos.hpp` |
| `robot_driver` | `joint_config.hpp`, `high_frequency_qos.hpp`, `yaml_loader.hpp`, `ankle_transform.hpp` |
| `robot_monitor` | `high_frequency_qos.hpp` |
| `robot_recorder` | `high_frequency_qos.hpp` |
| `robot_injector` | `high_frequency_qos.hpp` |

## 构建

```bash
colcon build --packages-select robot_common
```

- C++17，`-Wall -Wextra -Wpedantic`
- 依赖：`rclcpp`、`yaml-cpp`、`Eigen3`
- 导出 CMake target 供下游 `find_package(robot_common)` 使用
