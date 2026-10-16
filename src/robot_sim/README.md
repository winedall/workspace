# robot_sim

zl_robot 的 MuJoCo 仿真工作区。在无硬件环境下替代真实 IMU（`yesense_node`）和驱动（`robot_driver_node`）节点，对外发布完全相同的话题，支持控制管线的闭环测试与 RL 策略训练。

## 依赖仓库

| 仓库 | 内容 | 关系 |
|------|------|------|
| `src/robot_assets` | 机器人模型资产（MuJoCo XML / URDF / STL 网格） | 与 robot_sim 同级，独立 clone |

`mj_sim` 构建时通过 CMake 变量 `ROBOT_ASSETS_DIR`（默认 `../../robot_assets`）定位模型资产，并将 XML 安装到 `share/mj_sim/description`，仿真包通过 `FindPackageShare('mj_sim')` 解析模型路径，不依赖源码目录。

## 包结构

```
robot_sim
├── mj_sim                      基础库：MuJoCo 引擎 + 渲染 + 自动代码生成
├── sim_common                  共享仿真基类（SimNodeBase / MjInterface / 控制数学）
├── sim_d015_poc                D015（D2）四足，12 关节
├── sim_d040_poc                D040（D4）四足，12 关节
├── sim_z01_edm                 Z01 工程样机人形，29 关节（无头部执行器）
├── sim_z01_edm_add_head        Z01 工程样机人形，带头部执行器 + 相机仿真 ★ bringup 默认
├── sim_z01_edm_parallel_ankle  Z01 工程样机人形，并联踝关节，29 关节
├── sim_z01_std                 Z01_std 量产版人形，29 关节
├── sim_z01_std_add_head        Z01_std 量产版人形，带头部执行器 + 相机仿真
├── sim_z01_std_parallel_ankle  Z01_std 量产版人形，并联踝关节，29 关节
├── sim_z01_std_football_player Z01_std 量产版人形（并联踝）+ 足球场/球门/足球（单机）
└── sim_z01_std_robocup         RoboCup 多机器人：一个模型里 6 台并联踝人形 + 足球场
```

`_std`（量产版）与 `_edm`（工程样机）系列共用仿真框架，差异仅在模型资产与关节配置。

## 架构

每个仿真包统一遵循三层结构：

```
Sim<Name> : sim_common::SimNodeBase          ← 机器人特有逻辑
   └── SimNodeBase = MujocoSimBase + SimRosNodeBase
        ├── MujocoSimBase   物理引擎 + GLFW 渲染循环 + 生命周期
        └── SimRosNodeBase  ROS 2 通信层（话题、命令缓存、发布节奏）
```

- 基类已实现：`Start/Run` 线程样板、初始位姿、模型接口初始化、跟踪相机（含深度相机）、200 Hz JointStates/IMU 发布节奏、相机图像发布；
- 派生类只需实现 **`ComputeControl()`**（每个仿真步写入 `d->ctrl`），并按需重写 `HandleInterfaceReady()`（模型加载后的附加查表）与 `PublishExtra()`（200 Hz 发布块内的附加发布）。

数据流闭环：

```
robot_control ──/robot/control/hardware_commands──▶ 仿真节点
                                                      │ boost::lockfree::spsc_queue（容量 10）
                                                      ▼
                                             命令缓存 ─▶ ComputeControl()
                                                      │ torque = effort + kp·Δpos + kd·Δvel
                                                      │ （限幅 + 电机 TN 曲线限制）
  仿真节点 ──/robot/driver/joint_states (~200 Hz)──▶ robot_state / robot_control
          ──/robot/imu/nav_all (~200 Hz)──────────▶ 上层控制
          ──/robot/driver/status (5 Hz)───────────▶ 状态监控
```

## 快速开始

### 构建

```bash
cd zl_robot
colcon build --packages-select mj_sim sim_common sim_z01_edm_add_head   # 或按需选择
source install/setup.bash
```

MuJoCo / GLFW / Eigen 均以 vendored 形式内置在 `mj_sim/thirdparty/`，CMake 自动按 x86_64 / aarch64 选择二进制，无需系统级安装。

### 启动

```bash
# 通过 bringup 启动（推荐，与真机同一条控制管线）
ros2 launch robot_bringup zl_robot.launch.py enable_mujoco_sim:=true robot_type:=humanoid
ros2 launch robot_bringup zl_robot.launch.py enable_mujoco_sim:=true robot_type:=dog
ros2 launch robot_bringup zl_robot.launch.py enable_mujoco_sim:=true ankle_env_parallel:=true   # 并联踝

# 独立启动单个仿真（仅渲染 + 话题，不带控制管线）
ros2 launch sim_z01_edm_add_head sim_z01_edm_add_head_launch.py
ros2 launch sim_z01_std sim_z01_std_launch.py
ros2 launch sim_d040_poc sim_d040_poc_launch.py
ros2 launch sim_d015_poc sim_d015_poc_launch.py
```

`robot_type` 区分形态（`humanoid` / `dog`）；人形仿真的踝关节空间由 `ankle_env_parallel` 显式选择（`true` → `sim_z01_std_parallel_ankle`，`false` → `sim_z01_std`）。

## ROS 接口

所有仿真节点订阅/发布：

| 话题 | 类型 | 频率 | 方向 |
|------|------|------|------|
| `/robot/control/hardware_commands` | `JointCommands` | — | 订阅 |
| `/robot/driver/joint_states` | `JointStates` | ~200 Hz | 发布 |
| `/robot/driver/status` | `DriverStatus` | 5 Hz | 发布 |
| `/robot/imu/nav_all` | `NavAll` | ~200 Hz | 发布 |

仅 `*_add_head` 包额外发布相机话题：

| 话题 | 类型 |
|------|------|
| `/robot/camera/color/image_raw` | `sensor_msgs/Image` |
| `/robot/camera/color/camera_info` | `sensor_msgs/CameraInfo` |
| `/robot/camera/depth/image_rect_raw` | `sensor_msgs/Image` |
| `/robot/camera/depth/camera_info` | `sensor_msgs/CameraInfo` |

## 机器人模型

模型资产位于 `robot_assets/<模型>/`（XML + URDF + STL 网格）：

| 模型 | 类型 | 关节数 | 对应仿真包 |
|------|------|--------|-----------|
| D015_poc | 四足 | 12 | sim_d015_poc |
| D040_poc | 四足 | 12 | sim_d040_poc |
| Z01_edm | 人形 | 29 | sim_z01_edm |
| Z01_edm_add_head | 人形 | 29（含头部执行器） | sim_z01_edm_add_head |
| Z01_edm_parallel_ankle | 人形 | 29（并联踝） | sim_z01_edm_parallel_ankle |
| Z01_std | 人形 | 29 | sim_z01_std |
| Z01_std_add_head | 人形 | 29（含头部执行器） | sim_z01_std_add_head |
| Z01_std_parallel_ankle | 人形 | 29（并联踝） | sim_z01_std_parallel_ankle |
| Z01_std_football_player | 人形 | 29（并联踝）+ 足球场 | sim_z01_std_football_player |
| Z01_std_robocup | 人形 ×6（并联踝）+ 足球场 | 29 ×6 | sim_z01_std_robocup |

### 多机器人（sim_z01_std_robocup）

`sim_z01_std_robocup` 在一个 MuJoCo 模型里放 6 台机器人，因此话题按机器人前缀区分：
`/robot_01/...` … `/robot_06/...`（结构与单机表一致），另有世界系的全局话题
`/sim/ball_pose`、`/sim/ball_twist`、`/sim/goal_team_a|b`，以及重载/复位服务
`/reload_robot_0X`、`/reload_all_robot`（完整回到初始状态）、`/reset_robot_0X`、`/reset_all_robot`
（只挪基座、关节状态保持不变）与 `/reset_ball`。6 套控制栈需要按话题逐条 remap
（ROS 2 的 remap 是整名匹配，`-r /robot:=/robot_03` 不生效），完整配方见
`sim_z01_std_robocup/README.md`。

## 线程模型

每个仿真节点使用三线程：

1. **主线程**：MuJoCo `RenderLoop()`（GLFW GUI 窗口，可 headless 运行）
2. **物理线程**：MuJoCo 物理步进 + 实时倍率同步（仿真时间 vs CPU 时间偏差超过阈值时重新对齐），步进前调用消息交换
3. **ROS 线程**：`SingleThreadedExecutor` 的 `spin_once` 循环，与物理线程解耦，避免 spin 阻塞仿真；收到 SIGINT 后同步通知 GUI 循环退出

## 仿真参数（YAML 分组）

各仿真包的 `config/*.yaml` 按**影响面**分组，避免所有参数堆在同一层。新增参数都不改变模型本身（模型量仍走 XML / `robot_assets`）：

| 分组 | 只影响 | 参数 |
|------|--------|------|
| `viewer.*` | 窗口与界面显示；不碰物理，也不碰任何话题 | `ui0_enable` `ui1_enable` `spacing` `color` `font` `help` `info` `profiler` `sensor` `vsync` `pause_update` |
| `dynamics.*` | 物理线程的推进节奏与运行状态；不改变模型 | `start_paused` `busywait` `refresh_rate` `realtime_percent` |
| `errors.*` | 传感器读数与执行/反馈时序；不改变模型的物理参数 | `seed` `joint_zero_offset_std_rad` / `joint_zero_offset_rad` `joint_pos_noise_std_rad` `joint_vel_noise_std_rad_s` `mount_std_deg` `gyro_bias_std_rad_s` `gyro_noise_std_rad_s` `acc_bias_std_m_s2` `acc_noise_std_m_s2` `att_bias_std_deg` `att_noise_std_deg` `command_delay_ms` `torque_delay_ms` `drop_probability` `feedback_hold_ms` `feedback_delay_ms` |

- 参数名与默认值只在 `sim_common/include/sim_common/SimRuntimeOptions.hpp` 定义一处，单机（`SimNodeBase`）与多机（`SimZ01StdRobocup` 主节点）共用，避免两份 YAML 键名漂移。
- 写入时机：`viewer.*` 与除 `realtime_percent` 外的 `dynamics.*` 由 `MujocoSimBase::ApplyStartupOptions()` 在 `RenderLoop()` 之前写入 `mj::Simulate`（`spacing`/`color`/`font`/`vsync` 由 RenderLoop 启动时一次性读入，晚了不生效）；`dynamics.realtime_percent` 由 `ApplyPostLoadOptions()` 在模型加载后写入——`LoadOnRenderThread` 会先按 XML 的 `vis/global realtime` 自动选档，必须排在它后面才能覆盖。
- 特殊值：`viewer.font: -1` = 按屏幕 DPI 自动推断；`dynamics.realtime_percent: 0` = 用 XML 的 `vis/global realtime`。
- 窗口相机：`camera_index`（0 = 自由，1 = 跟踪基座（默认），≥2 = 模型里第 `index-2` 个固定相机）与 `camera_distance`/`camera_azimuth`/`camera_elevation` 一起平铺在顶层，只对单机包生效；多机包用已有的 `track_robot_index` 选俯瞰 / 跟踪某台。
- 尚未归组的平铺参数（`state_publish_period_us`、`driver_status_period_ms`、`camera_period_us`、`base_z` 及各包自己的场景参数）后续可按同样方式迁移到 `ros.*` 等分组——改名要同时动 YAML 与 launch，故单独一步做。

### 误差注入（`errors.*`）

`errors.seed: 0`（默认）表示**完全关闭**，行为与不注入逐位一致；给一个非 0 种子即开启，常值误差（零位偏置 / IMU 零偏 / 安装角 / 姿态误差）在**模型加载时抽一次**，GUI Reload 后仍是同一台"个体"。多机按机器人序号派生种子，6 台的误差各不相同。

**语义（与真机一致）**：注入的是"编码器 + 驱动器"这一层，误差**进入仿真内部的 PD 回路**。所以零位标定错时，关节**真值**会偏离指令（`q_true ≈ q_des − 零位偏置`），而**对外发布的关节角等于指令值**——这正是真机上看到的现象。若只想让观测带噪声、不影响物理，请把误差加在控制侧而不是这里。

- **注入点只有两处**：`MjInterface::ReadJointState` / `ReadState`。它们是 `q_rel_/dq_rel_/acc_` 的唯一来源，发布路径与各包 `ComputeControl` 的 PD 路径都从这里取值，所以一处注入同时覆盖"发布值"与"控制回路"，10 个仿真包无需改动。
- **白噪声按仿真时间键控**：每个仿真步抽一次并缓存，同一步内 PD 与发布读到同一个采样；暂停时反复 `mj_forward` 不会白吃随机数。同 seed + 同 `timestep` 的两次运行逐位可复现。
- **两条延迟落在完全不同的位置，别混用**：

  | 参数 | 延迟什么 | 落在哪 | 合理量级 |
  |---|---|---|---|
  | `command_delay_ms` | 整份**指令帧**（目标位置/速度/前馈/增益） | **参考路径（回路外）** | 几十毫秒 |
  | `torque_delay_ms` | 算完的**力矩** `d->ctrl` | **回路内** | 亚毫秒~几毫秒 |

  真机是 `τ = kp·(q_target(t−Δ) − q_enc(t)) + kd·(v_target(t−Δ) − v_enc(t)) + τ_ff(t−Δ)`——驱动器收到的是 Δ 之前的目标，但它用**自己本地的编码器**闭合 PD（协议帧里发的是 kp/kd/目标位置/目标速度/前馈力矩，见 `tienkung_protocol.cpp`）。所以实测的"指令发布 → 力矩体现" 25~40 ms 全部落在**参考路径**上，延迟在回路外，只让跟踪滞后、不消耗相位裕度——这也是真机带着 30 ms 还能走稳的原因。

  `torque_delay_ms` 延迟的是算完的力矩，等于把阻尼项 `kd·q̇` 也一起延迟（延迟阻尼），**直接吃相位裕度**。给它配到十几毫秒，简单关节控制就会抖甚至发散；配 1 ms 时注意 `dt=2ms` 会把 `llround(0.5)` 取整成**一个仿真步 = 2 ms**（启动日志会打印实际生效值）。**想补实测延迟请用 `command_delay_ms`。**

  - `command_delay_ms` 作用在 `SimRosNodeBase::UpdateCommandCache`——所有仿真包唯一共用的指令写入点，一处覆盖全部 10 个单机包 + 多机。它按**仿真时间**索引，不取整到仿真步（指令是按时间到达的，与 `timestep` 无关）。
  - `torque_delay_ms` 由 `MujocoSimBase::PostComputeControl` 在 `d->ctrl` 写好后、MuJoCo 读取前施加，按**整数个仿真步**取整。
- **`feedback_hold_ms` / `feedback_delay_ms` 是反馈上行**：真机上驱动以固定节拍发 `JointStates`，但底层电机快照刷新得更慢（2026-09-23 实测四次录制为 86.8 / 151.5 / 197.2 / 199.4 Hz，而发布固定 200 Hz），表现为**整帧数值逐位重复**——实测 56.6% 的帧与前一行完全相同，且相邻两拍就变的占比为 0.0%（31 个关节、425 秒内 4 万次变化无一例外）。`feedback_hold_ms` 复现这个采样保持周期，`feedback_delay_ms` 复现反馈链路的纯传输延迟。
  - **只改发布出去的那份快照**，`q_rel_/dq_rel_/tau_rel_` 保持新鲜：PD 回路不受影响（真机上驱动用自己的编码器闭环，上报给上位机的那份才是慢的）。`/robot/imu/nav_all` 也仍是新鲜值——真机 IMU 与关节反馈是两路独立传感器。
  - 语义：上报值在保持网格上更新，网格相位**自由漂移**（`next_sample_time_ += hold` 累加），所以实测的 `10ms/15ms` 混合刷新间隔能复现，而不是被发布栅格锁成恒定 `15ms`。
  - `feedback_hold_ms` 必须**大于** `state_publish_period_us`（默认 5ms）才有意义，否则每拍都刷新、保持不生效（运行时会打印一次 WARN）。
  - 与 `command_delay_ms`/`torque_delay_ms` 一样，**`seed=0` 时也不会生效**。
  - 记账：观测的**平均**滞后 ≈ `feedback_delay_ms + feedback_hold_ms/2`（保持本身已隐含半个周期）。标定时只用其中一个旋钮去拟合同一个观测滞后，两个一起给会重复计数。
- **`seed=0` 时所有时序参数（`command_delay_ms`/`torque_delay_ms`/`drop_probability`/`feedback_*`）都不会生效**（整体关闭）。只想加时序：设 `seed: 1`，各误差标准差留 0，只给时序参数。
- **验证必须走话题**：查看器的曲线/历史读的是 `mjData` 真值，`feedback_*` 只作用于 ROS 话题，GUI 里看不到效果。另外 `header.stamp` 仍是发布时刻的墙钟、`sequence` 每帧自增——"序号每帧都变、载荷逐位重复"正是真机的形态，故意保持这样。
- **误差只影响传感器读数**：足球/球门位置、出生点、`/reset_*` 等直接读 `mjData` 真值的路径不受影响；`/robot/imu/nav_all` 的 `pos`/`vel` 也是真值出口。`JointStates.effort` 是延迟后的力矩（真机上驱动回报的也是当前施加量），做阶跃标定时注意它与命令之间稳定相差 N 步。
- **明确不建模**：零偏无随机游走/温漂，无编码器量化，无反馈丢帧，不修改 `mjModel` 物理参数（质量/质心/摩擦）。多机若把积分器改成 RK4，控制回调每步会触发 4 次，实际延迟会变成 4 倍——`SimErrorModel` 检测到步长与 `timestep` 不符时会打印一次 WARN。
- **打开 `feedback_*` 后策略变差是预期行为**：这个改动不改变仿真内部 PD 环（关节真值稳定性不变），只是让外部策略/RL 看到的反馈变旧——真机上本来就是这样。默认 0 保持改造前的行为。
- `fullscreen` 没有做进 YAML：`Simulate::fullscreen` 只被 GUI 写入、无读取点（要生效需真去调 `platform_ui->ToggleFullscreen()`），窗口化/全屏仍走 GUI 的 F5。

## 仿真 vs 真机切换

由 bringup 的 `enable_mujoco_sim` 参数控制：

| `enable_mujoco_sim` | IMU | 驱动 | 关节配置 |
|---------------------|-----|------|----------|
| `false`（默认） | 真实 yesense | robot_driver | 真机配置 |
| `true` | 仿真虚拟 IMU | 仿真节点 | 仿真配置 |

仿真节点完全替代 `robot_driver_node` 和 `yesense_node`，对外发布相同的 `JointStates`、`DriverStatus` 和 `NavAll` 话题，上层控制管线无需感知差异。

## 自动代码生成

`robot_assets/utils/header_generator.py` 解析 MuJoCo XML，自动生成 `mj_sim/include/robot_header/<模型>.h`，包含：

- `kJointsName` / `kActuatorsName` — 关节/执行器名称数组
- `kJointsNum` — 关节数量
- `kJointsMaxTorque` / `kJointsMinTorque` — 力矩限幅
- 传感器名称、相机信息、基座名称
- Eigen 类型别名（`SystemQ`、`SystemDq`、`SystemTau` 等）

CMake 构建 `mj_sim` 时自动扫描 `robot_assets/*/xml/*.xml` 并触发生成，XML 是唯一真源。

## 添加新机器人

1. 在 `robot_assets/<NewRobot>/` 添加 XML（及可选 URDF、STL 网格）
2. 构建 `mj_sim`，自动生成 `robot_header/<NewRobot>.h`
3. 以 `sim_z01_std` 为模板创建仿真包，实现 `ComputeControl()`
4. 在 `robot_bringup` 的 `zl_robot.launch.py` 注册 `robot_type` 分支
5. 确保存在对应的关节配置 YAML

## 关键设计细节

- **头部关节占位**：`sim_z01_edm` 的模型没有头部执行器，在 JointStates 的索引 0/1 发布零填充的 `head_yaw`/`head_pitch` 占位条目，保证对外消息布局与真机一致。
- **并联踝关节空间**：`*_parallel_ankle` 包发布并联空间（主动电机位置）的踝关节状态，通过 bringup 的 `ankle_env_parallel:=true` 告知 `robot_control` 环境处于并联空间，避免二次串并联转换。
- **控制律**：`torque = effort + kp·(q_des - q) + kd·(dq_des - dq)`，经力矩限幅与电机 TN 曲线物理限制（`motor_spec`）后写入执行器。
- **状态向量约定**：`SystemQ` 前 7 维为浮动基座位姿，关节从索引 7 开始；`SystemDq`/`SystemTau` 前 6 维为基座速度/角速度，关节从索引 6 开始。
- **相机仿真**：`*_add_head` 包在 MuJoCo 中渲染 RGB/深度图像并发布标准 ROS 图像话题，可用作上层视觉链路的输入。
- **GUI 模型热重载**：仿真运行中可通过 MuJoCo 自带文件选择器重载 XML，自动重建 `mjModel`/`mjData` 并重新初始化状态与索引映射。
- **数据录制**：`record_folder` 参数指定 CSV 日志目录（默认 `logs/mj_sim`），独立启动时将仿真数据写入该目录。
- **可调参数**：发布频率、初始基座高度、相机几何、窗口显示与物理推进节奏等常量位于各仿真包自己的 `config/*.yaml`，经 launch 文件传入；分组方式见[仿真参数（YAML 分组）](#仿真参数yaml-分组)。

更多实现细节（vendored 依赖版本、模块划分）见 `mj_sim/README.md`。
