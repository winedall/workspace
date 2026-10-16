# robot_odometry

基于本体感觉的 RL 里程计推理节点。订阅硬件关节状态与 IMU，运行一个 50 帧
历史窗口的 MLP，输出机器人躯干在世界系下的位移增量，并积分为累计位姿。

本包是独立进程，不参与控制链路：不发布关节命令、不改变任何现有节点的行为，
也不依赖任何节点的话题来驱动推理节奏。

## 快速开始

```bash
colcon build --packages-select robot_odometry robot_bringup
# bringup 已包含本节点（enable_odometry 默认 true）

# 单独调试 / 换模型
ros2 launch robot_odometry odometry.launch.py
ros2 launch robot_odometry odometry.launch.py model_name:=z01_std_odom_no_acc
ros2 run robot_odometry robot_odometry_node --ros-args -p model_name:=z01_std_odom_no_acc

# 手动重置（清空模型状态并把累计位姿归零）
ros2 topic pub --once /robot/odometry/reset std_msgs/msg/Empty "{}"

# 观察输出
ros2 topic echo /robot/odometry/delta
ros2 topic echo /robot/odometry/odom
```

## 架构

```
                          config/odometry_models.yaml   ← 模型注册表
                                    │  model_name
                                    ▼
    /robot/control/hardware_states ─┐   ┌──────────────┐
    /robot/imu/nav_all            ─┼──→│   adapter    │→ obs[88|91]
                                   │   └──────────────┘        │
      50Hz 定时器（本节点自驱）──────┘                           ▼
                                              OnlineOdometry（50 帧窗口 + 自回归）
                                                            │
                                                OnnxOdometryBackend（onnxruntime）
                                                            │
                                                  δ[dx, dy]（世界系，米）
                                                            │
                                        ┌───────────────────┴──────────────┐
                                        ▼                                  ▼
                          /robot/odometry/delta                /robot/odometry/odom
                          (Vector3Stamped)                     (nav_msgs/Odometry)
                                                                        │
                                                            CSV「真值 vs 估计」对比
```

节点本身不关心观测由哪些字段拼成：那是 adapter 的职责。接入新的输入变体
只需要加一个 adapter 文件 + 一份每模型配置，节点代码不动。

## 模型选择

三级优先级：launch 参数 `model_name` → 节点参数 `model_name` → 注册表的
`default_model_name`。

注册表 `config/odometry_models.yaml` 与 robot_rl_manager 的 `rl_models.yaml`
同构：每项三个键，adapter 类名按文件名 PascalCase + `OdometryAdapter` 约定，
参数放在每模型配置的 `model:` 块下。

```yaml
default_model_name: "z01_std_odom_with_acc"
models:
  z01_std_odom_no_acc:
    config_path:  "config/z01_std_odom_no_acc.yaml"   # 每模型参数（model: 块）
    model_path:   "model/z01_std_odom_no_acc.onnx"    # ONNX 图
    adapter_path: "adapters/z01_std_odom_no_acc.py"   # 输入适配器
```

| 模型 | 观测维度 | 说明 |
|---|---|---|
| `z01_std_odom_no_acc` | 88 | 不含基座加速度 |
| `z01_std_odom_with_acc` | 91 | **默认**，含基座加速度 |

**每个模型一个 adapter 文件**，读那个文件就知道该模型的输入构成。
接入新模型 = 放图 + 写一份配置 + 写一个 adapter，节点代码不动。

两个包的区别只有两点，都是模型本身决定的：

- rl_manager 的 `ModelInput` 是**具名字典**（`obs`/`hist`/`depth`…），靠输入名
  与图比对来保证契约；里程计的输入名固定、只有维度不同，因此靠
  `obs_layout` 字符串保证——每模型配置里它是**必填**项，注册表加载时与代码里
  的布局逐字比对。复制一份配置来加新模型时，这里是唯一会暴露语义错配的关卡。
- 里程计的 `type` 目前只有 `onnx`。

模型接口与观测布局见 [model/README.md](model/README.md)。

## 触发与节奏

本节点**自驱一个定时器**，频率取自模型配置的 `control_frequency_hz`，这是
唯一的推理触发源。输入话题只提供数据——谁在什么频率上发布都不影响推理节奏。

模型的训练前提是固定 50 Hz（`step_dt = 0.02`，50 帧 = 1 秒），历史窗口的物理
时长等于 `50 / 频率` 秒。换频率就换了窗口含义，因此注册表加载时会校验
`control_frequency_hz` 与模型训练步长一致，不一致直接报错。

## 行走门控

只有 `RobotState == STATE_RL_ACTIVE` 时才做估计与积分。其它状态：

- 不运行模型，清空模型与 adapter 的帧间状态
- 累计位姿归零，并按 `idle_publish_rate_hz`（默认 10 Hz）持续发布零值

状态切换的瞬间立即处理：进入 `RL_ACTIVE` 时重置状态（新一轮从复制窗口起步、
原点归零、`joint_pos_previous` 回到自引用）；离开时立刻输出一次零值。

门控不只是省算力——模型在越界输入下自回归会放大，静止/非行走姿态离训练
分布较远，关掉比让它跑更安全。

此外还有一层发散保护：单步位移超过 `max_step_delta_m`（默认 0.2 m，折合
10 m/s）时丢弃本步并重置，避免一次异常输入污染后续整个窗口。

## ROS 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `models_config` | string | `""` | 注册表路径，空则用包内 `config/odometry_models.yaml` |
| `model_name` | string | `""` | 使用的模型名，空则取注册表默认值 |
| `onnx_intra_op_num_threads` | int | `1` | onnxruntime 线程数 |
| `onnx_inter_op_num_threads` | int | `1` | onnxruntime 线程数 |
| `joint_states_topic` | string | `/robot/control/hardware_states` | 关节反馈 |
| `imu_topic` | string | `/robot/imu/nav_all` | IMU（姿态/角速度/加速度/仿真真值） |
| `state_topic` | string | `/robot/state` | 行走门控依据 |
| `odom_topic` | string | `/robot/odometry/odom` | 累计位姿输出 |
| `delta_topic` | string | `/robot/odometry/delta` | 单步增量输出 |
| `reset_topic` | string | `/robot/odometry/reset` | 手动重置 |
| `odom_frame_id` | string | `odom` | 父坐标系名 |
| `base_frame_id` | string | `base_link` | 子坐标系名 |
| `input_timeout_sec` | float | `0.1` | 关节/IMU 超时阈值 |
| `reset_gap_sec` | float | `0.1` | 控制流中断判定阈值 |
| `idle_publish_rate_hz` | float | `10.0` | 非行走状态零值输出频率 |
| `max_step_delta_m` | float | `0.2` | 单步位移发散阈值 |
| `csv_enabled` | bool | `true` | 是否记录对比 CSV |
| `csv_dir` | string | `""` | CSV 输出目录，空则按下面优先级解析 |

## 输出

`/robot/odometry/delta`（`geometry_msgs/Vector3Stamped`）：
世界系单步位移增量，`x`/`y` 单位米，`z` 恒为 0。

`/robot/odometry/odom`（`nav_msgs/Odometry`）：
`pose` 为 2D 积分结果（`z` 恒为 0，姿态只保留偏航，取自 IMU 相对本次运行
起点的偏航角——模型本身不预测姿态）；`twist` 为增量按标称步长换算的世界系
线速度与 IMU 角速度。`covariance` 未做估计，保持全 0。

## CSV 对比记录

每个估计步写一行，列顺序：

```
t, step, gt_x, gt_y, gt_dx, gt_dy, est_x, est_y, est_dx, est_dy, gt_abs_x, gt_abs_y, gt_abs_z
```

- `gt_*`：真值位置，取自仿真填进 `NavAll.pos` 的 MuJoCo 基座位置。
  相对列以本次运行起点为原点，**与估计值共用同一个原点**，
  因此 `gt_x - est_x` 可直接当位置误差用
- `est_*`：里程计输出
- `gt_abs_*`：仿真世界系绝对真值（含初始高度），便于与其它工具对齐
- `t` 自节点启动起算、单调递增，运行之间的间隔体现为跳变

输出目录优先级：`csv_dir` → `$ZL_ROBOT_RECORDER_DIR/odometry`（跟随 run.sh
的 run 目录）→ `./logs/odometry`。文件名 `odometry_<RUN_ID>.csv`。

真机运行时可把 `csv_enabled` 置 false（`NavAll.pos` 在真机上是经纬高，不是
位置真值，记录出来没有意义）。

## 前置条件

- 机器人平台为 **T4-27DOF** 同构（27 个受控关节，含 `waist_yaw`、不含
  `waist_pitch`/`waist_roll`）。Z01_edm_add_head 的 27 个非头部关节名字与
  顺序与之完全一致，可直接使用
- 控制频率 50 Hz
- IMU 需提供躯干系姿态、角速度，以及（91 维模型）body 系线加速度，
  且轴向与 MuJoCo 模型一致

## 测试

```bash
colcon test --packages-select robot_odometry
```

覆盖：关节顺序与观测布局、adapter 帧间语义、注册表与配置校验、ONNX 契约、
逐模型加载与前向、反归一化仿射关系、几何工具，以及行走门控/积分/重置/
发散保护/CSV 记录的节点级行为。

## 已知限制

- 模型训练于 T4-27DOF 平地速度跟踪任务，未覆盖跳跃/翻转等大幅动作。
- 世界系参考与 IMU 航向参考存在任意常值偏航差。常值差无害（整条轨迹随之
  旋转，积分原点本就任意），但 AHRS 航向漂移会让预测的位移方向随时间旋转，
  这是本方案的固有限制。
- 模型输入 `base_quat` 在仿真中是真值姿态。实机使用 AHRS 姿态，其轴向与
  安装方向必须与 MuJoCo 模型的躯干系一致，这是上机前的标定项。
- 起步约 1 秒内历史窗口是退化的（首帧复制填满、反馈为 0），误差增长快于
  稳态。需要高精度时可在进入行走状态后丢弃前 ~1 秒的估计。
