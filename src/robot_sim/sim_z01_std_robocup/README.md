# sim_z01_std_robocup

RoboCup 多机器人仿真：**一个 MuJoCo 模型 / 一个进程 / 一个窗口**，里面 6 台
`Z01_std_parallel_ankle`（robot_01~06）+ 足球场 + 两个球门 + 场心足球。

```bash
ros2 launch sim_z01_std_robocup sim_z01_std_robocup_launch.py
```

模型来自 `robot_assets/Z01_std_robocup`（生成物，见该目录 README），launch 只启动仿真节点。

## 与 sim_z01_std_football_player 的差异

| | football_player（单机） | robocup（本包） |
| --- | --- | --- |
| 机器人 | 1 台 | 6 台（同一模型内） |
| 话题 | `/robot/...` | `/robot_01/...` … `/robot_06/...` |
| 球 / 球门 | 基座系，`/robot/perception/*`，随足球模式显隐 | **世界系全局话题** `/sim/*`，场地常显常有碰撞 |
| 足球模式 | 有（隐藏场地/球） | 无（不需要清除场地的接口） |
| 出生点 | 原点 | 两条边线外随机（见下） |
| 重载 / 复位 | 无 | `/reload_robot_0X`、`/reload_all_robot`（完整回初始）、`/reset_robot_0X`、`/reset_all_robot`（只挪基座）、`/reset_ball` |
| 深度相机 | 1 路 | 6 路，每台周期独立可配 |

## 话题

**每台机器人一套**（`X` = 01…06）：

| 话题 | 方向 | 说明 |
| --- | --- | --- |
| `/robot_X/control/hardware_commands` | 订阅 | 与单机仿真完全一致的控制接口（`robot_interfaces/JointCommands`，关节名不带前缀） |
| `/robot_X/driver/joint_states` | 发布 | 200 Hz，31 关节视图（含 waist_pitch/waist_roll 零占位） |
| `/robot_X/driver/status` | 发布 | 5 Hz |
| `/robot_X/imu/nav_all` | 发布 | 基座位姿/速度/陀螺/欧拉角（与真机 yesense 兼容） |
| `/robot_X/camera/color/image_raw`、`/robot_X/camera/depth/image_rect_raw`、`/robot_X/camera/{color,depth}/camera_info` | 发布 | 64×36，周期由 `camera_period_us_list[X]` 决定；0 = 该台不发图像 |

**全局（世界坐标系）**：

| 话题 | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `/sim/ball_pose` | 发布 | `geometry_msgs/Point` | 球心世界坐标（200 Hz） |
| `/sim/ball_twist` | 发布 | `geometry_msgs/Vector3` | 球的世界系线速度（200 Hz） |
| `/sim/goal_team_a` | 发布 | `geometry_msgs/Point` | team A（robot_01~03）**进攻**球门 (x=-7.02)（5 Hz 重发） |
| `/sim/goal_team_b` | 发布 | `geometry_msgs/Point` | team B（robot_04~06）**进攻**球门 (x=+7.02) |
| `/sim/set_ball_pose` | **订阅** | `geometry_msgs/Point` | 放置球（世界系，z 会被夹到球半径 ≥0.11） |
| `/sim/set_ball_twist` | **订阅** | `geometry_msgs/Vector3` | 设置球速 |
| `/sim/ball_preview` | **订阅** | `geometry_msgs/Point` | 放置预览点（z<0 隐藏） |

> 注意：`set_*` / `ball_preview` 与 `ball_pose/ball_twist/goal_*` 一样都是**话题**（不是服务，
> 所以不会出现在 `ros2 service list` 里）。用话题是为了能像 football_player 的
> `/robot/teleop/set_ball_position` 那样连续推位置做拖动；服务式的 `Trigger` 无法携带坐标，
> 而自定义 srv 需要改 robot_interfaces（不在本仿真包范围内）。用法：
> `ros2 topic pub /sim/set_ball_pose geometry_msgs/msg/Point "{x: 2.0, y: 1.0, z: 0.11}" --once`

> 球的感知信息是**基座系无法共用**的，因此改成世界系客观量；各台机器人需要自行
> 换算到自己的基座系（自己位置可从 `/robot_X/imu/nav_all` 的 `pos` 拿到）。

## 服务（`std_srvs/Trigger`）

| 服务 | 说明 |
| --- | --- |
| `/reload_robot_01` … `/reload_robot_06` | 把该台**完整重载**到初始状态 |
| `/reload_all_robot` | 6 台在同一物理步内一起重载 |
| `/reset_robot_01` … `/reset_robot_06` | 把该台**只挪到**本队边线外的随机位置 |
| `/reset_all_robot` | 6 台在同一物理步内一起挪位置 |
| `/reset_ball` | 球回到 `ball_spawn`（默认场心 (0, 0, 0.11)），速度清零 |

两个机器人服务的区别（都取"本队边线外随机点 + 同队避碰"的出生点，朝向场内）：

| | `/reload_*`（完整重载） | `/reset_*`（只挪位置） |
| --- | --- | --- |
| 基座位置/朝向 | 出生点 | 出生点 |
| 基座速度 | 清零 | 清零 |
| 关节角 | **回到 `initial_joint_positions`**（默认全 0） | **保持不变** |
| 关节速度 | **清零** | **保持不变** |
| 关节力矩 | **清空命令缓存**（kp/kd/目标归零、enable=false → 力矩=0） | **保持不变**（命令缓存不动，控制侧无缝继续） |

两者都会清一次求解器热启动 `qacc_warmstart`（瞬移后接触集全变，不清会让第一帧出现
接触力尖峰；它属于求解器内部状态，不属于机器人状态）。

响应语义是 **accepted**：回调只置请求位，真正的 `qpos/qvel` 写入在物理线程的
`ExchangeMessages` 里完成（避免跨线程改 `mjData`）。message 里会说明该服务改了什么、
"将在下一个物理步生效"；**MuJoCo 窗口处于暂停状态时不会执行**（恢复运行才生效）。

## 出生点 / 重载 / 队伍规则（可配，见 config YAML）

- 场地 14m×9m（x∈[-7,7]，y∈[-4.5,4.5]），球门在 x=±7.02。
- **team A = robot_01~03**：y = +4.5 边线外 `sideline_margin`(默认 0.7m)、x∈[0.5, 6.5]，
  朝向场内（yaw = -90°）；守 +x 球门、攻 -x 球门。
- **team B = robot_04~06**：对角（y = -5.2、x∈[-6.5, -0.5]、yaw = +90°）；守 -x、攻 +x。
- 同队三台之间保持 `min_teammate_distance`(默认 1m) 间距：出生与单台重载都用
  拒绝采样，`spawn_max_attempts` 次失败后放宽并告警。
- 初始关节角 `initial_joint_positions` 用于**启动初始化**与 **`/reload_*`**（`/reset_*` 不碰关节）；
  默认全 0，后续若要换成"能稳定坐在地上的姿态"，填 29 个数即可（靠关节阻尼/frictionloss 保持）。

## 参数（`config/sim_z01_std_robocup.yaml`，都声明在主节点上）

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `state_publish_period_us` | 5000 | JointStates/IMU 节拍（200 Hz） |
| `driver_status_period_ms` | 200 | DriverStatus 周期 |
| `enable_control` | true | 仿真执行器是否接收控制输入（6 台共用） |
| `base_z` | 1.0 | 出生高度 |
| `camera_period_us_list` | 6×33333 | 每台深度相机周期（us），顺序 robot_01..06，0 = 该台不发图 |
| `depthcam_wait_us` | 0 | 异步取帧（推荐）；>0 时物理线程最多等这么久 |
| `track_robot_index` | 0 | 0 = 自由俯瞰全场；1..6 = 窗口跟踪该台 |
| `camera_distance/azimuth/elevation/lookat_z` | 16/90/-35/0.5 | 俯瞰机位 |
| `sideline_margin`、`team_a_x_range`、`team_b_x_range`、`min_teammate_distance`、`spawn_max_attempts` | — | 出生/重载取点规则 |
| `initial_joint_positions` | 空 | 29 个关节角（初始化 + `/reload_*` 用）；**留空注释**（YAML 空列表无法推断类型会报错） |
| `ball_spawn` | [0,0,0.11] | 球出生/复位位置 |

## 接 6 套控制栈（robot_control / robot_rl_manager）

本仿真**不启动**控制栈（保持与单机仿真包一致的职责边界）。控制栈内部话题名是
硬编码的 `/robot/...`，而 ROS 2 的 remap 是**整名匹配**——`-r /robot:=/robot_03`
**不会**把 `/robot/driver/joint_states` 变成 `/robot_03/driver/joint_states`，
必须逐话题写：

```bash
# 以 robot_03 为例（每套控制栈用自己的前缀；节点名也要区分，避免重名）
ros2 run robot_control robot_control_node --ros-args \
  -r __node:=robot_control_03 \
  -r /robot/router/accepted_commands:=/robot_03/router/accepted_commands \
  -r /robot/driver/joint_states:=/robot_03/driver/joint_states \
  -r /robot/rl_manager/status:=/robot_03/rl_manager/status \
  -r /robot/state:=/robot_03/state \
  -r /robot/control/hardware_commands:=/robot_03/control/hardware_commands \
  -r /robot/control/hardware_states:=/robot_03/control/hardware_states \
  -r /robot/control/preparation_progress:=/robot_03/control/preparation_progress \
  -r /robot/control/rejected_commands:=/robot_03/control/rejected_commands

ros2 run robot_rl_manager rl_manager_node --ros-args \
  -r __node:=rl_manager_03 \
  -r /robot/control/hardware_states:=/robot_03/control/hardware_states \
  -r /robot/imu/nav_all:=/robot_03/imu/nav_all \
  -r /robot/router/active_intent:=/robot_03/router/active_intent \
  -r /robot/state:=/robot_03/state \
  -r /robot/camera/depth/image_rect_raw:=/robot_03/camera/depth/image_rect_raw \
  -r /robot/rl_manager/desired_commands:=/robot_03/rl_manager/desired_commands \
  -r /robot/rl_manager/status:=/robot_03/rl_manager/status \
  -r /robot/rl_manager/load_model:=/robot_03/rl_manager/load_model \
  -r /robot/rl_manager/switch_model:=/robot_03/rl_manager/switch_model \
  -r /robot/rl_manager/finish_loop:=/robot_03/rl_manager/finish_loop \
  -r /robot/rl_manager/motion_stop:=/robot_03/rl_manager/motion_stop
  # 其余话题同理（perception/target_* 已改为 /sim/* 世界系，需在适配层自行换算）
```

> 本仓库目前没有任何节点发布球相关话题（`robot_teleop` 里也没有球接口）——`/sim/set_ball_pose`、
> `/sim/set_ball_twist`、`/sim/ball_preview` 是留给外部遥操作工具/脚本的钩子（用 `ros2 topic pub`
> 或自己的键盘脚本发即可）。football_player 用的是 `/robot/teleop/set_ball_position|set_ball_velocity|ball_preview`
> 这套名字，如果你的外部工具一直在用它们，把它 remap 到 `/sim/*` 即可（`/robot/sim/soccer_mode`
> 无对应，忽略——本场景场地常显）。

## 实现要点

- 6 台机器人在同一个 `mjModel` 里，MuJoCo 对象名统一加 `robot_0X_` 前缀
  （3.1.1 对重名对象是硬报错）；`ComputeControl` 在同一个回调里循环 6 台
  （`mjcb_control` 是全局单例，一个进程只能有一个仿真实例）。
- 每个机器人一个 rclcpp 节点（`sim_z01_std_robocup_robot_01..06`）+ 一个主节点
  （`sim_z01_std_robocup`，参数/服务/`/sim/*` 话题），7 个节点共用一个
  `SingleThreadedExecutor` 线程（`spin_some`，6×200Hz 命令不会积压）。
- 状态填充（JointStates/IMU）与单机仿真共用 `sim_common/SimStateFill.hpp`，
  避免两套语义漂移。
- 深度相机走 `mj_sim` 的多槽位 API（每台一个槽位，槽位号 = 机器人序号）：
  渲染线程只服务"有请求或作为预览"的槽位，物理线程默认异步取帧
  （`depthcam_wait_us=0`），不因 6 路相机阻塞。
- 模型容量：`njmax=4000 / nconmax=2000`（6 台人形互相碰撞实测 ncon≈340、
  nefc≈1600），每秒自检 `CONTACTFULL/CNSTRFULL` 并在不足时告警。
- **窗口不显示关节/执行器面板**：6 台机器人 = 205 关节 + 174 执行器，而 mjui 把整块
  面板渲染到一张 aux 缓冲、高度受 OpenGL 上限 16384 约束（实测面板高 17286，报
  `Auxiliary buffer height exceeds maximum allowed by OpenGL implementation` 后
  渲染线程卡死），因此 `MakeUiSections` 在关节/执行器过多时不构建这两个面板；
  多机场景的控制一律走 ROS 话题（详见 `mj_sim/src/simulate/simulate.cc` 的注释）。
- **深度相机吞吐**：6 路 × 30 Hz 时，每帧要为 6 个相机各重建一次场景，实测单路约
  17 Hz（渲染线程是瓶颈）。需要更高帧率时下调 `camera_period_us_list`（如 10 Hz/台）
  或减少开启的路数。深度图数值已与 `sim_z01_std_football_player` 对齐（见资产 README），
  可直接复用在该仿真下训练的深度策略。
