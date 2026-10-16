# mj_sim

robot_sim 仿真工作区的**基础库包**：封装 MuJoCo 物理引擎与 GUI 渲染，提供与机器人型号无关的仿真生命周期基类，并负责把模型资产（XML）自动生成 C++ 配置头文件。各仿真包（`sim_*`）通过 `sim_common` 在本包之上实现具体机器人仿真。

## 组成

```
mj_sim
├── include/mj_sim/        MujocoSimBase（仿真生命周期基类）、MotorSpec（电机 TN 曲线）
├── include/simulate/      MuJoCo 官方 simulate UI（GLFW 适配层）
├── include/robot_header/  由 XML 自动生成的机器人配置头文件
├── src/mj_sim/            MujocoSimBase 实现（物理线程、实时同步、模型加载）
├── src/simulate/          simulate UI 源码（glfw_adapter、simulate、lodepng）
└── thirdparty/            vendored 依赖（MuJoCo / GLFW / Eigen）
```

### MujocoSimBase

与具体机器人无关的仿真生命周期封装：

- 加载 XML/MJB 模型，创建 `mjModel`/`mjData`；
- 主线程进入 MuJoCo GUI 渲染循环，物理线程推进仿真；
- 注册 MuJoCo 控制回调（静态函数指针经 `controller_instance_` 转发到派生类）；
- 在仿真节拍中调用派生类的消息交换钩子。

派生类需要实现的**纯虚钩子**：

| 钩子 | 职责 |
|------|------|
| `InitRobotState(mjData*)` | 设置初始 qpos/qvel（初始位姿） |
| `ComputeControl(mjModel*, mjData*)` | 每个 MuJoCo 控制步写入执行器控制量 `d->ctrl` |
| `ExchangeMessages(double sync_sim)` | 按仿真时间发布状态 / 读取外部命令 |

**可选钩子**：`InitInterface()`（模型加载后解析索引）、`InitCamera()`（设置相机）、`Cleanup()`（退出清理）、`RecordData()`（离线记录）。

其他特性：

- **实时倍率同步**：物理线程比较仿真时间与 CPU 时间推进步数，偏差超过 `syncMisalign`（0.1 s）时重新对齐；每个刷新周期最多用 `simRefreshFraction`（0.7）的时间做物理步进，避免渲染卡顿；
- **GUI 模型热重载**：运行中经 MuJoCo 文件选择器重载 XML，自动重建 `mjModel`/`mjData` 并重新调用初始化钩子；
- **插件扫描**：启动时扫描可执行文件同目录下的 `mujoco_plugin/`，加载自定义 actuator/sensor 插件；
- **暂停时仍做 `mj_forward`**，保证拖拽、扰动后传感器读数最新。

### simulate/（渲染）

MuJoCo 官方 `simulate` UI 的 vendored 拷贝，经 `GlfwAdapter` 提供窗口渲染、暂停/运行、速度调节、模型重载等 GUI 能力，可 headless 运行。

### 深度相机（多槽位）

`mj::Simulate` 支持最多 8 个深度相机槽位（`ConfigureDepthCamera` / `SetDepthCameraCount`
/ `SetDepthPreviewSlot` / `RequestDepthCamera` / `TryTakeDepthFrame`）：

- **物理线程**只置请求位并在 `TryTakeDepthFrame` 里取帧（`timeout=0` 即异步，不阻塞）；
- **渲染线程**在 `Render()` 里经 `ProcessDepthCamRequest` 服务"有请求或作为常驻预览"
  的槽位，写像素时持 `slot.mtx`，写完才清请求位；
- 锁序只允许 `sim_->mtx → slot.mtx`：`ProcessDepthCamRequest` **不能**取 `sim_->mtx`
  （物理线程持 `sim_->mtx` 等深度帧，反向加锁即死锁）；
- 单机器人仿真只用槽位 0（`SimNodeBase::InitDepthCamera` 配置，行为与旧单相机一致）；
  `sim_z01_std_robocup` 每台机器人一个槽位，且关闭常驻预览（`SetDepthPreviewSlot(-1)`）。

### MotorSpec（电机模型）

描述电机 TN 曲线（扭矩-速度曲线）的物理规格：恒扭矩区 → 恒功率线性衰减区 → 超速零扭矩区，并区分驱动/制动工况系数。`computeTorqueLimit()` 根据当前关节速度计算允许的扭矩上限，供仿真包在控制律输出上叠加物理限制。

### vendored 依赖

无需系统级安装，CMake 自动检测架构（x86_64 / aarch64）并选择对应二进制：

| 依赖 | 版本 | 路径 | 平台 |
|------|------|------|------|
| MuJoCo | 3.1.1 | `thirdparty/mujoco/` | x86_64 / aarch64 |
| GLFW3 | 静态库 | `thirdparty/glfw/` | x86_64 / aarch64 |
| Eigen3 | vendored | `thirdparty/eigen3/` | 头文件 |

## 模型资产与自动代码生成

模型资产（MuJoCo XML / URDF / STL 网格）位于独立仓库 `robot_assets`（默认 `src/robot_assets`），由 CMake 变量 `ROBOT_ASSETS_DIR` 定位，未找到时构建报错提示。

构建时自动扫描 `robot_assets/*/xml/*.xml`，调用 `robot_assets/utils/header_generator.py` 生成 `include/robot_header/<模型>.h`，内容包括：

- `kJointsName` / `kActuatorsName` — 关节/执行器名称数组（按 XML 顺序）
- `kJointsNum` — 关节数量
- `kJointsMaxTorque` / `kJointsMinTorque` — 关节力矩限幅
- 传感器名称（orientation / gyro / acc）、相机信息、基座 body 名称
- Eigen 类型别名（`SystemQ`/`SystemDq`/`SystemTau`/`JointTau` 等）

XML 是唯一真源，头文件随 XML 变化自动重新生成。

## 构建

```bash
colcon build --packages-select mj_sim
# 资产仓库位于其他路径时：
colcon build --packages-select mj_sim --cmake-args -DROBOT_ASSETS_DIR=/path/to/robot_assets
```

产物与安装布局：

| 产物 | 安装位置 |
|------|----------|
| `libmj_sim`（含 simulate UI） | `lib/` |
| 头文件（含生成的 robot_header） | `include/mj_sim/` |
| XML 场景文件 | `share/mj_sim/description/`（仿真包经 `FindPackageShare('mj_sim')` 解析模型路径） |
| `libmujoco.so.3.1.1` | `lib/` |

## 注意事项

- **mj_sim 是静态库**（`libmj_sim.a`）：改动 mj_sim 后必须重建依赖它的仿真包
  （`colcon build` 会按依赖顺序重建；只跑 `--packages-select mj_sim` 不会重新链接
  仿真可执行文件，运行时会仍是旧代码）。
- **多机器人模型的 UI 面板**：`MakeUiSections` 在"关节数或执行器数 > 100"时不再构建
  Joint/Control 面板。原因是面板高度 ≈ 滑块数 × 单项高度，而 mjui 把整块面板渲染到一张
  offscreen aux 缓冲，高度受 OpenGL 上限 16384 约束：6 台机器人（205 关节 + 174 执行器）
  实测面板高 17286，`mjr_addAux` 报
  `ERROR: Auxiliary buffer height exceeds maximum allowed by OpenGL implementation`
  （只写在进程 CWD 的 `MUJOCO_LOG.TXT` 里），随后渲染线程卡死在 `UiModify(ui1)`、
  物理线程停在 `Simulate::Load()`，表现为"窗口打开但没有任何话题发布"。
  单项高度随 UI 字体缩放变化，故用保守判据；单机模型（34/29）不受影响。

## 使用方式

本包不生成可执行文件，只提供库与头文件。下游 `sim_common` 将 `MujocoSimBase` 与 ROS 通信层组合为 `SimNodeBase`；各仿真包（`sim_z01_edm` 等）声明 `depend mj_sim` 并继承基类实现钩子。整体架构与数据流见工作区根目录的 `README.md`。
