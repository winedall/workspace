# robot_driver

硬件驱动适配层。将统一关节命令转换为 EtherCAT 总线上的电机控制帧，同时将硬件反馈标准化为统一关节状态。是控制管线的最末端——软件与物理世界的接口。

## 架构

```
RobotDriverNode (ROS 2 节点, MultiThreadedExecutor 2 线程)
├── HardwareBackend (抽象接口)
│   ├── EncosBackend         ← Encos + SOEM EtherCAT
│   ├── TienkungBackend      ← Tienkung + IGH EtherCAT
│   └── MixBackend           ← Encos/Tienkung 混装 + SocketCAN 或 IGH gateway
├── BackendFactory           ← 根据 hardware.vendor_type 选择后端
└── HardwareConfig           ← YAML 硬件配置解析，vendor/bus 唯一来源
```

## 运行模式

`config_mode` 决定驱动节点是否进入正常运控周期：

| config_mode | 行为 |
|-------------|------|
| `false` | 正常运控模式：完整通讯周期（编码→交换→解码），运动写入启用 |
| `true` | 配置模式：只开放厂家维护 service，不启动周期运控 worker |

## ROS 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `robot_joints_config` | string | `""` | 关节配置 YAML 路径 |
| `hardware_config` | string | `""` | 硬件映射 YAML 路径（如 `hw_mix_canfd.yaml`） |
| `config_mode` | bool | `false` | 配置模式，仅开放维护服务 |
| `joint_state_pub_hz` | double | `200.0` | 关节状态发布频率 |
| `driver_status_pub_hz` | double | `10.0` | 驱动状态发布频率 |
| `motor_fault_confirm_frames` | int | `10` | 电机故障确认帧数（去抖） |

## 话题与服务

| 类型 | 名称 | 用途 |
|------|------|------|
| **Sub** | `control/hardware_commands` | 接收 `robot_control` 输出的统一关节命令（高频 QoS） |
| **Pub** | `driver/joint_states` | 发布标准化关节反馈（高频 QoS） |
| **Pub** | `driver/status` | 发布驱动诊断状态 |
| **Srv** | `driver/encos/get_motor_id` | 读取电机 ID |
| **Srv** | `driver/encos/set_motor_id` | 修改电机 ID |
| **Srv** | `driver/encos/set_motor_zero` | 设置电机当前位置为零点 |
| **Srv** | `driver/encos/get_motor_param` | 读取电机参数（17 种参数码） |
| **Srv** | `driver/mix/query_joint` | 按关节名查询混装 CANFD 关节状态 |
| **Srv** | `driver/mix/set_zero` | 按关节名设置混装 CANFD 电机零点 |
| **Srv** | `driver/mix/query_motor_id` | 按关节名查询混装 CANFD 电机 ID |
| **Srv** | `driver/mix/set_motor_id` | 按关节名修改混装 CANFD 电机 ID |
| **Srv** | `driver/tienkung/query_status` | 按关节名查询天工关节状态 |
| **Srv** | `driver/tienkung/set_zero` | 按关节名设置天工电机零点 |
| **Srv** | `driver/tienkung/modify_id` | 按关节名修改天工电机 ID |
| **Srv** | `driver/tienkung/query_id` | 广播查询天工电机 ID |
| **Srv** | `driver/tienkung/reset_id` | 广播重置天工电机 ID |
| **Srv** | `driver/tienkung/query_mcu_id` | 按关节名发送 0x05 查询帧，原样回传电机 MCU ID 应答数据 |

Encos、Tienkung 和 Mix 的关节级维护接口均以 `joint_name` 定位关节；`slave_id`、`passage`、`canfd.bus` 等拓扑字段只保存在硬件配置 YAML 中，由 backend 解析。

## 命令处理流程

```
handleCommand()                          ← ROS 回调，200 Hz
  ├── 校验关节名称（白名单）
  ├── 校验位置限幅（min_pos/max_pos）
  ├── 3 帧去抖检测 INVALID_ANGLE_CMD
  └── 缓存到 pending 命令队列

ethercatWorkerLoop()                     ← 独立实时线程
  ├── encodePendingCommands()            ← 编码到 CAN PDO
  ├── runEthercatCycle()                 ← SOEM ec_send_processdata / ec_receive_processdata
  ├── decodeFeedback()                   ← 解码反馈帧（位置/速度/力矩/温度/故障码）
  ├── refreshJointFeedback()             ← 故障去抖 + 位置限幅检查 + 零点缺失检测
  └── clock_nanosleep(TIMER_ABSTIME)    ← 硬实时周期（默认 2ms）
```

## EtherCAT 集成

### PDO 布局
每个 Encos EtherCAT 从站包含 6 路 CAN 电机帧（`EthercatMsg`），每路包含 CAN ID + RTR + DLC + 8 字节数据。

### 总线生命周期
```
initEthercat → open(网卡) → configure(扫描从站) → 进入 OP
    ↓ 失败
setFault() → 锁存故障，等待上层重启或恢复流程
```

### 运动命令编码（8 字节 DLC）
kp（12-bit）、kd（9-bit）、position（16-bit）、velocity（12-bit）、torque（12-bit）经方向/偏移/减速比变换后量化为 CAN 帧。

### 反馈解码
- 校验 `dlc == 8` 且 `ack_status == 1`
- 校验 `motor_id` 匹配
- 逆量化 + 逆减速比 + 逆偏移 + 逆方向 → 关节空间物理值
- 提取 `motor_error`（data[0] bit 0-4）、`motor_temp`、`mos_temp`

### 安全管理事务
ROS 服务调用通过 mutex + condition_variable 等待 EtherCAT 工作线程完成多周期管理事务（SendRequest → WaitFeedback，含重试），同时仅允许一个事务。

## 故障处理

| 故障恢复动作 | 触发条件 | 行为 |
|-------------|----------|------|
| 配置/协议错误 | 锁存故障，停止周期写入 |
| 通信错误（WKC、输出缓冲区异常） | 锁存故障，停止周期写入 |

### 运动写入安全门（`canWriteMotionCommand`）
全部条件满足才允许写入：`ethercat_master_` 非空 ∧ `protocol_` 非空 ∧ `ethercat_configured_` 为真 ∧ `fault_active_` 为假。

### 反馈层安全
- **电机故障去抖**：连续 `motor_fault_confirm_frames` 帧后才确认故障
- **位置限幅**：超出 `min_pos/max_pos` → `JOINT_ERR_INVALID_ANGLE_STATE`
- **零点缺失检测**：真机模式下编码器反馈恰好为 0.0 → `JOINT_ERR_ZERO_ABSENCE`

## 硬件配置 YAML 结构

```yaml
vendor_type: encos
bus_type: ethercat
ethercat:
  network_interface: enp2s0
  cycle_time_ms: 2
  slave_count: 6
joints:
  - name: left_hip_pitch
    motor_id: 1
    direction: 1        # +/-1
    offset: 0.0         # 度
    ratio: 1.0          # 减速比
    encos:
      slave_id: 0
      passage: 1
      kt: 0.5
      kp_min: 0.0
      kp_max: 500.0
      kd_min: 0.0
      kd_max: 5.0
```

## 扩展新厂商

```cpp
// backend_factory.cpp
if (vendor == "new_vendor") {
    return std::make_unique<NewVendorBackend>(config);
}
```

新增后端时在 `backend_factory.cpp` 中按 `hardware.vendor_type` 路由，并实现 `HardwareBackend` 的周期钩子：`transportReady()`、`exchangeProcessData()`、`encodeJointCommand()`、`readJointFeedback()`、`configured()`；如电机需要首帧后的启动握手，再覆盖 `runStartup()`。

## SOEM 主站细节

- 单实例设计（SOEM 使用全局状态）
- `io_map_` 4096 字节，由 `ec_config_map` 映射 PDO
- 后台监控线程（50ms）：从站状态检查、WKC 错误检测、从站恢复
- 配置重试：最多 3 次，每次关闭/重新打开
- 禁用 `ECT_COEDET_SDOCA` 标志（与部分 Encos 子板不兼容）
- WKC 健康：1000 周期窗口内最多 10 次错误，需连续 3 次健康周期才算稳定
