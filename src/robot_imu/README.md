# robot_imu

YESENSE 惯性测量单元（IMU）ROS 2 驱动。通过串口读取 YESENSE 传感器二进制协议数据，解码后发布标准 `sensor_msgs/Imu` 和 YESENSE 专用 `NavAll` 消息。

## 包结构

```
robot_imu/
├── yesense_interface/          ← ROS 2 接口包（22 个自定义消息）
│   └── msg/                    ← NavAll, ImuData, EulerAngle, Quat, GnssPos...
└── yesense_std_ros2/           ← ROS 2 驱动节点
    ├── src/                    ← 解码器 + 节点实现
    │   └── serial/libserial.so ← vendored 串口库（wjwwood/serial）
    ├── config/yesense_config.yaml
    └── launch/yesense_node.launch.py
```

## 快速启动

```bash
ros2 launch yesense_std_ros2 yesense_node.launch.py
```

通过 bringup 启动时自动包含（真机模式下）。

## 硬件连接

```
YESENSE IMU ──UART/串口──→ USB-UART 适配器 ──USB──→ /dev/ttyACM0
                                                      │
                                                 (udev 规则)
                                                      ↓
                                               /dev/robot_imu
```

### udev 规则

```udev
SUBSYSTEM=="tty", ATTRS{idVendor}=="5953", ATTRS{idProduct}=="5543", SYMLINK+="robot_imu"
```

安装：
```bash
sudo cp scripts/udev/99-robot-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## ROS 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `serial_port` | string | `/dev/ttyACM0` | 串口设备路径（生产环境用 `/dev/robot_imu`） |
| `baud_rate` | int | `460800` | 波特率（支持 9600~921600） |
| `frame_id` | string | `basic_id` | TF 坐标系 ID |
| `driver_type` | string | `ros_serial` | 串口驱动：`ros_serial`（vendored 库）或 `linux_serial`（原生 POSIX） |

串口配置（应用层设置）：8 数据位，无校验，1 停止位，无流控，1000ms 超时。

## 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `imu/imu_data_ros` | `sensor_msgs/Imu` | 标准 ROS IMU 消息（四元数姿态 + 角速度 rad/s + 加速度 m/s²） |
| `imu/nav_all` | `yesense_interface/NavAll` | 完整导航输出（加速度、陀螺仪、欧拉角、四元数、GNSS、速度、UTC） |

以下消息类型已实现但默认注释关闭，可按需启用以降低带宽：`ImuData`、`ImuDataTenAxis`、`EulerOnly`、`RobotLord`、`AttitudeMinVru`、`AttitudeMinAhrs`、`AttitudeAllData`、`PosOnly`、`NavMin`、`NavMinUtc`。

## NavAll 消息结构

`NavAll` 是系统中最常使用的 YESENSE 消息（被 RL Manager 和仿真节点订阅）：

| 字段 | 类型 | 说明 |
|------|------|------|
| `tid` | `uint16` | 类型标识符 |
| `acc` | `ThreeAxis` | 加速度 (m/s²) |
| `gyro` | `ThreeAxis` | 角速度 |
| `euler` | `EulerAngle` | 欧拉角 (pitch/roll/yaw, 度) |
| `quat` | `Quat` | 四元数 (q0, q1, q2, q3) |
| `temp` | `SensorTemp` | 传感器温度 (°C) |
| `gnss` | `GnssPos` | GNSS 位置 (经纬度 + 海拔) |
| `nav_status` | `NavStatus` | 融合状态 + GNSS 状态 |
| `vel` | `Vel` | 速度 (东/北/天, m/s) |
| `utc` | `Utc` | UTC 时间 |
| `pressure` | `Pressure` | 气压 |

## YESENSE 二进制协议

```
帧格式: | 0x59 | 0x53 | TID (2B) | Len (1B) | Payload (TLV)... | CK1 | CK2 |
         ←─── CRC 计算起始 ───→
```

- **CRC**：16 位累加校验和，`CK1 = Σpayload_bytes`，`CK2 = Σ(CK1 增量)`
- **Payload**：重复的 TLV 块 `data_id (1B) | data_len (1B) | data (N bytes)`
- 数据 ID：`0x10` 加速度、`0x20` 陀螺仪、`0x40` 欧拉角、`0x41` 四元数、`0x50` UTC、`0x70` 速度、`0x80` 导航状态等
- 所有传感器数据为有符号 32-bit 整数 × 灵敏度因子（加速度 1e-6、欧拉角 1e-6、GNSS 经纬度 1e-10）

解码器使用 1024 字节环形缓冲区，500 Hz 轮询线程（2ms 定时器）从串口读取并逐帧解析。

## vendored 串口库

使用 `wjwwood/serial` 跨平台串口库（MIT License）：

| 文件 | 说明 |
|------|------|
| `src/serial/libserial.so` | 预编译 x86-64 动态库（~190KB） |
| `src/serial/include/serial/serial.h` | 完整 C++ API |
| `src/serial/include/serial/impl/unix.h` | POSIX/termios 实现 |
| `src/serial/include/serial/impl/win.h` | Windows 实现 |

也可通过 `driver_type: "linux_serial"` 使用原生 POSIX 调用（`open`/`termios`/`tcsetattr`），完全绕过 vendored 库。

## 依赖

| 依赖 | 说明 |
|------|------|
| `rclcpp` | ROS 2 C++ 客户端库 |
| `sensor_msgs` | 标准 IMU 消息 |
| `yesense_interface` | YESENSE 自定义消息（同仓） |

## 解码器错误码

| 错误码 | 常量 | 说明 |
|--------|------|------|
| -10 | `not_supported` | 不支持的操作 |
| -3 | `crc_err` | CRC 校验失败 |
| -2 | `data_len_err` | 数据长度错误 |
| -1 | `para_err` | 参数错误 |
| 0 | `analysis_ok` | 解析正常 |
| 1 | `analysis_done` | 解析完成 |
| 5 | `no_header` | 未找到帧头 |
| 10 | `buf_full` | 缓冲区满 |
