# usbcan_bridge_interface

USB-CAN 桥接接口定义包。纯 ROS 2 消息和服务定义，用于机器人电源/配电系统通过 USB-CAN 总线与 BMS 和电源板的通信。

## 消息

| 消息 | 用途 |
|------|------|
| `BatteryInfo` | BMS 完整遥测（15 路电芯电压/温度、总电压/电流/SOC、MOS 状态、充放电状态、2 页故障/告警标志，共 129 字段） |
| `PowerInfo` | 电源板遥测（6 路电压/电流通道、GPIO 状态、5 轴 P48V 故障标志、ADC 温度） |
| `SbusSwitch` | SBUS 遥控开关状态（16 通道、丢帧标志、失控保护标志） |

## 服务

| 服务 | 用途 |
|------|------|
| `PowerLog` | 电源板日志控制（启停/文件路径/间隔/阈值） |
| `VersionQuery` | 固件版本查询 |

## 依赖

- `std_msgs`、`std_srvs`、`rosidl_default_generators`、`ament_cmake`
- 成员包：`rosidl_interface_packages`

## 构建

```bash
colcon build --packages-select usbcan_bridge_interface
```
