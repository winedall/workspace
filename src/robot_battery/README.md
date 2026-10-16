# robot_battery

通过 SocketCAN 读取 BMS 电池状态，并发布 `robot_interfaces/msg/BatteryInfo` 到 `battery/info`。

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `device` | `"can0"` | SocketCAN 接口名称 |
| `poll_rate_hz` | `10.0` | BMS 查询帧发送频率 |
| `publish_rate_hz` | `10.0` | 电池状态发布频率 |

## 话题

| 类型 | 名称 | 说明 |
|------|------|------|
| **Pub** | `battery/info` | 电池总压、电流、SOC、温度、MOS 和状态信息 |

## 协议

节点周期发送扩展帧 `0x0400FF80` 查询 BMS，并解析回包中 `robot_interfaces/msg/BatteryInfo` 已保留的字段。当前不解析旧程序中的单体数组和大量 fault 位，避免常规运行路径过重。
