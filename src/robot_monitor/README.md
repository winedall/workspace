# robot_monitor

系统健康监控节点。采集 CPU、内存、磁盘、电池、音频状态，按阈值触发语音告警，并通过音频守护防止音量被外部程序意外修改。

## 采集指标

| 指标 | 数据源 | 方法 |
|------|--------|------|
| `cpu_usage_percent` | `/proc/stat` | 首行 `cpu` 字段，两次采样间 (Δtotal - Δidle) / Δtotal × 100% |
| `cpu_temperature` | `/sys/class/thermal/thermal_zone*/temp` | 扫描 zone0~zone9，取首个可读值 ÷ 1000，无可读返回 -1 |
| `memory_usage_percent` | `/proc/meminfo` | (MemTotal - MemAvailable) / MemTotal × 100% |
| `memory_total_mb` | `/proc/meminfo` | MemTotal kB / 1024 |
| `disk_usage_percent` | `statvfs()` | (总量 - 可用) / 总量 × 100% |
| `disk_total_gb` | `statvfs()` | f_blocks × f_frsize / 1024³ |
| `battery_voltage`/`soc` | `battery/info` 话题 | 来自 robot_battery 节点 |
| `audio_volume_percent` | pactl / amixer | 取决于 `audio_backend` |

## 快速启动

```bash
# 通过 bringup（默认启用）
ros2 launch robot_bringup zl_robot.launch.py

# 独立运行
ros2 run robot_monitor robot_monitor_node
```

## ROS 参数

### 监控阈值

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `rate_hz` | `1.0` | 指标采集发布频率 |
| `cpu_usage_max` | `80.0` | CPU 使用率告警阈值（%） |
| `cpu_temp_max` | `80.0` | CPU 温度告警阈值（°C） |
| `mem_usage_max` | `90.0` | 内存使用率告警阈值（%） |
| `disk_usage_max` | `90.0` | 磁盘使用率告警阈值（%） |
| `batt_soc_min` | `20.0` | 电池电量最低告警阈值（%） |
| `batt_voltage_min` | `21.0` | 电池电压最低告警阈值（V） |

### 告警控制

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `voice_enabled` | `true` | 语音告警总开关 |
| `alert_cooldown_sec` | `30.0` | 同类型告警最小间隔 |
| `disk_path` | `"/"` | 磁盘检查挂载点 |
| `battery_source` | `"robot"` | 电池来源：`robot` 订阅 `battery/info`，`usbcan` 订阅旧版 `/usbcan_bridge/battery_info` |

### 音频守护

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `audio_guard` | `true` | 音频守护开关 |
| `audio_rate_hz` | `1.0` | 音频轮询频率 |
| `audio_volume` | `60.0` | 目标音量（%） |
| `audio_volume_tol` | `2.0` | 音量容差（%） |
| `audio_backend` | `"auto"` | 后端：`auto`、`pactl`、`amixer` |

### 版本

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `version_file` | `""` | VERSION.txt 路径（空则从 CWD 向上搜索） |

## 话题与服务

| 类型 | 名称 | 用途 |
|------|------|------|
| **Pub** | `monitor/system_status` | 系统指标（CPU/内存/磁盘/电池/音频 + 告警标志，1 Hz） |
| **Pub** | `version` | 版本信息（3s 周期） |
| **Pub** | `audio/voice_text` | 告警语音文本（事件驱动，优先级 150） |
| **Sub** | `battery/info` | 接收 BMS 数据 |
| **Srv** | `monitor/set_volume` | 设置音量 |
| **Srv** | `monitor/set_mute` | 设置静音 |

## 告警语音

| 告警 | 触发条件 | 语音文本 |
|------|----------|----------|
| CPU 使用率 | > 80% | "警告：CPU使用率过高，当前百分之XX" |
| CPU 温度 | > 80°C | "警告：CPU温度过高，当前XX度" |
| 内存 | > 90% | "警告：内存占用率过高" |
| 磁盘 | > 90% | "警告：磁盘空间不足" |
| 电池 SOC | < 20% | "警告：电池电量不足，剩余百分之XX" |
| 电池电压 | < 21V | "警告：电池电压过低" |

### 冷却机制

每个告警键独立跟踪 `last_alert_time`。首次触发立即播报，后续在 `alert_cooldown_sec`（默认 30s）内抑制。冷却到期后重置时间戳并再次播报。

## 音频守护

`AudioManager` 在后台轮询系统音量/静音状态：

1. **检测到被外部静音** → 解除静音
2. **检测到音量偏离目标 ± 容差** → 恢复到 `audio_volume`
3. **主动静音**（`SetMute(true)`）→ 保持静音，不被守护恢复
4. **解除静音**（`SetMute(false)`）→ 同时恢复到目标音量

后端自动选择：优先 `pactl`（PulseAudio/PipeWire），失败后回退到 `amixer`（ALSA）。

## 后端命令

| 后端 | 读取 | 写入 |
|------|------|------|
| `pactl` | `pactl get-sink-volume @DEFAULT_SINK@` + `get-sink-mute` | `pactl set-sink-volume` / `set-sink-mute` |
| `amixer` | `amixer get Master` | `amixer set Master` |

## 依赖

| 依赖 | 说明 |
|------|------|
| `rclcpp` | ROS 2 C++ 客户端库 |
| `robot_interfaces` | `SystemStatus`、`VersionInfo`、`VoiceText`、`BatteryInfo` 消息 + `SetVolume`、`SetMute` 服务 |
