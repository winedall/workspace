# 系统监控状态。
# robot_monitor 节点定时采集并发布，供日志、看板或上层安全逻辑使用。
# 单位约定：内存以 MB 计，磁盘以 GB 计。

# 标准消息头；stamp 为采集时间戳。
std_msgs/Header header

# CPU 使用率（%），取值范围 0.0 ~ 100.0。
float32 cpu_usage_percent
# CPU 温度（摄氏度），无法获取时返回负值。
float32 cpu_temperature

# 内存占用率（%），取值范围 0.0 ~ 100.0。
float32 memory_usage_percent
# 物理内存总量（MB）。
uint64 memory_total_mb
# 可用物理内存（MB）。
uint64 memory_available_mb

# 磁盘占用率（%），取值范围 0.0 ~ 100.0。
float32 disk_usage_percent
# 磁盘总容量（GB）。
uint64 disk_total_gb
# 磁盘可用容量（GB）。
uint64 disk_available_gb

# 电池电压（伏特），取自 battery/info 或兼容的旧版电池话题。
float32 battery_voltage
# 电池电流（安培）。
float32 battery_current
# 电池荷电状态（%），取值范围 0.0 ~ 100.0。
float32 battery_soc
# 电池 MOS 温度（摄氏度），数据不可用时为 -1。
float32 battery_mos_temp
# 电池数据是否可用。
bool battery_available

# 当前系统输出音量（%），无法获取时为 -1。
float32 audio_volume_percent
# 当前系统是否静音。
bool audio_muted

# 以下告警标志由 robot_monitor 根据阈值自动设置。
# CPU 使用率超过 cpu_usage_max 阈值。
bool cpu_usage_over_threshold
# CPU 温度超过 cpu_temperature_max 阈值。
bool cpu_temperature_over_threshold
# 内存占用率超过 memory_usage_max 阈值。
bool memory_usage_over_threshold
# 磁盘占用率超过 disk_usage_max 阈值。
bool disk_usage_over_threshold
# 电池 SOC 低于 battery_soc_min 阈值。
bool battery_soc_low_warning
# 电池电压低于 battery_voltage_min 阈值。
bool battery_voltage_low_warning
# 电池 MOS 温度超过 mos_temp_max 阈值。
bool battery_mos_temp_over_threshold
