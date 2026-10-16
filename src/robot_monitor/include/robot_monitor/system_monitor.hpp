#ifndef ROBOT_MONITOR__SYSTEM_MONITOR_HPP_
#define ROBOT_MONITOR__SYSTEM_MONITOR_HPP_

#include <cstdint>
#include <string>

namespace robot_monitor
{

// 系统监控数据结构，包含一次采集的全部指标。
struct SystemStats
{
  float cpu_usage_percent{0.0f};
  float cpu_temperature{0.0f};
  float memory_usage_percent{0.0f};
  uint64_t memory_total_mb{0};
  uint64_t memory_available_mb{0};
  float disk_usage_percent{0.0f};
  uint64_t disk_total_gb{0};
  uint64_t disk_available_gb{0};
};

// 系统信息采集类。
// 通过读取 /proc 和 /sys 伪文件系统获取 CPU/内存/磁盘/温度等系统指标。
class SystemMonitor
{
public:
  SystemMonitor() = default;

  // 执行一次完整的系统信息采集。
  // disk_path 指定要检查的磁盘挂载点，默认为根目录。
  SystemStats collect(const std::string & disk_path = "/");

private:
  // 读取 /proc/stat 计算 CPU 使用率。
  // 首次调用不返回有效值，仅作为采样基点；后续调用返回与前次采样间隔内的平均使用率。
  float readCpuUsage();

  // 读取 /sys/class/thermal/thermal_zone*/temp 获取 CPU 温度。
  // 返回摄氏度，无法获取时返回 -1.0。
  float readCpuTemperature();

  // 读取 /proc/meminfo 获取 MemTotal 和 MemAvailable。
  // 输出参数 total_mb 和 available_mb 单位为 MB。
  void readMemoryInfo(uint64_t & total_mb, uint64_t & available_mb);

  // 调用 statvfs 系统调用获取指定路径的磁盘信息。
  // 输出参数 total_gb 和 available_gb 单位为 GB。
  void readDiskInfo(const std::string & path, uint64_t & total_gb, uint64_t & available_gb);

  // 上次 CPU 采样数据，用于计算间隔内的使用率。
  uint64_t prev_idle_{0};
  uint64_t prev_total_{0};
  bool first_sample_{true};
};

}  // namespace robot_monitor

#endif  // ROBOT_MONITOR__SYSTEM_MONITOR_HPP_
