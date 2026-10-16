#include "robot_monitor/system_monitor.hpp"

#include <cstdio>
#include <fstream>
#include <string>
#include <sys/statvfs.h>

namespace robot_monitor
{

// 读取系统统计信息
SystemStats SystemMonitor::collect(const std::string & disk_path)
{
  SystemStats stats;

  // 读取 CPU 使用率
  stats.cpu_usage_percent = readCpuUsage();
  // 读取 CPU 温度
  stats.cpu_temperature = readCpuTemperature();

  // 读取内存占用率
  readMemoryInfo(stats.memory_total_mb, stats.memory_available_mb);
  if (stats.memory_total_mb > 0) {
    stats.memory_usage_percent = 100.0f *
      static_cast<float>(stats.memory_total_mb - stats.memory_available_mb) /
      static_cast<float>(stats.memory_total_mb);
  }

  // 读取磁盘空间占用率
  readDiskInfo(disk_path, stats.disk_total_gb, stats.disk_available_gb);
  if (stats.disk_total_gb > 0) {
    stats.disk_usage_percent = 100.0f *
      static_cast<float>(stats.disk_total_gb - stats.disk_available_gb) /
      static_cast<float>(stats.disk_total_gb);
  }

  return stats;
}

// 读取 CPU 使用率
float SystemMonitor::readCpuUsage()
{
  std::ifstream file("/proc/stat");
  if (!file.is_open()) {
    return 0.0f;
  }

  std::string line;
  std::getline(file, line);
  if (line.compare(0, 3, "cpu") != 0) {
    return 0.0f;
  }

  uint64_t user = 0, nice = 0, system = 0, idle = 0;
  uint64_t iowait = 0, irq = 0, softirq = 0, steal = 0;
  std::sscanf(line.c_str(), "cpu %lu %lu %lu %lu %lu %lu %lu %lu",
    &user, &nice, &system, &idle, &iowait, &irq, &softirq, &steal);

  uint64_t idle_time = idle + iowait;
  uint64_t total_time = user + nice + system + idle + iowait + irq + softirq + steal;

  if (first_sample_) {
    // 首次采样仅记录数据，不返回值，下一个周期才能计算差值。
    prev_idle_ = idle_time;
    prev_total_ = total_time;
    first_sample_ = false;
    return 0.0f;
  }

  uint64_t delta_idle = idle_time - prev_idle_;
  uint64_t delta_total = total_time - prev_total_;

  prev_idle_ = idle_time;
  prev_total_ = total_time;

  if (delta_total == 0) {
    return 0.0f;
  }
  return 100.0f * static_cast<float>(delta_total - delta_idle) / static_cast<float>(delta_total);
}

// 读取 CPU 温度
float SystemMonitor::readCpuTemperature()
{
  // 遍历 thermal_zone 目录，取第一个可读取的温度值。
  for (int i = 0; i < 10; ++i) {
    std::string path = "/sys/class/thermal/thermal_zone" + std::to_string(i) + "/temp";
    std::ifstream file(path);
    if (!file.is_open()) {
      continue;
    }
    int temp_milli = 0;
    file >> temp_milli;
    if (file.fail()) {
      continue;
    }
    // 文件中的值为毫摄氏度，除以 1000 转换为摄氏度。
    return static_cast<float>(temp_milli) / 1000.0f;
  }
  return -1.0f;
}

// 读取内存占用率
void SystemMonitor::readMemoryInfo(uint64_t & total_mb, uint64_t & available_mb)
{
  total_mb = 0;
  available_mb = 0;

  std::ifstream file("/proc/meminfo");
  if (!file.is_open()) {
    return;
  }

  std::string line;
  while (std::getline(file, line)) {
    uint64_t val = 0;
    if (std::sscanf(line.c_str(), "MemTotal: %lu kB", &val) == 1) {
      total_mb = val;
    } else if (std::sscanf(line.c_str(), "MemAvailable: %lu kB", &val) == 1) {
      available_mb = val;
    }
    if (total_mb > 0 && available_mb > 0) {
      break;
    }
  }
  // meminfo 中单位为 kB，转换为 MB。
  total_mb /= 1024;
  available_mb /= 1024;
}

// 读取磁盘空间
void SystemMonitor::readDiskInfo(const std::string & path, uint64_t & total_gb, uint64_t & available_gb)
{
  struct statvfs stat;
  if (statvfs(path.c_str(), &stat) != 0) {
    total_gb = 0;
    available_gb = 0;
    return;
  }
  const uint64_t bytes_total = static_cast<uint64_t>(stat.f_blocks) * static_cast<uint64_t>(stat.f_frsize);
  const uint64_t bytes_avail = static_cast<uint64_t>(stat.f_bavail) * static_cast<uint64_t>(stat.f_frsize);
  total_gb = bytes_total / (1024ULL * 1024ULL * 1024ULL);
  available_gb = bytes_avail / (1024ULL * 1024ULL * 1024ULL);
}

}  // namespace robot_monitor
