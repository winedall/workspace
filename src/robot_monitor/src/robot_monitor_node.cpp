#include <cmath>
#include <filesystem>
#include <fstream>
#include <vector>

#include "robot_common/audio_tips.hpp"
#include "robot_monitor/robot_monitor_node.hpp"

namespace robot_monitor
{

RobotMonitorNode::RobotMonitorNode(const rclcpp::NodeOptions & options)
: Node("robot_monitor_node", options)
{
  // 声明 ROS 参数，支持 launch/YAML 覆盖。
  rate_hz_ = declare_parameter<double>("rate_hz", 1.0);
  disk_path_ = declare_parameter<std::string>("disk_path", "/");
  battery_source_ = declare_parameter<std::string>("battery_source", "robot");
  voice_enabled_ = declare_parameter<bool>("voice_enabled", true);
  alert_cooldown_sec_ = declare_parameter<double>("alert_cooldown_sec", 30.0);

  voice_cpu_usage_ = declare_parameter<bool>("voice_cpu_usage", true);
  voice_cpu_temp_ = declare_parameter<bool>("voice_cpu_temp", true);
  voice_mem_usage_ = declare_parameter<bool>("voice_mem_usage", true);
  voice_disk_usage_ = declare_parameter<bool>("voice_disk_usage", true);
  voice_batt_soc_ = declare_parameter<bool>("voice_batt_soc", true);
  voice_batt_voltage_ = declare_parameter<bool>("voice_batt_voltage", true);
  voice_batt_mos_temp_ = declare_parameter<bool>("voice_batt_mos_temp", true);

  cpu_usage_max_ = static_cast<float>(declare_parameter<double>("cpu_usage_max", 80.0));
  cpu_temp_max_ = static_cast<float>(declare_parameter<double>("cpu_temp_max", 80.0));
  mem_usage_max_ = static_cast<float>(declare_parameter<double>("mem_usage_max", 90.0));
  disk_usage_max_ = static_cast<float>(declare_parameter<double>("disk_usage_max", 90.0));
  batt_soc_min_ = static_cast<float>(declare_parameter<double>("batt_soc_min", 20.0));
  batt_voltage_min_ = static_cast<float>(declare_parameter<double>("batt_voltage_min", 21.0));
  batt_mos_temp_max_ = static_cast<float>(declare_parameter<double>("batt_mos_temp_max", 80.0));
  audio_guard_ = declare_parameter<bool>("audio_guard", true);
  audio_rate_hz_ = declare_parameter<double>("audio_rate_hz", 1.0);
  audio_volume_ = static_cast<float>(declare_parameter<double>("audio_volume", 60.0));
  audio_volume_tol_ = static_cast<float>(declare_parameter<double>("audio_volume_tol", 2.0));
  audio_backend_ = declare_parameter<std::string>("audio_backend", "auto");

  version_file_ = declare_parameter<std::string>("version_file", "");

  // 创建系统信息采集器。
  system_monitor_ = std::make_unique<SystemMonitor>();
  // 创建音频管理器；音量读写、静音和外部修改恢复逻辑都封装在子模块内。
  AudioManagerConfig audio_config;
  audio_config.guard_enabled = audio_guard_;
  audio_config.desired_volume_percent = audio_volume_;
  audio_config.volume_tolerance_percent = audio_volume_tol_;
  audio_config.backend = audio_backend_;
  audio_manager_ = std::make_unique<AudioManager>(audio_config);

  // 订阅电池信息；新老来源只在回调中转换，监控主逻辑统一读取 BatterySample。
  if (battery_source_ == "usbcan") {
    usbcan_battery_sub_ = create_subscription<usbcan_bridge_interface::msg::BatteryInfo>(
      "/usbcan_bridge/battery_info",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&RobotMonitorNode::onUsbcanBattery, this, std::placeholders::_1));
  } else {
    if (battery_source_ != "robot") {
      RCLCPP_WARN(
        get_logger(), "未知电池来源 %s，默认使用 robot", battery_source_.c_str());
      battery_source_ = "robot";
    }
    robot_battery_sub_ = create_subscription<robot_interfaces::msg::BatteryInfo>(
      "battery/info",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&RobotMonitorNode::onRobotBattery, this, std::placeholders::_1));
  }

  // 发布系统监控状态。
  status_pub_ = create_publisher<robot_interfaces::msg::SystemStatus>(
    "monitor/system_status", rclcpp::SystemDefaultsQoS());

  // 版本信息发布器，3 秒周期发布，不缓存旧消息。
  version_pub_ = create_publisher<robot_interfaces::msg::VersionInfo>(
    "version", rclcpp::SystemDefaultsQoS());

  // 启动时加载版本信息。
  loadVersion();

  // 语音播报客户端，使用 transient_local 确保 audio 节点启动较晚时消息不丢失。
  audio_client_ = std::make_unique<robot_common::AudioClient>(*this);

  // 创建监控定时器。
  const auto monitor_period = std::chrono::duration<double>(1.0 / std::max(rate_hz_, 0.1));
  monitor_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(monitor_period),
    std::bind(&RobotMonitorNode::onMonitor, this));

  // 创建音频监控定时器。
  const auto audio_period = std::chrono::duration<double>(1.0 / std::max(audio_rate_hz_, 0.1));
  audio_guard_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(audio_period),
    std::bind(&RobotMonitorNode::onAudioGuard, this));

  // 版本发布定时器（3 秒周期，版本信息不频繁变化）。
  version_timer_ = create_wall_timer(
    std::chrono::seconds(3),
    std::bind(&RobotMonitorNode::onVersion, this));

  // 创建音频音量设置服务。
  set_volume_srv_ = create_service<robot_interfaces::srv::SetVolume>(
    "monitor/set_volume",
    std::bind(
      &RobotMonitorNode::handleSetVolume, this, std::placeholders::_1, std::placeholders::_2));

  // 创建音频静音设置服务。
  set_mute_srv_ = create_service<robot_interfaces::srv::SetMute>(
    "monitor/set_mute",
    std::bind(
      &RobotMonitorNode::handleSetMute, this, std::placeholders::_1, std::placeholders::_2));

  // 创建音频恢复服务。
  audio_recover_srv_ = create_service<std_srvs::srv::Trigger>(
    "monitor/audio_recover",
    std::bind(
      &RobotMonitorNode::handleRecoverAudio, this, std::placeholders::_1, std::placeholders::_2));
  
  // 初始化监控节点日志。
  RCLCPP_INFO(get_logger(), "监控节点已启动，采集频率 %.1f Hz，阈值: CPU使用率>%.0f%%, CPU温度>%.0f°C, 内存>%.0f%%, 磁盘>%.0f%%, 电池SOC<%.0f%%, 电压<%.1fV, MOS温度>%.0f°C",
    rate_hz_,
    cpu_usage_max_, cpu_temp_max_,
    mem_usage_max_, disk_usage_max_,
    batt_soc_min_, batt_voltage_min_,
    batt_mos_temp_max_);

  start_time_ = now();
}

// 处理音频监控定时器回调函数
void RobotMonitorNode::onAudioGuard()
{
  if (audio_manager_) {
    audio_manager_->pollAndRestore();
  }
}

// 处理最新电池信息回调函数
void RobotMonitorNode::onRobotBattery(const robot_interfaces::msg::BatteryInfo::SharedPtr msg)
{
  battery_.voltage = std::round(static_cast<float>(msg->total_volt) * 0.1f * 1000.0f) / 1000.0f;
  battery_.current = std::round(static_cast<float>(msg->current) * 0.1f * 1000.0f) / 1000.0f;
  battery_.soc = std::round(static_cast<float>(msg->soc) * 0.1f * 1000.0f) / 1000.0f;
  battery_.mos_temp = static_cast<float>(msg->mos_temp);
  have_battery_ = true;
}

// 旧版 usbcan_bridge 电池信息订阅回调。
void RobotMonitorNode::onUsbcanBattery(const usbcan_bridge_interface::msg::BatteryInfo::SharedPtr msg)
{
  battery_.voltage = std::round(static_cast<float>(msg->total_volt) * 0.1f * 1000.0f) / 1000.0f;
  battery_.current = std::round(static_cast<float>(msg->current) * 0.1f * 1000.0f) / 1000.0f;
  battery_.soc = std::round(static_cast<float>(msg->soc) * 0.1f * 1000.0f) / 1000.0f;
  battery_.mos_temp = static_cast<float>(msg->mos_temp);
  have_battery_ = true;
}

// 加载版本信息：从 VERSION.txt 解析最新版本号与发布日期。
void RobotMonitorNode::loadVersion()
{
  std::string path = version_file_;

  // 路径为空时从当前目录向上搜索 VERSION.txt（与 robot_common 的 resolve 模式一致）。
  if (path.empty()) {
    auto current = std::filesystem::current_path();
    while (true) {
      auto candidate = current / "VERSION.txt";
      if (std::filesystem::exists(candidate)) {
        path = candidate.string();
        break;
      }
      if (!current.has_parent_path() || current == current.parent_path()) {
        break;
      }
      current = current.parent_path();
    }
  }

  if (path.empty()) {
    RCLCPP_WARN(get_logger(), "未找到 VERSION.txt，版本信息不可用");
    return;
  }

  std::ifstream file(path);
  if (!file.is_open()) {
    RCLCPP_WARN(get_logger(), "无法打开版本文件: %s", path.c_str());
    return;
  }

  std::string line;
  std::string current_date;
  std::string current_version;
  while (std::getline(file, line)) {
    if (line.compare(0, 5, "date=") == 0) {
      current_date = line.substr(5);
    } else if (line.compare(0, 8, "version=") == 0) {
      current_version = line.substr(8);
    }

    if (!current_version.empty() && !current_date.empty()) {
      version_info_.version = current_version;
      version_info_.release_date = current_date;
      version_loaded_ = true;
      RCLCPP_INFO(get_logger(), "已加载版本信息: %s (%s)",
                  version_info_.version.c_str(),
                  version_info_.release_date.c_str());
      return;
    }
  }

  RCLCPP_WARN(get_logger(), "VERSION.txt 格式不正确，未找到 version/date 字段");
}

// 版本发布定时器回调：每 3 秒发布一次版本信息。
void RobotMonitorNode::onVersion()
{
  if (version_loaded_) {
    version_pub_->publish(version_info_);
  }
}

// 处理系统监控定时器回调函数
void RobotMonitorNode::onMonitor()
{
  // 采集系统信息。
  auto stats = system_monitor_->collect(disk_path_);

  // 填充系统状态消息。
  robot_interfaces::msg::SystemStatus status;
  status.header.stamp = now();

  // 所有浮点值保留 3 位小数以便阅读。
  status.cpu_usage_percent = std::round(stats.cpu_usage_percent * 1000.0f) / 1000.0f;
  status.cpu_temperature = std::round(stats.cpu_temperature * 1000.0f) / 1000.0f;
  status.memory_usage_percent = std::round(stats.memory_usage_percent * 1000.0f) / 1000.0f;
  status.memory_total_mb = stats.memory_total_mb;
  status.memory_available_mb = stats.memory_available_mb;
  status.disk_usage_percent = std::round(stats.disk_usage_percent * 1000.0f) / 1000.0f;
  status.disk_total_gb = stats.disk_total_gb;
  status.disk_available_gb = stats.disk_available_gb;

  // 填充电池信息；电池回调已经把新老消息统一转换成实际单位。
  if (have_battery_) {
    status.battery_voltage = battery_.voltage;
    status.battery_current = battery_.current;
    status.battery_soc = battery_.soc;
    status.battery_mos_temp = battery_.mos_temp;
    status.battery_available = true;
  } else {
    status.battery_voltage = -1.0f;
    status.battery_current = -1.0f;
    status.battery_soc = -1.0f;
    status.battery_mos_temp = -1.0f;
    status.battery_available = false;
  }

  if (audio_manager_) {
    // SystemStatus 只发布基础音频信息，详细期望状态由 AudioManager 内部维护。
    const auto audio_status = audio_manager_->refreshStatus();
    status.audio_volume_percent = audio_status.volume_percent;
    status.audio_muted = audio_status.muted;
  } else {
    status.audio_volume_percent = -1.0f;
    status.audio_muted = false;
  }

  // 首次采集时打印一次实际状态值（等待电池数据就绪，最多等 10 秒）。
  if (!status_logged_) {
    const auto elapsed = (now() - start_time_).seconds();
    if (have_battery_ || elapsed > 10.0) {
      status_logged_ = true;
      const char * battery_state = have_battery_ ? "已连接" : "未连接（超时）";
      RCLCPP_INFO(
        this->get_logger(),
        "初始系统状态: CPU使用率=%.1f%%, CPU温度=%.1f°C, "
        "内存=%.1f%% (%luMB/%luMB), "
        "磁盘=%.1f%% (%luGB/%luGB), "
        "电池%s",
        status.cpu_usage_percent, status.cpu_temperature,
        status.memory_usage_percent,
        static_cast<unsigned long>(status.memory_available_mb),
        static_cast<unsigned long>(status.memory_total_mb),
        status.disk_usage_percent,
        static_cast<unsigned long>(status.disk_available_gb),
        static_cast<unsigned long>(status.disk_total_gb),
        battery_state);
    }
  }

  // 检查各项指标是否超阈值，并设置告警标志。
  checkLimits(status);

  // 发布系统状态。
  status_pub_->publish(status);
}

// 检查系统状态是否超阈值
void RobotMonitorNode::checkLimits(robot_interfaces::msg::SystemStatus & status)
{
  // 每个 check 内部先设置告警标志，再判断是否需要语音播报。

  // CPU 使用率检查。
  if (status.cpu_usage_percent > cpu_usage_max_) {
    status.cpu_usage_over_threshold = true;
    if (voice_cpu_usage_ && alertReady("cpu_usage")) {
      playTip(robot_common::audio_tips::CPU_USAGE_HIGH);
    }
  }

  // CPU 温度检查（温度有效时才对阈值判断）。
  if (status.cpu_temperature > 0.0f && status.cpu_temperature > cpu_temp_max_) {
    status.cpu_temperature_over_threshold = true;
    if (voice_cpu_temp_ && alertReady("cpu_temperature")) {
      playTip(robot_common::audio_tips::CPU_TEMPERATURE_HIGH);
    }
  }

  // 内存占用率检查。
  if (status.memory_usage_percent > mem_usage_max_) {
    status.memory_usage_over_threshold = true;
    if (voice_mem_usage_ && alertReady("memory_usage")) {
      playTip(robot_common::audio_tips::MEMORY_USAGE_HIGH);
    }
  }

  // 磁盘空间检查。
  if (status.disk_usage_percent > disk_usage_max_) {
    status.disk_usage_over_threshold = true;
    if (voice_disk_usage_ && alertReady("disk_usage")) {
      playTip(robot_common::audio_tips::DISK_USAGE_HIGH);
    }
  }

  // 电池 SOC 检查。
  if (status.battery_available && status.battery_soc < batt_soc_min_) {
    status.battery_soc_low_warning = true;
    if (voice_batt_soc_ && alertReady("battery_soc")) {
      playTip(robot_common::audio_tips::BATTERY_SOC_LOW);
    }
  }

  // 电池电压检查。
  if (status.battery_available && status.battery_voltage > 0.0f &&
    status.battery_voltage < batt_voltage_min_)
  {
    status.battery_voltage_low_warning = true;
    if (voice_batt_voltage_ && alertReady("battery_voltage")) {
      playTip(robot_common::audio_tips::BATTERY_VOLTAGE_LOW);
    }
  }

  // 电池 MOS 温度检查（温度有效时才对阈值判断）。
  if (status.battery_available && status.battery_mos_temp > 0.0f &&
    status.battery_mos_temp > batt_mos_temp_max_)
  {
    status.battery_mos_temp_over_threshold = true;
    if (voice_batt_mos_temp_ && alertReady("battery_mos_temp")) {
      playTip(robot_common::audio_tips::BATTERY_MOS_TEMP_HIGH);
    }
  }
}

// 语音播报告警信息
void RobotMonitorNode::playTip(const std::string & key, uint8_t priority)
{
  if (!voice_enabled_) {
    return;
  }

  if (audio_client_) {
    audio_client_->playTip(key, priority);
  }

  RCLCPP_WARN(get_logger(), "语音告警: %s", key.c_str());
}

// 检查告警冷却时间是否已过
bool RobotMonitorNode::alertReady(const std::string & alert_key)
{
  auto it = cooldowns_.find(alert_key);
  if (it == cooldowns_.end()) {
    // 首次触发，创建冷却记录并允许播报。
    AlertCooldown cd;
    cd.last_alert_time = now();
    cd.cooldown_seconds = alert_cooldown_sec_;
    cooldowns_[alert_key] = cd;
    return true;
  }

  auto & cd = it->second;
  const auto elapsed = (now() - cd.last_alert_time).seconds();
  if (elapsed >= cd.cooldown_seconds) {
    cd.last_alert_time = now();
    return true;
  }
  return false;
}

// 处理音频音量设置服务请求
void RobotMonitorNode::handleSetVolume(
  const std::shared_ptr<robot_interfaces::srv::SetVolume::Request> request,
  std::shared_ptr<robot_interfaces::srv::SetVolume::Response> response)
{
  if (!audio_manager_) {
    response->success = false;
    response->message = "音频管理器未初始化";
    response->current_volume_percent = -1.0f;
    response->muted = false;
    return;
  }

  const auto status = audio_manager_->setVolume(request->volume_percent);
  response->success = status.available;
  response->message = status.message;
  response->current_volume_percent = status.volume_percent;
  response->muted = status.muted;
}

// 处理音频静音设置服务请求
void RobotMonitorNode::handleSetMute(
  const std::shared_ptr<robot_interfaces::srv::SetMute::Request> request,
  std::shared_ptr<robot_interfaces::srv::SetMute::Response> response)
{
  if (!audio_manager_) {
    response->success = false;
    response->message = "音频管理器未初始化";
    response->current_volume_percent = -1.0f;
    response->muted = false;
    return;
  }

  const auto status = audio_manager_->setMute(request->muted);
  response->success = status.available;
  response->message = status.message;
  response->current_volume_percent = status.volume_percent;
  response->muted = status.muted;
}

// 处理音频恢复服务请求
void RobotMonitorNode::handleRecoverAudio(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!audio_manager_) {
    response->success = false;
    response->message = "音频管理器未初始化";
    RCLCPP_WARN(get_logger(), "audio recover service rejected: %s", response->message.c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "audio recover service called");
  const auto status = audio_manager_->recoverBackend();
  response->success = status.available;
  response->message = status.message;
  if (response->success) {
    RCLCPP_INFO(get_logger(), "audio recover service completed: %s", response->message.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "audio recover service failed: %s", response->message.c_str());
  }
}

}  // namespace robot_monitor
