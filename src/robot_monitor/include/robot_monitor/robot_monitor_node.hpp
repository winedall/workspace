#ifndef ROBOT_MONITOR__ROBOT_MONITOR_NODE_HPP_
#define ROBOT_MONITOR__ROBOT_MONITOR_NODE_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "robot_common/audio_client.hpp"
#include "robot_interfaces/msg/battery_info.hpp"
#include "robot_interfaces/srv/set_mute.hpp"
#include "robot_interfaces/srv/set_volume.hpp"
#include "robot_interfaces/msg/system_status.hpp"
#include "robot_interfaces/msg/version_info.hpp"
#include "usbcan_bridge_interface/msg/battery_info.hpp"
#include "robot_monitor/audio_manager.hpp"
#include "robot_monitor/system_monitor.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace robot_monitor
{

// 语音告警冷却状态：同一类型告警在冷却期内不再重复播报。
struct AlertCooldown
{
  rclcpp::Time last_alert_time;
  double cooldown_seconds{30.0};
};

// monitor 内部统一使用的电池样本，隐藏新老电池消息的字段单位差异。
struct BatterySample
{
  float voltage{-1.0f};
  float current{-1.0f};
  float soc{-1.0f};
  float mos_temp{-1.0f};
};

// 系统监控 ROS 2 节点。
// 定时采集 CPU/内存/磁盘/温度等系统信息，订阅电池状态，
// 当指标超过阈值时通过 audio/request 进行语音播报。
class RobotMonitorNode : public rclcpp::Node
{
public:
  explicit RobotMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 定时器回调：采集系统信息并发布 SystemStatus。
  void onMonitor();

  // 音频守护定时器回调：检测系统音量/静音状态并按期望状态恢复。
  void onAudioGuard();

  // 电池信息订阅回调。
  void onRobotBattery(const robot_interfaces::msg::BatteryInfo::SharedPtr msg);

  // 旧版 usbcan_bridge 电池信息订阅回调。
  void onUsbcanBattery(const usbcan_bridge_interface::msg::BatteryInfo::SharedPtr msg);

  // 版本发布定时器回调：每 3 秒发布一次 version。
  void onVersion();

  // 加载版本信息：从 VERSION.txt 解析最新版本号与发布日期。
  void loadVersion();

  // 检查各项指标是否超过阈值，设置告警标志并触发语音播报。
  void checkLimits(robot_interfaces::msg::SystemStatus & status);

  // 发布一条提示音播报请求。
  void playTip(const std::string & key, uint8_t priority = 150);

  // 检查指定类型的告警是否已过冷却期。
  // 冷却期内返回 false，超过冷却期则更新时间戳并返回 true。
  bool alertReady(const std::string & alert_key);

  void handleSetVolume(
    const std::shared_ptr<robot_interfaces::srv::SetVolume::Request> request,
    std::shared_ptr<robot_interfaces::srv::SetVolume::Response> response);

  void handleSetMute(
    const std::shared_ptr<robot_interfaces::srv::SetMute::Request> request,
    std::shared_ptr<robot_interfaces::srv::SetMute::Response> response);

  void handleRecoverAudio(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // 系统信息采集器。
  std::unique_ptr<SystemMonitor> system_monitor_;

  // 系统音频管理器，独立封装音量读写、静音和外部修改恢复逻辑。
  std::unique_ptr<AudioManager> audio_manager_;

  // ---------- 参数 ----------
  double rate_hz_{1.0};
  std::string disk_path_{"/"};
  std::string battery_source_{"robot"};
  bool voice_enabled_{true};
  double alert_cooldown_sec_{30.0};

  // 各告警类型的语音播报独立开关；仅控制播报，不影响告警标志写入。
  bool voice_cpu_usage_{true};
  bool voice_cpu_temp_{true};
  bool voice_mem_usage_{true};
  bool voice_disk_usage_{true};
  bool voice_batt_soc_{true};
  bool voice_batt_voltage_{true};
  bool voice_batt_mos_temp_{true};

  // 阈值参数。
  float cpu_usage_max_{80.0f};
  float cpu_temp_max_{80.0f};
  float mem_usage_max_{90.0f};
  float disk_usage_max_{90.0f};
  float batt_soc_min_{20.0f};
  float batt_voltage_min_{21.0f};
  float batt_mos_temp_max_{80.0f};

  bool audio_guard_{true};
  double audio_rate_hz_{1.0};
  // 以下音频参数只传给 AudioManager，由子模块维护详细状态和恢复策略。
  float audio_volume_{60.0f};
  float audio_volume_tol_{2.0f};
  std::string audio_backend_{"auto"};

  // ---------- ROS 通信 ----------
  rclcpp::Subscription<robot_interfaces::msg::BatteryInfo>::SharedPtr robot_battery_sub_;
  rclcpp::Subscription<usbcan_bridge_interface::msg::BatteryInfo>::SharedPtr usbcan_battery_sub_;
  rclcpp::Publisher<robot_interfaces::msg::SystemStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<robot_interfaces::msg::VersionInfo>::SharedPtr version_pub_;
  std::unique_ptr<robot_common::AudioClient> audio_client_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  rclcpp::TimerBase::SharedPtr audio_guard_timer_;
  rclcpp::TimerBase::SharedPtr version_timer_;
  rclcpp::Service<robot_interfaces::srv::SetVolume>::SharedPtr set_volume_srv_;
  rclcpp::Service<robot_interfaces::srv::SetMute>::SharedPtr set_mute_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr audio_recover_srv_;

  // 最近一次电池数据缓存。
  BatterySample battery_;
  bool have_battery_{false};

  // 版本信息缓存。
  robot_interfaces::msg::VersionInfo version_info_;
  bool version_loaded_{false};
  std::string version_file_;

  // 各告警类型的冷却状态表。
  std::unordered_map<std::string, AlertCooldown> cooldowns_;

  // 首次采集数据是否已打印到日志。
  bool status_logged_{false};

  // 节点启动时间，用于等待电池数据时的超时判断。
  rclcpp::Time start_time_;
};

}  // namespace robot_monitor

#endif  // ROBOT_MONITOR__ROBOT_MONITOR_NODE_HPP_
