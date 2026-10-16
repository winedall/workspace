#ifndef ROBOT_MONITOR__AUDIO_MANAGER_HPP_
#define ROBOT_MONITOR__AUDIO_MANAGER_HPP_

#include <string>

namespace robot_monitor
{

struct AudioManagerConfig
{
  // 是否启用音量守护；启用后外部绕过 monitor 的音量/静音修改会被恢复。
  bool guard_enabled{true};
  // monitor 期望维持的输出音量。
  float desired_volume_percent{60.0f};
  // 音量比较容差，避免后端四舍五入导致反复设置。
  float volume_tolerance_percent{2.0f};
  // 音频控制后端：auto 优先 pactl，失败后回退 amixer。
  std::string backend{"auto"};
};

struct AudioStatus
{
  // 最近一次读取/设置后端是否成功。
  bool available{false};
  // 当前系统实际输出音量；无法获取时为 -1。
  float volume_percent{-1.0f};
  // 当前系统实际静音状态。
  bool muted{false};
  // monitor 期望维持的音量，通过 setVolume() 更新。
  float desired_volume_percent{60.0f};
  // monitor 期望的静音状态；为 true 时音量守护不会自动取消静音。
  bool desired_muted{false};
  // 最近成功使用的后端或配置后端。
  std::string backend;
  // 后端返回信息或错误说明。
  std::string message;
};

class AudioManager
{
public:
  explicit AudioManager(const AudioManagerConfig & config);

  // 获取当前音频状态。
  AudioStatus getStatus() const;
  // 刷新音频状态，包括读取后端音量/静音状态。
  AudioStatus refreshStatus();
  // 检测实际音量/静音是否偏离期望值，必要时自动恢复。
  AudioStatus pollAndRestore();
  // 设置系统音量，并把该音量保存为后续自动恢复目标。
  AudioStatus setVolume(float volume_percent);
  // 设置期望静音状态；取消静音时恢复到期望音量。
  AudioStatus setMute(bool muted);
  // 重启用户会话 PulseAudio，并恢复期望音量/取消静音。
  AudioStatus recoverBackend();

private:
  struct CommandResult
  {
    int exit_code{1};
    std::string output;
  };

  struct BackendStatus
  {
    // 后端读取是否成功；失败时 message 保存错误输出。
    bool ok{false};
    float volume_percent{-1.0f};
    bool muted{false};
    std::string message;
  };

  // 确保音量在 [0, 100] 范围内。
  static float clampVolume(float volume_percent);
  // 执行系统命令并返回结果。
  static CommandResult runCommand(const std::string & command);
  // 从命令输出中解析百分比值，成功时返回 true，失败返回 false。
  static bool parsePercent(const std::string & text, float & percent);

  // 读取后端音量/静音状态。
  BackendStatus readStatus();
  // 读取 pactl 后端音量/静音状态。
  BackendStatus readPactlStatus();
  // 读取 amixer 后端音量/静音状态。
  BackendStatus readAmixerStatus();
  // 设置后端音量。
  bool setBackendVolume(float volume_percent, std::string & message);
  // 设置后端静音状态。
  bool setBackendMute(bool muted, std::string & message);
  // 设置 pactl 后端音量。
  bool setPactlVolume(float volume_percent, std::string & message);
  // 设置 pactl 后端静音状态。
  bool setPactlMute(bool muted, std::string & message);
  // 设置 amixer 后端音量。
  bool setAmixerVolume(float volume_percent, std::string & message);
  // 设置 amixer 后端静音状态。
  bool setAmixerMute(bool muted, std::string & message);

  // 更新最近一次读取/设置后端音量/静音状态。
  void updateLastStatus(const BackendStatus & status);

  // 音置。
  AudioManagerConfig config_;
  // 最近一次读取/设置后端音量/静音状态。
  AudioStatus last_status_;
};

}  // namespace robot_monitor

#endif  // ROBOT_MONITOR__AUDIO_MANAGER_HPP_
