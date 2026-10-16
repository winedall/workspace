#include "robot_monitor/audio_manager.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <regex>
#include <sstream>
#include <sys/wait.h>
#include <thread>

namespace robot_monitor
{

AudioManager::AudioManager(const AudioManagerConfig & config)
: config_(config)
{
  // 初始化期望状态：每次启动使用参数默认值。
  config_.desired_volume_percent = clampVolume(config_.desired_volume_percent);
  last_status_.desired_volume_percent = config_.desired_volume_percent;
  last_status_.desired_muted = false;
  last_status_.backend = config_.backend;
  refreshStatus();
}

// 获取当前音频状态。
AudioStatus AudioManager::getStatus() const
{
  return last_status_;
}

// 刷新音频状态，包括读取后端音量/静音状态。
AudioStatus AudioManager::refreshStatus()
{
  const auto status = readStatus();
  updateLastStatus(status);
  return last_status_;
}

// 检测实际音量/静音是否偏离期望值，必要时自动恢复。
AudioStatus AudioManager::pollAndRestore()
{
  const auto status = readStatus();
  updateLastStatus(status);
  if (!config_.guard_enabled || !status.ok) {
    return last_status_;
  }

  std::string message;
  bool changed = false;
  if (last_status_.desired_muted) {
    // 当前期望就是静音：如果外部取消了静音，重新静音。
    if (!status.muted) {
      changed = setBackendMute(true, message);
    }
  } else {
    // 当前期望是正常发声：如果外部静音或改动音量，则恢复到期望音量。
    if (status.muted) {
      changed = setBackendMute(false, message);
    }
    if (std::abs(status.volume_percent - last_status_.desired_volume_percent) > config_.volume_tolerance_percent) {
      changed = setBackendVolume(last_status_.desired_volume_percent, message) || changed;
    }
  }

  if (changed) {
    const auto restored = readStatus();
    updateLastStatus(restored);
  } else if (!message.empty()) {
    last_status_.message = message;
  }
  return last_status_;
}

// 设置系统音量，并把该音量保存为后续自动恢复目标。
AudioStatus AudioManager::setVolume(float volume_percent)
{
  const auto clamped = clampVolume(volume_percent);
  std::string message;
  // 设置音量接口代表用户希望发声，因此同时取消静音。
  const bool volume_ok = setBackendVolume(clamped, message);
  const bool mute_ok = setBackendMute(false, message);
  if (volume_ok && mute_ok) {
    last_status_.desired_volume_percent = clamped;
    last_status_.desired_muted = false;
  }

  const auto status = readStatus();
  updateLastStatus(status);
  if (!volume_ok || !mute_ok) {
    last_status_.available = false;
    last_status_.message = message.empty() ? "设置音量失败" : message;
  }
  return last_status_;
}

// 设置期望静音状态；取消静音时恢复到期望音量。
AudioStatus AudioManager::setMute(bool muted)
{
  std::string message;
  bool ok = true;
  if (muted) {
    // 通过 monitor 主动静音时，守护逻辑会把静音视为期望状态。
    ok = setBackendMute(true, message);
    if (ok) {
      last_status_.desired_muted = true;
    }
  } else {
    // 取消静音时恢复到 monitor 记住的期望音量，避免停留在外部改成的 0 音量。
    ok = setBackendMute(false, message);
    ok = setBackendVolume(last_status_.desired_volume_percent, message) && ok;
    if (ok) {
      last_status_.desired_muted = false;
    }
  }

  const auto status = readStatus();
  updateLastStatus(status);
  if (!ok) {
    last_status_.available = false;
    last_status_.message = message.empty() ? "设置静音状态失败" : message;
  }
  return last_status_;
}

// 重启用户会话 PulseAudio，并恢复期望音量/取消静音。
AudioStatus AudioManager::recoverBackend()
{
  const auto kill_result = runCommand("pulseaudio -k");
  (void)kill_result;
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  const auto start_result = runCommand("pulseaudio --start");
  if (start_result.exit_code != 0) {
    const auto status = readStatus();
    updateLastStatus(status);
    last_status_.available = false;
    last_status_.message = start_result.output.empty() ? "pulseaudio --start failed" : start_result.output;
    return last_status_;
  }

  std::string message;
  bool ok = setBackendMute(false, message);
  ok = setBackendVolume(last_status_.desired_volume_percent, message) && ok;

  const auto status = readStatus();
  updateLastStatus(status);
  if (!ok) {
    last_status_.available = false;
    last_status_.message = message.empty() ? "pulseaudio recovered but audio restore failed" : message;
  } else {
    last_status_.message = "pulseaudio recovered";
  }
  return last_status_;
}

// 对音量值进行范围限制，确保在 0-100% 范围内。
float AudioManager::clampVolume(float volume_percent)
{
  if (volume_percent < 0.0f) {
    return 0.0f;
  }
  if (volume_percent > 100.0f) {
    return 100.0f;
  }
  return volume_percent;
}

// 运行音频控制命令，返回命令执行结果。
AudioManager::CommandResult AudioManager::runCommand(const std::string & command)
{
  CommandResult result;
  std::array<char, 256> buffer{};
  std::string full_command = command + " 2>&1";
  // 后端命令输出需要解析，因此合并 stderr，失败时也能返回可读错误。
  FILE * pipe = popen(full_command.c_str(), "r");
  if (!pipe) {
    result.output = "popen failed";
    return result;
  }
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
    result.output += buffer.data();
  }
  const int rc = pclose(pipe);
  if (rc == -1) {
    result.exit_code = 1;
  } else if (WIFEXITED(rc)) {
    result.exit_code = WEXITSTATUS(rc);
  } else {
    result.exit_code = 1;
  }
  return result;
}

// 解析音频控制命令输出，提取音量值。
bool AudioManager::parsePercent(const std::string & text, float & percent)
{
  // pactl/amixer 都会输出形如 60% 的音量值，这里取第一个百分比。
  const std::regex percent_regex(R"(([0-9]+(?:\.[0-9]+)?)%)");
  std::smatch match;
  if (!std::regex_search(text, match, percent_regex)) {
    return false;
  }
  percent = clampVolume(std::stof(match[1].str()));
  return true;
}

// 读取音频后端状态，包括音量和静音状态。
AudioManager::BackendStatus AudioManager::readStatus()
{
  // 指定后端时只使用指定后端；auto 模式下优先用户会话级 pactl，再回退 ALSA amixer。
  if (config_.backend == "pactl") {
    return readPactlStatus();
  }
  if (config_.backend == "amixer") {
    return readAmixerStatus();
  }

  auto status = readPactlStatus();
  if (status.ok) {
    last_status_.backend = "pactl";
    return status;
  }
  status = readAmixerStatus();
  if (status.ok) {
    last_status_.backend = "amixer";
  }
  return status;
}

// 读取 pactl 后端状态，包括音量和静音状态。
AudioManager::BackendStatus AudioManager::readPactlStatus()
{
  BackendStatus status;
  // pactl 操作默认 sink，适配 PulseAudio 和 PipeWire-pulse 用户会话。
  const auto volume = runCommand("pactl get-sink-volume @DEFAULT_SINK@");
  const auto mute = runCommand("pactl get-sink-mute @DEFAULT_SINK@");
  if (volume.exit_code != 0 || mute.exit_code != 0) {
    status.message = volume.output.empty() ? mute.output : volume.output;
    return status;
  }

  float percent = -1.0f;
  if (!parsePercent(volume.output, percent)) {
    status.message = "无法解析 pactl 音量输出";
    return status;
  }

  status.ok = true;
  status.volume_percent = percent;
  status.muted = mute.output.find("yes") != std::string::npos;
  status.message = "pactl";
  return status;
}

// 读取 amixer 后端状态，包括音量和静音状态。
AudioManager::BackendStatus AudioManager::readAmixerStatus()
{
  BackendStatus status;
  // amixer 作为后备路径，控制 ALSA 的 Master mixer。
  const auto output = runCommand("amixer get Master");
  if (output.exit_code != 0) {
    status.message = output.output;
    return status;
  }

  float percent = -1.0f;
  if (!parsePercent(output.output, percent)) {
    status.message = "无法解析 amixer 音量输出";
    return status;
  }

  status.ok = true;
  status.volume_percent = percent;
  status.muted = output.output.find("[off]") != std::string::npos;
  status.message = "amixer";
  return status;
}

// 设置系统音量，并把该音量保存为后续自动恢复目标。
bool AudioManager::setBackendVolume(float volume_percent, std::string & message)
{
  // auto 模式下分别尝试 pactl 和 amixer，成功后记录最近可用后端。
  if (config_.backend == "pactl") {
    return setPactlVolume(volume_percent, message);
  }
  if (config_.backend == "amixer") {
    return setAmixerVolume(volume_percent, message);
  }
  if (setPactlVolume(volume_percent, message)) {
    last_status_.backend = "pactl";
    return true;
  }
  if (setAmixerVolume(volume_percent, message)) {
    last_status_.backend = "amixer";
    return true;
  }
  return false;
}

// 设置期望静音状态；取消静音时恢复到期望音量。
bool AudioManager::setBackendMute(bool muted, std::string & message)
{
  // mute 控制和音量控制使用同一套后端选择规则。
  if (config_.backend == "pactl") {
    return setPactlMute(muted, message);
  }
  if (config_.backend == "amixer") {
    return setAmixerMute(muted, message);
  }
  if (setPactlMute(muted, message)) {
    last_status_.backend = "pactl";
    return true;
  }
  if (setAmixerMute(muted, message)) {
    last_status_.backend = "amixer";
    return true;
  }
  return false;
}

// 设置 pactl 后端音量。
bool AudioManager::setPactlVolume(float volume_percent, std::string & message)
{
  std::ostringstream cmd;
  cmd << "pactl set-sink-volume @DEFAULT_SINK@ " << static_cast<int>(volume_percent) << "%";
  const auto result = runCommand(cmd.str());
  if (result.exit_code != 0) {
    message = result.output;
    return false;
  }
  return true;
}

// 设置 pactl 后端静音状态。
bool AudioManager::setPactlMute(bool muted, std::string & message)
{
  std::string cmd = std::string("pactl set-sink-mute @DEFAULT_SINK@ ") + (muted ? "1" : "0");
  const auto result = runCommand(cmd);
  if (result.exit_code != 0) {
    message = result.output;
    return false;
  }
  return true;
}

// 设置 amixer 后端音量。
bool AudioManager::setAmixerVolume(float volume_percent, std::string & message)
{
  std::ostringstream cmd;
  cmd << "amixer set Master " << static_cast<int>(volume_percent) << "%";
  const auto result = runCommand(cmd.str());
  if (result.exit_code != 0) {
    message = result.output;
    return false;
  }
  return true;
}

// 设置 amixer 后端静音状态。
bool AudioManager::setAmixerMute(bool muted, std::string & message)
{
  std::string cmd = std::string("amixer set Master ") + (muted ? "mute" : "unmute");
  const auto result = runCommand(cmd);
  if (result.exit_code != 0) {
    message = result.output;
    return false;
  }
  return true;
}

// 更新最近一次读取/设置后端状态。
void AudioManager::updateLastStatus(const BackendStatus & status)
{
  // 后端不可用时保留期望状态，只把实际音量标记为不可读。
  last_status_.available = status.ok;
  if (status.ok) {
    last_status_.volume_percent = status.volume_percent;
    last_status_.muted = status.muted;
  } else {
    last_status_.volume_percent = -1.0f;
  }
  last_status_.message = status.message;
}

}  // namespace robot_monitor
