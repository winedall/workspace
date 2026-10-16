#ifndef ROBOT_RECORDER__RECORDING_PATHS_HPP_
#define ROBOT_RECORDER__RECORDING_PATHS_HPP_

#include <iomanip>
#include <sstream>
#include <string>

namespace robot_recorder
{

/// recorder 会写出的固定文件类型。
/// 文件名不再携带 run id；一次运行的身份由 output_dir 的外层目录表达。
enum class RecordingFile
{
  kJointStates,
  kJointCommands,
  kDesiredCommands,
  kAcceptedCommands,
  kEvents,
};

/// 拼接 recorder 输出目录和文件/子目录名。
/// 这里统一处理目录末尾是否已有 '/'，避免调用处重复做字符串判断。
inline std::string joinRecordingPath(const std::string & dir, const std::string & name)
{
  if (dir.empty() || dir.back() == '/') {
    return dir + name;
  }
  return dir + "/" + name;
}

/// 根据固定文件类型返回 CSV 文件名。
/// continuous 模式和 event_window 模式共用这套命名规则。
inline std::string recordingFileName(const RecordingFile file)
{
  switch (file) {
    case RecordingFile::kJointStates:
      return "joint_states.csv";
    case RecordingFile::kJointCommands:
      return "joint_commands.csv";
    case RecordingFile::kDesiredCommands:
      return "desired_commands.csv";
    case RecordingFile::kAcceptedCommands:
      return "accepted_commands.csv";
    case RecordingFile::kEvents:
      return "events.csv";
  }
  return "unknown.csv";
}

/// 返回某个 recorder CSV 文件在指定目录下的完整路径。
inline std::string recordingPath(const std::string & dir, const RecordingFile file)
{
  return joinRecordingPath(dir, recordingFileName(file));
}

/// 返回单次事件窗口录制目录名，例如 event_0001。
/// event_id 来自 EventManager，目录名只表达事件序号，不再重复外层 run id。
inline std::string eventRecordingDir(const std::string & output_dir, const int event_id)
{
  std::ostringstream suffix;
  suffix << "event_" << std::setw(4) << std::setfill('0') << event_id;
  return joinRecordingPath(output_dir, suffix.str());
}

/// 将命令流后缀映射为对应 CSV 类型。
/// suffix 来自 recorder 内部 topic 分流：desired / accepted / joint。
inline RecordingFile commandRecordingFile(const std::string & suffix)
{
  if (suffix == "desired") {
    return RecordingFile::kDesiredCommands;
  }
  if (suffix == "accepted") {
    return RecordingFile::kAcceptedCommands;
  }
  return RecordingFile::kJointCommands;
}

}  // namespace robot_recorder

#endif  // ROBOT_RECORDER__RECORDING_PATHS_HPP_
