#ifndef ROBOT_RECORDER__CSV_WRITER_HPP_
#define ROBOT_RECORDER__CSV_WRITER_HPP_

#include <fstream>
#include <mutex>
#include <string>
#include <vector>

namespace robot_recorder
{

/// 线程安全的 CSV 文件写入器。
/// 构造函数写入 header 行，writeRow 写入数据行，flush 刷新缓冲区。
/// 字符串字段中的逗号、双引号、换行符会自动转义。
class CsvWriter
{
public:
  /// 打开文件并写入 header 行。
  CsvWriter(const std::string & file_path, const std::vector<std::string> & headers);

  ~CsvWriter();

  /// 写入一行数据，字段数量必须与 header 一致。
  void writeRow(const std::vector<std::string> & fields);

  /// 刷新文件缓冲区，确保数据落盘。
  void flush();

  /// 文件流是否正常。
  bool ok() const { return file_.good(); }

private:
  /// 转义字段中的特殊字符（逗号、引号、换行）。
  static std::string escape(const std::string & field);

  std::mutex mutex_;       ///< 写操作互斥锁，支持多线程安全写入
  std::ofstream file_;     ///< 底层文件输出流
};

}  // namespace robot_recorder

#endif  // ROBOT_RECORDER__CSV_WRITER_HPP_
