#include "robot_recorder/csv_writer.hpp"

#include <algorithm>
#include <sstream>

namespace robot_recorder
{

/// 构造函数：打开 CSV 文件并写入 header 行。
/// 打开时启用异常，后续 writeRow/flush 出错时抛出异常便于上层处理。
CsvWriter::CsvWriter(const std::string & file_path, const std::vector<std::string> & headers)
{
  file_.exceptions(std::ofstream::failbit | std::ofstream::badbit);
  file_.open(file_path);
  // 逐字段写入 header，字段间以逗号分隔，末尾换行
  for (size_t i = 0; i < headers.size(); ++i) {
    if (i > 0) {
      file_ << ',';
    }
    file_ << headers[i];
  }
  file_ << '\n';
}

/// 析构函数：关闭文件，确保资源释放。
CsvWriter::~CsvWriter()
{
  if (file_.is_open()) {
    file_.close();
  }
}

/// 写入一行 CSV 数据。字段数应与 header 数一致，字段自动转义。
void CsvWriter::writeRow(const std::vector<std::string> & fields)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < fields.size(); ++i) {
    if (i > 0) {
      file_ << ',';
    }
    file_ << escape(fields[i]);
  }
  file_ << '\n';
}

/// 刷新文件输出缓冲区，确保数据已写入磁盘。
void CsvWriter::flush()
{
  std::lock_guard<std::mutex> lock(mutex_);
  file_ << std::flush;
}

/// 转义 CSV 字段中的特殊字符。
/// 标准 CSV 规则：若字段含逗号、双引号或换行符，需用双引号包裹，
/// 字段内部的双引号需要重复两次（""）转义。
std::string CsvWriter::escape(const std::string & field)
{
  // 检查是否需要引号包裹
  const bool needs_quoting =
    field.find(',') != std::string::npos ||
    field.find('"') != std::string::npos ||
    field.find('\n') != std::string::npos ||
    field.find('\r') != std::string::npos;
  if (!needs_quoting) {
    return field;
  }
  // 对字段进行转义：用双引号包裹，内部的 " 替换为 ""
  std::string escaped;
  escaped.reserve(field.size() + 2);
  escaped += '"';
  for (const char c : field) {
    if (c == '"') {
      escaped += "\"\"";
    } else {
      escaped += c;
    }
  }
  escaped += '"';
  return escaped;
}

}  // namespace robot_recorder
