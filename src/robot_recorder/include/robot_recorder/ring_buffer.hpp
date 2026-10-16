#ifndef ROBOT_RECORDER__RING_BUFFER_HPP_
#define ROBOT_RECORDER__RING_BUFFER_HPP_

#include <algorithm>
#include <cstdint>
#include <mutex>
#include <utility>
#include <vector>

namespace robot_recorder
{

/// 固定容量环形缓冲区（线程安全）
/// 容量满时 tryPush 返回 false（非阻塞），适合写多读少的场景
template<typename T>
class RingBuffer
{
public:
  explicit RingBuffer(size_t capacity)
  : buffer_(capacity), capacity_(capacity)
  {
  }

  /// 尝试推入一条记录。如果缓冲区已满，返回 false 且不阻塞调用方
  bool tryPush(const T & item)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (size_ >= capacity_) {
      return false;
    }
    buffer_[(head_ + size_) % capacity_] = item;
    ++size_;
    return true;
  }

  /// 取出所有记录并清空缓冲区
  /// 返回的记录按推入顺序排列
  std::vector<T> drain()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<T> result;
    result.reserve(size_);
    for (size_t i = 0; i < size_; ++i) {
      result.push_back(buffer_[(head_ + i) % capacity_]);
    }
    head_ = 0;
    size_ = 0;
    return result;
  }

  size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return size_;
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    head_ = 0;
    size_ = 0;
  }

private:
  std::vector<T> buffer_;     // 底层环形数组
  size_t capacity_;           // 最大容量
  size_t head_{0};            // 环形队列头部索引
  size_t size_{0};            // 当前元素个数
  mutable std::mutex mutex_;  // 所有 public 方法的互斥锁
};

/// 时间窗口缓冲区（线程安全）
/// 自动淘汰超出时间窗口的旧记录（基于 recorder_time_ms）
template<typename T>
class TimeWindowBuffer
{
public:
  explicit TimeWindowBuffer(double window_sec)
  : window_sec_(window_sec)
  {
  }

  /// 推入一条记录，推入后自动淘汰超出窗口的旧记录
  void push(T rec)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.push_back(std::move(rec));
    prune();
  }

  /// 取出所有记录并清空缓冲区
  std::vector<T> drain()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<T> result;
    result.swap(buffer_);
    return result;
  }

  size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.size();
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.clear();
  }

private:
  /// 淘汰超时旧记录：保留最近 window_sec_ 秒内的记录
  /// 缓冲区按 recorder_time_ms 有序，使用二分查找定位分界点
  void prune()
  {
    if (buffer_.empty()) {
      return;
    }
    const int64_t cutoff = buffer_.back().recorder_time_ms -
      static_cast<int64_t>(window_sec_ * 1000.0);
    auto it = std::lower_bound(
      buffer_.begin(), buffer_.end(), cutoff,
      [](const T & rec, int64_t val) {
        return rec.recorder_time_ms < val;
      });
    buffer_.erase(buffer_.begin(), it);
  }

  double window_sec_;          // 时间窗口，单位秒
  std::vector<T> buffer_;      // 底层动态数组，尾部插入，头部淘汰
  mutable std::mutex mutex_;   // 所有 public 方法的互斥锁
};

}  // namespace robot_recorder

#endif  // ROBOT_RECORDER__RING_BUFFER_HPP_
