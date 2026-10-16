#ifndef ROBOT_BATTERY__BATTERY_PARSER_HPP_
#define ROBOT_BATTERY__BATTERY_PARSER_HPP_

#include <cstddef>
#include <cstdint>

#include "robot_interfaces/msg/battery_info.hpp"

namespace robot_battery
{

// 解析 BMS CAN 回包，并缓存最近一次电池状态消息。
class BatteryParser
{
public:
  // 清空已解析的数据，恢复为默认空状态。
  void reset();

  // 解析一帧 BMS CAN 数据；识别到支持的帧时返回 true。
  bool parse(uint32_t can_id, const uint8_t * data, std::size_t size);

  // 返回最近一次解析得到的电池状态。
  const robot_interfaces::msg::BatteryInfo & message() const;

  // 返回是否已经收到过关键总信息帧。
  bool hasSummary() const;

private:
  // 按大端格式读取 16 位无符号整数。
  static uint16_t u16be(const uint8_t * data);

  robot_interfaces::msg::BatteryInfo msg_;
  bool has_summary_{false};
};

}  // namespace robot_battery

#endif  // ROBOT_BATTERY__BATTERY_PARSER_HPP_
