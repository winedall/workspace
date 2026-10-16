#include "robot_battery/battery_parser.hpp"

namespace robot_battery
{

// 清空解析器缓存，重新等待总信息帧。
void BatteryParser::reset()
{
  msg_ = robot_interfaces::msg::BatteryInfo{};
  has_summary_ = false;
}

// 按 BMS 协议解析一帧 CAN 数据，支持 BatteryInfo 中保留的字段。
bool BatteryParser::parse(uint32_t can_id, const uint8_t * data, std::size_t size)
{
  if (data == nullptr || size < 8) {
    return false;
  }

  const uint32_t id_base = can_id & 0xFFFFFF00U;
  switch (id_base) {
    case 0x04028000U:
      msg_.total_volt = u16be(&data[0]);
      msg_.current = static_cast<int16_t>(u16be(&data[2])) - 30000;
      msg_.soc = u16be(&data[4]);
      msg_.life = data[6];
      has_summary_ = true;
      return true;

    case 0x04038000U:
      msg_.power = u16be(&data[0]);
      msg_.energy = u16be(&data[2]);
      msg_.mos_temp = static_cast<int8_t>(data[4]) - 40;
      msg_.board_temp = static_cast<int8_t>(data[5]) - 40;
      msg_.heat_temp = static_cast<int8_t>(data[6]) - 40;
      msg_.heat_current = data[7];
      return true;

    case 0x04048000U:
      msg_.max_volt = u16be(&data[0]);
      msg_.max_volt_no = data[2];
      msg_.min_volt = u16be(&data[3]);
      msg_.min_volt_no = data[5];
      msg_.diff_volt = u16be(&data[6]);
      return true;

    case 0x04058000U:
      msg_.max_temp = static_cast<int8_t>(data[0]) - 40;
      msg_.max_temp_no = data[1];
      msg_.min_temp = static_cast<int8_t>(data[2]) - 40;
      msg_.min_temp_no = data[3];
      msg_.diff_temp = data[4];
      return true;

    case 0x04068000U:
      msg_.chg_mos = data[0];
      msg_.dis_mos = data[1];
      msg_.pre_mos = data[2];
      msg_.heat_mos = data[3];
      msg_.fan_mos = data[4];
      return true;

    case 0x04078000U:
      msg_.bat_state = data[0];
      msg_.chg_detect = data[1];
      msg_.load_detect = data[2];
      msg_.do_state = data[3];
      msg_.di_state = data[4];
      return true;

    case 0x04088000U:
      msg_.cell_num = data[0];
      msg_.ntc_num = data[1];
      msg_.remain_capacity =
        (static_cast<uint32_t>(data[2]) << 24) |
        (static_cast<uint32_t>(data[3]) << 16) |
        (static_cast<uint32_t>(data[4]) << 8) |
        static_cast<uint32_t>(data[5]);
      msg_.cycle_count = u16be(&data[6]);
      return true;

    case 0x040B8000U:
      msg_.rest_charge_time = u16be(&data[0]);
      msg_.wake_source = data[2];
      return true;

    case 0x040D8000U:
      msg_.limit_state = data[0];
      msg_.limit_current = static_cast<int16_t>(u16be(&data[1])) - 30000;
      msg_.soh = u16be(&data[3]);
      msg_.pwm = u16be(&data[5]);
      return true;

    default:
      return false;
  }
}

// 返回最近一次解析后的电池状态消息。
const robot_interfaces::msg::BatteryInfo & BatteryParser::message() const
{
  return msg_;
}

// 判断是否已经收到总压、电流、SOC 等关键总信息。
bool BatteryParser::hasSummary() const
{
  return has_summary_;
}

// 按协议使用的大端字节序读取 16 位无符号值。
uint16_t BatteryParser::u16be(const uint8_t * data)
{
  return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

}  // namespace robot_battery
