#include <array>

#include "gtest/gtest.h"
#include "robot_battery/battery_parser.hpp"

namespace
{

TEST(BatteryParser, ParsesSummaryFrame)
{
  robot_battery::BatteryParser parser;
  const std::array<uint8_t, 8> data{0x01, 0x7C, 0x75, 0x32, 0x01, 0xF4, 0x63, 0x00};

  ASSERT_TRUE(parser.parse(0x04028000, data.data(), data.size()));
  const auto msg = parser.message();

  EXPECT_EQ(msg.total_volt, 380);
  EXPECT_EQ(msg.current, 2);
  EXPECT_EQ(msg.soc, 500);
  EXPECT_EQ(msg.life, 99);
}

TEST(BatteryParser, ParsesPowerAndTemperatureFrame)
{
  robot_battery::BatteryParser parser;
  const std::array<uint8_t, 8> data{0x00, 0x64, 0x00, 0xC8, 65, 66, 67, 3};

  ASSERT_TRUE(parser.parse(0x04038000, data.data(), data.size()));
  const auto msg = parser.message();

  EXPECT_EQ(msg.power, 100);
  EXPECT_EQ(msg.energy, 200);
  EXPECT_EQ(msg.mos_temp, 25);
  EXPECT_EQ(msg.board_temp, 26);
  EXPECT_EQ(msg.heat_temp, 27);
  EXPECT_EQ(msg.heat_current, 3);
}

}  // namespace
