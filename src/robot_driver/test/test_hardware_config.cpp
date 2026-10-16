#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "robot_driver/hardware_config.hpp"

namespace robot_driver
{
namespace
{

std::string writeTempConfig(const std::string & name, const std::string & content)
{
  const auto path = std::string("/tmp/") + name;
  std::ofstream stream(path);
  stream << content;
  return path;
}

TEST(HardwareConfigTest, MixEthercatUsesGatewayMapping)
{
  const auto path = writeTempConfig(
    "robot_driver_mix_ethercat_gateway.yaml",
    R"(
hardware:
  vendor_type: mix
  bus_type: ethercat
  ethercat:
    network_interface: eth1
    cycle_time_ms: 1.25
    slave_count: 1
  joints:
    - {name: left_shoulder_pitch, vendor: tienkung, motor_id: 11, direction: +1, offset: -154.0, ratio: 1.0, gateway: {slave_id: 1, bus: 0, slot: 0}, tienkung: {kt: 1.37}}
    - {name: left_wrist_pitch, vendor: encos, motor_id: 16, direction: -1, offset: 80.0, ratio: 1.0, gateway: {slave_id: 1, bus: 0, slot: 5}, encos: {kt: 1.35, kp_min: 0.0, kp_max: 500.0, kd_min: 0.0, kd_max: 5.0, pos_min: -12.5, pos_max: 12.5, spd_min: -18.0, spd_max: 18.0, tor_min: -12.0, tor_max: 12.0, cur_min: -10.0, cur_max: 10.0}}
)");

  const auto config = loadHardwareConfig(path);

  ASSERT_EQ(config.joints.size(), 2u);
  EXPECT_EQ(config.vendor_type, "mix");
  EXPECT_EQ(config.bus_type, "ethercat");
  EXPECT_EQ(config.joints[0].gateway.slave_id, 1);
  EXPECT_EQ(config.joints[0].gateway.bus, 0);
  EXPECT_EQ(config.joints[0].gateway.slot, 0);
  EXPECT_EQ(config.joints[1].gateway.slave_id, 1);
  EXPECT_EQ(config.joints[1].gateway.bus, 0);
  EXPECT_EQ(config.joints[1].gateway.slot, 5);
}

TEST(HardwareConfigTest, TemplateFilesLoad)
{
  const std::string workspace = ROBOT_DRIVER_TEST_WORKSPACE;

  const auto encos = loadHardwareConfig(workspace + "/config/humanoid/hw_encos.yaml");
  EXPECT_EQ(encos.vendor_type, "encos");
  EXPECT_EQ(encos.bus_type, "ethercat");
  EXPECT_FALSE(encos.joints.empty());

  const auto tienkung = loadHardwareConfig(workspace + "/config/humanoid/hw_tienkung.yaml");
  EXPECT_EQ(tienkung.vendor_type, "tienkung");
  EXPECT_EQ(tienkung.bus_type, "ethercat");
  EXPECT_FALSE(tienkung.joints.empty());

  const auto canfd = loadHardwareConfig(workspace + "/config/humanoid/hw_mix_canfd.yaml");
  EXPECT_EQ(canfd.vendor_type, "mix");
  EXPECT_EQ(canfd.bus_type, "canfd");
  EXPECT_FALSE(canfd.joints.empty());

  const auto ethercat = loadHardwareConfig(workspace + "/config/humanoid/hw_mix_ethercat.yaml");
  EXPECT_EQ(ethercat.vendor_type, "mix");
  EXPECT_EQ(ethercat.bus_type, "ethercat");
  EXPECT_EQ(ethercat.joints.size(), canfd.joints.size());
}

}  // namespace
}  // namespace robot_driver
