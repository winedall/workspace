#include <string>

#include <gtest/gtest.h>

#include "robot_driver/backend/hardware_backend.hpp"
#include "robot_driver/hardware_config.hpp"

namespace robot_driver
{
namespace
{

BackendConfig makeConfig()
{
  const std::string workspace = ROBOT_DRIVER_TEST_WORKSPACE;
  BackendConfig config;
  config.robot_joints_config = workspace + "/config/humanoid/robot_joints_encos.yaml";
  config.hardware_config = loadHardwareConfig(workspace + "/config/humanoid/hw_encos.yaml");
  return config;
}

class FaultDetailBackend : public HardwareBackend
{
public:
  FaultDetailBackend()
  : HardwareBackend(makeConfig(), "test_backend")
  {
  }

  void triggerDetailedFault()
  {
    FaultDetail detail;
    detail.source = "ethercat_exchange";
    detail.message = "process data exchange failed";
    detail.transport = "ethercat";
    detail.joint_name = "left_elbow";
    detail.expected_work_counter = 12;
    detail.actual_work_counter = 3;
    detail.transport_error = "EtherCAT workcounter abnormal";
    detail.slave_summary = "slave1{name=drive,state=4,al=30,online=false,lost=false}";
    recordFault(detail);
  }

protected:
  bool transportReady() const override { return false; }
  bool exchangeProcessData() override { return false; }
  bool encodeJointCommand(const robot_interfaces::msg::JointCommand &) override { return false; }
  void readJointFeedback() override {}
  bool configured() const override { return false; }
};

TEST(FaultDetailTest, DriverStatusIncludesStructuredFaultContext)
{
  FaultDetailBackend backend;

  backend.triggerDetailedFault();

  const auto status = backend.driverStatus();
  EXPECT_TRUE(status.fault_active);
  EXPECT_NE(status.status_message.find("test_backend fault"), std::string::npos);
  EXPECT_NE(status.status_message.find("source=ethercat_exchange"), std::string::npos);
  EXPECT_NE(status.status_message.find("joint=left_elbow"), std::string::npos);
  EXPECT_NE(status.status_message.find("wkc=3/12"), std::string::npos);
  EXPECT_NE(status.status_message.find("EtherCAT workcounter abnormal"), std::string::npos);
  EXPECT_NE(status.status_message.find("slave1"), std::string::npos);
}

}  // namespace
}  // namespace robot_driver
