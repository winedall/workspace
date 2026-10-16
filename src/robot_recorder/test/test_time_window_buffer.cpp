#include "robot_recorder/ring_buffer.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace robot_recorder
{
namespace
{

struct TestRecord
{
  int64_t recorder_time_ms{0};
  int value{0};
};

TEST(TimeWindowBufferTest, KeepsOnlyRecordsInsideWindow)
{
  TimeWindowBuffer<TestRecord> buffer(3.0);

  buffer.push({1000, 1});
  buffer.push({2500, 2});
  buffer.push({4500, 3});

  const auto records = buffer.drain();

  ASSERT_EQ(records.size(), 2u);
  EXPECT_EQ(records[0].value, 2);
  EXPECT_EQ(records[1].value, 3);
}

}  // namespace
}  // namespace robot_recorder
