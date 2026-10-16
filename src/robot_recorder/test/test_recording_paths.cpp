#include "robot_recorder/recording_paths.hpp"

#include <gtest/gtest.h>

namespace robot_recorder
{
namespace
{

TEST(RecordingPaths, UsesPlainCsvNames)
{
  EXPECT_EQ(recordingPath("/tmp/recorder", RecordingFile::kJointStates), "/tmp/recorder/joint_states.csv");
  EXPECT_EQ(
    recordingPath("/tmp/recorder", RecordingFile::kJointCommands),
    "/tmp/recorder/joint_commands.csv");
  EXPECT_EQ(
    recordingPath("/tmp/recorder", RecordingFile::kDesiredCommands),
    "/tmp/recorder/desired_commands.csv");
  EXPECT_EQ(
    recordingPath("/tmp/recorder", RecordingFile::kAcceptedCommands),
    "/tmp/recorder/accepted_commands.csv");
  EXPECT_EQ(recordingPath("/tmp/recorder", RecordingFile::kEvents), "/tmp/recorder/events.csv");
}

TEST(RecordingPaths, UsesPlainEventDirectoryNames)
{
  EXPECT_EQ(eventRecordingDir("/tmp/recorder", 1), "/tmp/recorder/event_0001");
  EXPECT_EQ(eventRecordingDir("/tmp/recorder", 12), "/tmp/recorder/event_0012");
  EXPECT_EQ(eventRecordingDir("/tmp/recorder", 123), "/tmp/recorder/event_0123");
  EXPECT_EQ(eventRecordingDir("/tmp/recorder", 1234), "/tmp/recorder/event_1234");
}

}  // namespace
}  // namespace robot_recorder
