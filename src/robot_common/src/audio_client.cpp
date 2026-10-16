#include "robot_common/audio_client.hpp"

namespace robot_common
{

AudioClient::AudioClient(
  rclcpp::Node & node,
  const std::string & topic,
  const rclcpp::QoS & qos)
{
  publisher_ = node.create_publisher<robot_interfaces::msg::AudioRequest>(topic, qos);
}

void AudioClient::playTip(const std::string & key, uint8_t priority, bool interrupt)
{
  publish(robot_interfaces::msg::AudioRequest::KIND_TIP, key, priority, interrupt);
}

void AudioClient::playFile(const std::string & name, uint8_t priority, bool interrupt)
{
  publish(robot_interfaces::msg::AudioRequest::KIND_FILE, name, priority, interrupt);
}

void AudioClient::speakText(const std::string & text, uint8_t priority, bool interrupt)
{
  publish(robot_interfaces::msg::AudioRequest::KIND_TEXT, text, priority, interrupt);
}

void AudioClient::publish(uint8_t kind, const std::string & name, uint8_t priority, bool interrupt)
{
  robot_interfaces::msg::AudioRequest msg;
  msg.kind = kind;
  msg.name = name;
  msg.priority = priority;
  msg.interrupt = interrupt;
  publisher_->publish(msg);
}

}  // namespace robot_common
