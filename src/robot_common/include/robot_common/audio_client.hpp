#ifndef ROBOT_COMMON__AUDIO_CLIENT_HPP_
#define ROBOT_COMMON__AUDIO_CLIENT_HPP_

#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/audio_request.hpp"

namespace robot_common
{

class AudioClient
{
public:
  explicit AudioClient(
    rclcpp::Node & node,
    const std::string & topic = "audio/request",
    const rclcpp::QoS & qos = rclcpp::QoS(10).transient_local());

  void playTip(const std::string & key, uint8_t priority = 100, bool interrupt = false);
  void playFile(const std::string & name, uint8_t priority = 100, bool interrupt = false);
  void speakText(const std::string & text, uint8_t priority = 100, bool interrupt = false);

private:
  void publish(uint8_t kind, const std::string & name, uint8_t priority, bool interrupt);

  rclcpp::Publisher<robot_interfaces::msg::AudioRequest>::SharedPtr publisher_;
};

}  // namespace robot_common

#endif  // ROBOT_COMMON__AUDIO_CLIENT_HPP_
