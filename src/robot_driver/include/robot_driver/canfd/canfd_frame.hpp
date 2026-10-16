#ifndef ROBOT_DRIVER__CANFD__CANFD_FRAME_HPP_
#define ROBOT_DRIVER__CANFD__CANFD_FRAME_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace robot_driver
{

// 当前厂家控制协议使用的 CANFD 数据区长度。
//
// 参考代码和现场协议里，天工控制/反馈帧使用 16 字节，Encos 控制/反馈帧使用 8 字节。
// 这里统一按 16 字节建模，避免在 mix backend 中暴露厂家差异。
constexpr std::size_t kCanfdMaxDataLength = 16;

// 单个电机的 CANFD 帧信箱。
//
// CanfdMaster 只负责把该结构送到 socketcan，或把收到的 socketcan 帧写回该结构；
// id/length/data 的协议含义由 EncosProtocol/TienkungProtocol 解释。
struct CanfdFrame
{
  std::uint32_t id{0};
  std::uint8_t length{0};
  std::array<std::uint8_t, kCanfdMaxDataLength> data{};
};

}  // namespace robot_driver

#endif  // ROBOT_DRIVER__CANFD__CANFD_FRAME_HPP_
