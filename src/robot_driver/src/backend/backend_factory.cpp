#include "robot_driver/backend/backend_factory.hpp"

#include <algorithm>
#include <cctype>
#include <memory>
#include <stdexcept>
#include <string>

#include "robot_driver/backend/encos_backend.hpp"
#include "robot_driver/backend/mix_backend.hpp"
#include "robot_driver/backend/tienkung_backend.hpp"

namespace robot_driver
{

namespace
{

// 将 vendor_type 统一转换成小写，避免 launch/参数文件中出现大小写差异导致匹配失败。
std::string normalizeVendor(std::string vendor)
{
  std::transform(vendor.begin(), vendor.end(), vendor.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return vendor;
}

}  // namespace

// 根据 hardware.vendor_type 创建具体硬件后端。
//
// 这里仅做厂商路由；硬件配置合法性、通讯初始化和安全门控都留给具体 backend 完成。
std::unique_ptr<HardwareBackend> createHardwareBackend(const BackendConfig & config)
{
  // 根据厂商类型选择对应 backend；这里不做协议细节判断，只负责路由。
  const auto vendor = normalizeVendor(config.hardware_config.vendor_type);
  if (vendor.empty()) {
    throw std::runtime_error("robot_driver hardware.vendor_type is empty");
  }
  if (vendor == "encos") {
    // Encos backend 内部负责真实链路诊断和受控写入门控。
    return std::make_unique<EncosBackend>(config);
  }
  if (vendor == "tienkung") {
    return std::make_unique<TienkungBackend>(config);
  }
  if (vendor == "mix") {
    return std::make_unique<MixBackend>(config);
  }

  // 未实现的 vendor_type 是启动配置错误，直接 fail-fast，避免用假 backend 掩盖问题。
  throw std::runtime_error("unsupported robot_driver hardware.vendor_type: " + vendor);
}

}  // namespace robot_driver
