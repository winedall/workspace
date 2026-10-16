#ifndef ROBOT_DRIVER__BACKEND__BACKEND_FACTORY_HPP_
#define ROBOT_DRIVER__BACKEND__BACKEND_FACTORY_HPP_

#include <memory>

#include "robot_driver/backend/hardware_backend.hpp"

namespace robot_driver
{

// 根据 vendor_type 选择具体 backend。
//
// factory 只负责路由，不解析硬件配置，也不做厂商专用校验。各 backend
// 自己负责各自配置加载和安全状态处理；未知 vendor_type 直接抛出异常，尽早暴露配置错误。
std::unique_ptr<HardwareBackend> createHardwareBackend(const BackendConfig & config);

}  // namespace robot_driver

#endif  // ROBOT_DRIVER__BACKEND__BACKEND_FACTORY_HPP_
