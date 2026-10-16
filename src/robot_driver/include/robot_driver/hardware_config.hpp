#ifndef ROBOT_DRIVER__HARDWARE_CONFIG_HPP_
#define ROBOT_DRIVER__HARDWARE_CONFIG_HPP_

#include <string>
#include <unordered_map>
#include <vector>

namespace robot_driver
{

// EtherCAT 总线级配置。当前为 Encos/Tienkung 这类 EtherCAT 后端预留。
struct EthercatBusConfig
{
  // 网卡名，例如 enp3s0。真实 EtherCAT backend 会要求非空。
  std::string network_interface;
  // EtherCAT 控制周期，单位 ms。未配置时为 0，由真实 backend 决定默认值或拒绝启动。
  double cycle_time_ms{0.0};
  // 预期 EtherCAT 从站（子板）数量。真实 EtherCAT backend 要求显式配置。
  int slave_count{0};
};

// 串口总线级配置。当前为 Unitree 等串口后端预留。
struct SerialBusConfig
{
  // 串口设备路径，例如 /dev/ttyUSB0。
  std::string device;
  // 串口波特率。
  int baudrate{0};
};

// 单路 SocketCAN 接口配置。
//
// bus 是机器人内部逻辑总线号；name 是 Linux CAN 网络接口名，例如 can0。
// 关节通过 joints[*].canfd.bus 绑定到这里的 bus。
struct CanfdInterfaceConfig
{
  // 逻辑 CANFD 总线号，通常为 0..3。
  int bus{0};
  // SocketCAN 接口名，例如 can0/can1/can2/can3。
  std::string name;
};

// CANFD 总线级配置。当前只用于 vendor_type=mix、bus_type=canfd。
struct CanfdBusConfig
{
  // CANFD 控制周期，单位 ms。未配置时由 MixBackend 使用默认周期。
  double cycle_time_ms{0.0};
  // 每周期发送后接收反馈的有界等待窗口，单位 ms。支持小数，0 表示只做非阻塞收割。
  double poll_timeout_ms{0.0};
  // 可用 CANFD 接口列表，支持 4 路或更多配置，实际拓扑由 YAML 决定。
  std::vector<CanfdInterfaceConfig> interfaces;
};

// 单关节 CANFD 位置配置。
struct CanfdJointConfig
{
  // 该关节所属逻辑 CANFD 总线号，对应 hardware.canfd.interfaces[*].bus。
  int bus{-1};
};

// EtherCAT-CANFD 网关中的关节槽位配置。
//
// mix/ethercat 模式下，厂家协议仍然产出通用 CanfdFrame；gateway 字段只描述
// 这帧 CANFD 应写入哪个 EtherCAT 从站、哪一路 CAN-FD 总线、哪个槽位。
struct GatewayJointConfig
{
  // EtherCAT 逻辑从站编号。IGH 路径沿用现场拓扑的 1 基编号。
  int slave_id{0};
  // 网关内部 CAN-FD 总线编号，当前 0=CAN-FD0，1=CAN-FD1。
  int bus{-1};
  // 网关 CAN-FD 槽位编号，当前每路 0..6。
  int slot{-1};
};

// Encos/EtherCAT 单关节扩展字段。
//
// YAML 中仍保持一行一个关节的扁平写法；解析后把这些字段归入 encos 子结构，
// 避免它们被误认为所有厂商都需要的通用字段。
struct EncosJointConfig
{
  // EtherCAT 从站编号或板卡编号。
  int slave_id{0};
  // 从站内通道号。
  int passage{0};
  // 力矩系数，供真实 Encos backend 做 effort/current/torque 换算。
  double kt{0.0};
  // Encos 协议中各物理量的量化范围，完全按关节平铺配置。
  double kp_min{0.0};
  double kp_max{0.0};
  double kd_min{0.0};
  double kd_max{0.0};
  double pos_min{0.0};
  double pos_max{0.0};
  double spd_min{0.0};
  double spd_max{0.0};
  double tor_min{0.0};
  double tor_max{0.0};
  double cur_min{0.0};
  double cur_max{0.0};
};

// Tienkung/EtherCAT 网关单关节扩展字段。
//
// 天工这套硬件不是传统“一个 slave 对一个电机”的平铺映射，而是把每个
// 逻辑关节落到网关里的某个 CAN-FD bus/slot 上，再由 IGH 主站把 PDO 数据
// 送到对应从站。这里把厂商私有字段单独收拢，方便和 Encos 配置并列阅读。
struct TienkungJointConfig
{
  // EtherCAT 逻辑从站编号，配置文件里使用 1 基编号，和现场拓扑表保持一致。
  int slave_id{0};
  // CAN-FD 总线编号：0=CAN-FD0，1=CAN-FD1。天工网关按这个字段区分两路 CAN-FD。
  int bus{0};
  // CAN-FD 槽位编号，当前网关每路 0..6；同一条 bus 内必须唯一。
  int slot{0};
  // 当前天工运动控制默认使用 CAN-FD。这里保留成字符串，是为了后续协议扩展
  // 时能在配置层继续明确标识模式。
  std::string can_mode{"canfd"};
  // 默认反馈报文类型 0x80。天工目前只接受默认状态帧，因此配置里固定校验。
  std::string feedback_type{"default"};
  // 力矩系数，供 current/effort 换算；编码和解码都依赖这个值。
  double kt{0.0};
};

// 单个逻辑关节到具体硬件通道的映射。
//
// 通用字段放在顶层，厂商/总线特有字段放到 encos 等扩展结构中。
struct HardwareJointConfig
{
  // 逻辑关节名，必须与 robot_joints.yaml 和 JointCommand.joint_name 保持一致。
  std::string name;
  // mix/canfd 模式下的单关节厂家类型；单厂家模式默认等于 hardware.vendor_type。
  std::string vendor;
  // 电机编号。保留 motor_id 命名以兼容现有 hw_encos.yaml 和旧驱动语义。
  int motor_id{0};
  // 方向修正，允许值为 +1 或 -1。
  int direction{1};
  // 硬件零位偏移，当前沿用旧配置中的度单位，换算封装在 driver 层。
  double offset{0.0};
  // 机械减速比，关节空间到电机物理空间的缩放系数。非 1.0 时，位置/速度/力矩会按此比例缩放。例如 calf_joint 减速比为 1.5。
  double ratio{1.0};
  // 串口类厂家可选字段，例如 Unitree 电机型号。
  std::string motor_type;
  // CANFD 总线定位字段。mix/canfd 模式下必填。
  CanfdJointConfig canfd;
  // EtherCAT-CANFD 网关定位字段。mix/ethercat 模式下必填。
  GatewayJointConfig gateway;
  // Encos/EtherCAT 可选字段。
  EncosJointConfig encos;
  // Tienkung/EtherCAT 可选字段。
  TienkungJointConfig tienkung;
};

// 驱动硬件配置。hardware.joints 仍支持“一行一个关节”的 YAML 风格。
struct HardwareConfig
{
  // 厂商类型，例如 encos、tienkung、unitree。未配置时可由 launch 参数 vendor_type 兜底。
  std::string vendor_type;
  // 总线类型，例如 ethercat、serial、canfd。
  std::string bus_type;
  // EtherCAT 总线配置。
  EthercatBusConfig ethercat;
  // 串口总线配置。
  SerialBusConfig serial;
  // CANFD 总线配置，仅 mix/canfd 后端使用；旧 EtherCAT 后端不读取该字段。
  CanfdBusConfig canfd;
  // 按配置文件顺序保存硬件关节映射，便于诊断和保持可预测遍历顺序。
  std::vector<HardwareJointConfig> joints;
  // joint_name 到配置项的索引表，用于快速校验和查找。
  std::unordered_map<std::string, HardwareJointConfig> joint_map;

  // 判断硬件映射中是否包含指定逻辑关节。
  bool hasJoint(const std::string & name) const;
  // 返回指定逻辑关节的硬件配置；不存在时返回 nullptr。
  const HardwareJointConfig * getJoint(const std::string & name) const;
  // 返回硬件配置中的全部关节名，顺序与 joints 数组保持一致。
  std::vector<std::string> jointNames() const;
};

// 加载并校验硬件映射文件。
//
// 当前只负责结构和字段合法性检查，不初始化 SOEM/串口，也不访问真实硬件。
HardwareConfig loadHardwareConfig(const std::string & path);

}  // namespace robot_driver

#endif  // ROBOT_DRIVER__HARDWARE_CONFIG_HPP_
