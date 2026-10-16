# 驱动状态消息。
# robot_driver 将厂家/总线相关细节归一化后发布该消息，供状态机和诊断模块使用。

# 驱动状态未知，通常表示节点刚启动或尚未收到有效硬件/模拟状态。
uint8 STATUS_UNKNOWN=0
# 驱动离线，例如真实硬件不可达或未初始化。
uint8 STATUS_OFFLINE=1
# 驱动在线，接口可用。
uint8 STATUS_ONLINE=2
# 驱动降级运行，例如部分关节不可用但系统仍可进入受限模式。
uint8 STATUS_DEGRADED=3
# 驱动故障，状态机将进入 FAULT。
uint8 STATUS_FAULT=4

# 标准消息头；stamp 为状态生成时间，frame_id 可按驱动或机器人基准坐标系填写。
std_msgs/Header header

# 当前驱动状态，取值为 STATUS_* 常量。
uint8 status
# 面向日志/诊断的人类可读状态说明。
string status_message

# 驱动侧是否检测到故障状态。该字段用于状态机快速判断 FAULT。
bool fault_active
# 驱动通信是否在线。
bool online

# 配置中应管理的关节总数。
uint32 total_joints
# 当前处于使能或活动状态的关节数量。
uint32 active_joints
# 当前上报故障的关节数量。
uint32 faulted_joints
# 当前故障关节名称列表，名称应与 robot_joints.yaml 中的 name 一致。
string[] faulted_joint_names
