# 机器人系统状态。
# robot_state 是系统状态唯一权威，发布该消息供其他模块做准入判断。

# 节点启动中，参数和接口尚未完全初始化。
uint8 STATE_BOOTING=0
# 初始化阶段，加载配置、模型或驱动状态。
uint8 STATE_INIT=1
# 空闲状态，不输出运动命令。
uint8 STATE_IDLE=2
# 准备执行中：正在执行一次性的起身/准备动作插值，完成后进入 READY。
uint8 STATE_DO_READY=3
# 准备完成，可以接受高层控制意图。
uint8 STATE_READY=4
# RL 模型已准备但尚未输出控制。
uint8 STATE_RL_STANDBY=5
# RL 控制运行中。
uint8 STATE_RL_ACTIVE=6
# 规则控制器已准备但尚未输出控制。
uint8 STATE_RULE_STANDBY=7
# 规则控制运行中。
uint8 STATE_RULE_ACTIVE=8
# 模式、动作或控制源切换过渡中。
uint8 STATE_TRANSITIONING=9
# 故障状态，等待确认或恢复流程。
uint8 STATE_FAULT=10
# 阻尼状态，允许发布受控阻尼命令，但不进入模型推理。
uint8 STATE_DAMP=11
# 关闭或下电流程。
uint8 STATE_SHUTDOWN=12

# 标准消息头；stamp 为状态生成时间。
std_msgs/Header header

# 当前系统状态，取值为 STATE_* 常量。
uint8 state
# 当前状态名称，便于日志和外部 API 展示。
string state_name
# 当前活动模式，例如 idle、rl、rule。
string active_mode
# 当前获得控制权的来源，例如 teleop、api、rl_manager。
string active_control_source
# 当前活动 RL 模型名称；非 RL 模式可为空。
string active_rl_model

# 当前是否处于故障状态。
bool fault_active
# 当前主要故障原因；无故障时可为空。
string fault_reason
