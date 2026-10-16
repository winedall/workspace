# 多关节命令帧。
# RL、规则控制器、调试工具等模块应使用该消息表达一帧关节级命令。

# 标准消息头；stamp 用于命令新鲜度检查，frame_id 可按控制链路约定填写。
std_msgs/Header header

# 全链路关联 ID。由命令源创建并由 router/control/driver 透传，用于离线串联同一条控制链路。
uint64 trace_id
# 当前发布者在当前 topic 上的输出序号，用于检测该输出流的乱序、重复或丢帧。
uint32 sequence
# 当前帧的直接上游 JointCommands.sequence。无上游时为 0。
uint32 parent_sequence
# 命令来源，例如 rl_manager、command_router、robot_control、rule_controller。
string source
# 当前帧的直接上游 source。无上游时为空字符串。
string parent_source
# 当前链路阶段，例如 desired、accepted、control_output、prepare_interp、timeout。
string stage

# 命令帧类型：NONE=0（常规执行命令），DAMPING=1（阻尼命令），PREPARE=2（准备插值命令），RUN=3（模型推理输出）。
uint8 TYPE_NONE=0
uint8 TYPE_ZERO_MOMENT=1
uint8 TYPE_DAMPING=2
uint8 TYPE_PREPARE=3
uint8 TYPE_RUN=4

# 命令帧类型，下游（robot_control）据此决定处理策略：PREPARE 触发多周期插值，RUN/DAMPING 正常平滑输出。
uint8 type

# 当前模型踝关节是串联空间（true）还是并联空间（false）。
# robot_control 据此联合 ankle_env_parallel 决定是否需要做串并联空间转换。
bool ankle_in_series

# 本帧包含的关节命令列表。同一帧中不应出现重复 joint_name。
JointCommand[] commands
