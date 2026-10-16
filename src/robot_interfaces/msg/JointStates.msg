# 多关节反馈状态帧。
# robot_driver 周期发布该消息，供 control、RL、state_manager 等模块使用。

# 标准消息头；stamp 为反馈生成时间，frame_id 可按机器人基准坐标系填写。
std_msgs/Header header

# 状态帧序号，用于检测状态丢帧、乱序或重复。
uint32 sequence
# 最近一次被 driver 接收/应用的控制命令 trace_id，用于命令-反馈关联。
uint64 last_command_trace_id
# 最近一次被 driver 接收/应用的 control/hardware_commands.sequence。
uint32 last_command_sequence

# 本帧包含的关节状态列表，建议按 robot_joints.yaml 中的关节顺序排列。
JointState[] states
