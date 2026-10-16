# 统一高层控制意图。
# teleop、HTTP/API、SDK、MQTT、云端或工具入口都应先转换为该消息，
# 不应绕过 command_router 直接下发关节命令。

# 标准消息头；stamp 用于判断意图是否过期。
std_msgs/Header header

# 意图来源，例如 teleop、http、mqtt、sdk、cloud、test。
string source
# 意图类型：status、motion、mode、model、config 或 reset。
string intent_type
# 具体命令，例如 ready、walk、switch_mode、switch_model、speed。
string command
# 命令对象或参数。mode 使用 rl/rule；model 使用模型名；config 使用配置值。
string action

# 期望前进速度，单位 m/s。
float32 linear_x_mps
# 期望侧向速度，单位 m/s。
float32 linear_y_mps
# 期望偏航角速度，单位 rad/s。
float32 angular_z_radps

# 意图优先级。
uint8 priority
