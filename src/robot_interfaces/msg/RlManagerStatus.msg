# RL 管理节点状态。
# robot_rl_manager 周期发布该消息，供 robot_state 判断 RL 模型是否已准备好。

# 标准消息头；stamp 为状态生成时间。
std_msgs/Header header

# RL 管理节点是否在线。
bool online
# 当前 active model 是否已经成功加载到后端。
bool model_loaded
# 当前 policy backend 是否 ready，可用于推理。
bool model_ready
# 当前是否正在执行推理周期。
bool inference_active
# 最近一次推理或命令构造是否出现错误。
bool inference_error

# 当前活动模型名称，例如 stand、walk。
string active_model
# 当前请求切换到的模型名称；无切换请求时可为空。
string requested_model
# 当前活动模型类型，例如 onnx、torchscript、mock。
string model_type
# 当前 policy backend 模块路径。
string policy_backend
# 当前模型踝关节是串联空间（true）还是并联空间（false）。robot_control 据此做状态路径 P2S 转换。
bool ankle_in_series

# 模型推理目标频率，单位 Hz。
float32 inference_frequency_hz
# RL 候选关节命令发布频率，单位 Hz。
float32 command_publish_frequency_hz
# RL 状态发布频率，单位 Hz。
float32 status_publish_frequency_hz
# 最近一次推理耗时，单位 ms。
float32 last_inference_latency_ms

# 当前 IMU 输入是否可用于推理。
bool imu_ready
# 最近一次 IMU 输入年龄，单位 s；未知或未启用时为 -1。
float32 imu_age_sec
# 当前 policy adapter 缺失的必需输入名称，例如 imu。
string[] missing_inputs

# 最近主要错误信息；无错误时为空。
string error_message
