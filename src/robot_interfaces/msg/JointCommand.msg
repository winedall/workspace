# 单个逻辑关节命令。
# 该消息描述上层控制意图，不包含 motor_id、slave_id、通道号等硬件映射字段。

# 关节唯一名称，必须与 robot_joints.yaml 中的 name 保持一致。
string joint_name

# 目标位置：旋转关节单位 rad，直线关节单位 m。
float32 position
# 目标速度：旋转关节单位 rad/s，直线关节单位 m/s。
float32 velocity
# 目标力矩/推力：旋转关节单位 Nm，直线关节单位 N。
float32 effort

# 位置环比例增益。允许为 0，不能为负。
float32 kp
# 速度/阻尼增益。允许为 0，不能为负。
float32 kd

# 是否请求该关节使能。最终是否允许使能仍需经过 command_router/control/driver 安全判断。
bool enable
