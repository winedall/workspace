# 单个逻辑关节反馈状态。
# robot_driver 应把硬件反馈转换为该统一格式后发布，避免上层依赖厂家协议字段。

# 关节正常。
uint8 JOINT_ERR_NONE=0
# 电机或驱动器底层 motor_error 告警；具体原因见 motor_error 原始码。
uint8 JOINT_ERR_MOTOR_WARN=1
# 反馈角度/位置状态非法，例如越界或跳变。
uint8 JOINT_ERR_INVALID_ANGLE_STATE=2
# 目标角度/位置命令非法，上游命令应被拒绝。
uint8 JOINT_ERR_INVALID_ANGLE_CMD=3
# 零位缺失或零点状态异常。
uint8 JOINT_ERR_ZERO_ABSENCE=4


# 关节唯一名称，必须与 robot_joints.yaml 中的 name 保持一致。
string joint_name

# 当前位置：旋转关节单位 rad，直线关节单位 m。
float32 position
# 当前速度：旋转关节单位 rad/s，直线关节单位 m/s。
float32 velocity
# 当前力矩/推力：旋转关节单位 Nm，直线关节单位 N。
float32 effort
# 电机温度，单位摄氏度。温度不可用时为 0。
float32 motor_temp
# 电机驱动 MOS 温度，单位摄氏度。不可用时为 0。
float32 mos_temp
# 电机驱动底层故障码，由 Encos/EtherCAT 直接读取，未经映射。
uint8 motor_error

# 当前关节是否处于使能状态。
bool enabled
# 当前关节错误码，取值为 JOINT_ERR_* 常量。
uint8 joint_error
# 人类可读错误说明；无错误时可为空字符串。
string error_message
