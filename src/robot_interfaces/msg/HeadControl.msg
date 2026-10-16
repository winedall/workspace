# 头部外部控制指令
# yaw 和 pitch 使用角度值（度），方便操作人员使用。
# 节点内部会自动转换为弧度下发。

# 目标角度（度，-180 ~ 180）
float32 yaw
float32 pitch

# true=相对角度（当前位置 + delta），false=绝对角度
bool is_relative
