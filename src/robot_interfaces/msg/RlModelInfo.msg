# 模型注册名称，例如 t4_walk。
string name
# 模型展示别名，例如 行走。
string alias
# 模型业务分类，例如 走跑类、单次动作类、持续动作类。
string category
# 当前平台实际使用的推理后端，例如 onnx、pt、rknn、pipeline。
string backend
# 配置声明的模型时长，单位秒；仅供查询展示，不参与控制逻辑。
float32 duration_sec
# 是否配置了固定时长；仅供查询展示，不参与控制逻辑。
bool has_duration
