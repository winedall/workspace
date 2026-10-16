# 系统提示文本：name 填 config/data/tips.yaml 中的提示文本 key。
uint8 KIND_TIP=1

# 动作音频文件：name 填 config/data/actions 下的文件名，或绝对路径。
uint8 KIND_FILE=2

# 临时文本 TTS：name 填要直接播报的文本内容。
uint8 KIND_TEXT=3

# 请求类型，只支持 KIND_TIP/KIND_FILE/KIND_TEXT；其他值会被 robot_audio_node 忽略。
uint8 kind

# 请求内容。robot_audio_node 会先去掉首尾空白，空字符串直接忽略。
string name

# 播放优先级，数值越大越先播放；priority >= 200 会触发中断逻辑。
uint8 priority

# 是否允许中断当前播放；为 true 时会触发中断逻辑。
bool interrupt
