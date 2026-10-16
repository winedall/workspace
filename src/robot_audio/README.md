# robot_audio

离线中英文文本转语音（TTS）播报模块。基于 Piper ONNX 模型 + eSpeak 音素引擎，无需网络连接即可将任意文本合成语音并通过 ALSA 播放。

**v1.2.11 起**：系统播报（状态切换、配置菜单、告警）迁移到**预录制提示音**体系——`AudioClient`（`audio_client.py`）+ `audio_tips` 常量直接播放 WAV 文件，TTS 仅保留用于任意文本合成场景。

## 架构

```
RobotAudioNode (ROS 2 节点)
├── PriorityQueue        ← 线程安全优先队列（heapq）
├── _play_worker 线程    ← 后台消费队列，串行合成+播放
├── AudioPlay 处理        ← 预录制 WAV 直接播放（提示音通道）
└── TTSEngine            ← Piper ONNX 语音合成引擎
    ├── zh_CN-huayan-medium.onnx (61 MB, 中文女声)
    └── en_US-lessac-medium.onnx (61 MB, 英文女声)
```

## 提示音库（`robot_audio.audio_tips`）

- **清单**：`config/data/tips.yaml`（~50 个提示音键，每个键对应 `config/data/*.wav` 文件）
- **代码生成**：`scripts/audio/tips.py` 从 tips.yaml 生成 `audio_tips.py`（Python）和 `robot_common/audio_tips.hpp`（C++）常量
- **客户端**：`audio_client.py`（Python）/ `robot_common::AudioClient`（C++），封装 `AudioPlay` 消息发布到 `audio/play`，支持优先级和打断
- **使用方**：`robot_state`（状态切换/电机告警）、`robot_rl_manager`（配置菜单/模型切换）

## 快速启动

```bash
# 系统依赖
sudo apt install espeak-ng espeak-ng-data alsa-utils

# Python 依赖
pip install piper-tts numpy

# 通过 bringup（默认启用）
ros2 launch robot_bringup zl_robot.launch.py

# 独立运行
ros2 run robot_audio robot_audio_node
```

## ROS 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `tts_rate` | int | `80` | 语速（影响 `length_scale = 100.0/rate`，越高越快） |
| `tts_volume` | float | `1.0` | 音量（当前未应用于合成，存储预留） |
| `tts_voice` | string | `"zh"` | 偏好语言（仅日志用途，实际按文本自动检测） |
| `queue_size` | int | `10` | 最大队列深度（0 = 无限） |
| `interrupt_enabled` | bool | `true` | 高优先级消息（≥200）打断当前播放 |
| `subscribe_topic` | string | `audio/voice_text` | TTS 文本订阅话题 |
| `audio_play_topic` | string | `audio/play` | 预录制 WAV 播放订阅话题（`AudioPlay` 消息） |
| `service_name` | string | `audio/speak` | 服务名称 |
| `status_topic` | string | `audio/status` | 状态发布话题 |

## 话题与服务

| 类型 | 名称 | 类型 | 说明 |
|------|------|------|------|
| **Sub** | `audio/voice_text` | `VoiceText` | 异步 TTS 文本输入（含优先级） |
| **Sub** | `audio/play` | `AudioPlay` | 预录制 WAV 音频播放（含优先级和中断开关） |
| **Pub** | `audio/status` | `SpeakStatus` | 播放状态（2 Hz） |
| **Srv** | `audio/speak` | `Speak` | 同步 TTS 请求 |

## API 使用

### 异步播报（话题）

```python
from robot_interfaces.msg import VoiceText

msg = VoiceText()
msg.text = "你好，我是机器人"
msg.priority = 50
publisher.publish(msg)
```

### 同步播报（服务）

```bash
ros2 service call /robot/audio/speak robot_interfaces/srv/Speak "{text: '你好', priority: 50}"
```

### 打断机制

优先级 ≥ 200 的消息会立即终止当前播放并插队：

```python
msg = VoiceText()
msg.text = "紧急停止"
msg.priority = 200  # 打断当前播放
publisher.publish(msg)
```

仅当 `interrupt_enabled:=true` 时生效。

## 语言自动检测

引擎按 Unicode 汉字比例检测语言：

- 汉字（U+4E00~U+9FFF）占比 > 30% → 中文模型（`zh_CN-huayan-medium`，eSpeak `cmn`）
- 其他 → 英文模型（`en_US-lessac-medium`，eSpeak `en-us`）

每次 `speak()` 调用使用单一模型，不支持单句内混合语言。

## 合成管线

```
原始文本
  → 语言检测（汉字比例）
  → PiperVoice.synthesize()
      → eSpeak 音素化（cmn / en-us）
      → 音素 ID 映射
      → ONNX 推理 → float32 音频块
  → np.concatenate → int16 数组
  → wave 模块写入临时 WAV（22050 Hz, 1ch, 16-bit PCM）
  → aplay -q <temp.wav>
  → 删除临时文件
```

## 模型文件解析顺序

引擎按以下顺序搜索 `.onnx` 模型：

1. `<ament_share>/robot_audio/models/`（colcon 安装路径）
2. `<source>/src/robot_audio/models/`（源码树）
3. `~/.local/share/piper/voices/`（手动安装）

可通过将模型文件放置在上述路径来添加新语音。

## 模型配置

| 属性 | 中文 | 英文 |
|------|------|------|
| 模型 | `zh_CN-huayan-medium` | `en_US-lessac-medium` |
| 数据集 | huayan | lessac |
| 采样率 | 22050 Hz | 22050 Hz |
| eSpeak 语音 | `cmn` (Mandarin) | `en-us` |
| 音素数 | 152 | 154 |
| Piper 版本 | 1.0.0 | 1.0.0 |

## 依赖

| 依赖 | 说明 |
|------|------|
| `piper-tts` | Piper 语音合成（ONNX Runtime + eSpeak 集成） |
| `numpy` | 音频数组拼接和数据类型转换 |
| `espeak-ng` | 系统级音素转换引擎 |
| `alsa-utils` | 提供 `aplay` 命令行播放器 |
| `rclpy` | ROS 2 Python 客户端库 |
| `robot_interfaces` | `VoiceText`、`SpeakStatus`、`Speak` 接口定义 |

## 已知限制

- **音量参数未应用**：`tts_volume` 存储但未在合成中生效，音量由系统 ALSA 控制
- **无停顿/取消接口**：`SpeakStatus` 定义了 `STATE_PAUSED` 但当前无暂停实现
- **阻塞式播放**：`speak()` 同步等待 `aplay` 完成，长文本会长时间占用工作线程
- **无队列溢出保护**：队列满时 `put()` 静默返回 `False`，不记录告警
