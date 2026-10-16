# 云卓 G12 遥控器 + GR01 接收机 SBUS 接入说明

> 面向**开发者/调试**（SBUS、通道、代码底层）。
> 只管怎么遥控器操作、面向**操作员**的，请看《G12遥控器操作手册.md》。

- 工作区：`/home/zoomlion/work/01_code/zl_robot_01`
- 链路：`G12 遥控器 → GR01 接收机(SBUS) → USB 转串口(CH340 /dev/g12_sbus) → teleop_sbus_node → /robot/input/intent`
- 说明：`teleop_sbus_node` 直接读 SBUS 串口、按 G12 通道语义分类（摇杆/按键/滚轮），一步生成 `RobotIntent`，不再走 `sbus_joy_bridge`/`/joy` 旧链路。
- 串口名固定：优先用 udev 生成的固定链接 `/dev/g12_sbus`（与 ttyUSB 编号无关），找不到时节点自动按 SBUS 帧特征探测，无需手动改配置，详见[二、硬件接入与串口固定](#二硬件接入与串口固定)。

---

## 一、快速上手（怎么开始用）

**第 1 步：接线。** G12 遥控器 → GR01 接收机（对频成功）→ 接收机 SBUS 输出接 USB 转串口（CH340）→ 电脑 USB 口。

**第 2 步：电脑一次性准备（只做一次）。**

```bash
# 当前用户加入串口组（dialout），否则打不开 /dev/ttyUSB*
sudo usermod -aG dialout $USER        # 加完**重新登录**生效
# 安装 udev 规则：把遥控器固定映射到 /dev/g12_sbus（与 ttyUSB 编号无关）
sudo cp <仓库路径>/scripts/udev/99-robot-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=tty
```

> 换任何一台电脑/随便插哪个 USB 口，串口都固定叫 `/dev/g12_sbus`。
> 没装规则也能用：节点会自动探测（见[二](#二硬件接入与串口固定)）。

**第 3 步：插上设备并验证。**

```bash
ls -l /dev/g12_sbus      # 应能看到 -> /dev/ttyUSB*（装过规则后）
```

**第 4 步：开遥控器 + 开总开关。** G12 开机后把 **E 开关拨到高档、F 开关拨到低档**（总开关开启），遥控器才可控制机器人。

**第 5 步：启动系统。**

```bash
cd /home/zoomlion/work/01_code/zl_robot_01
source install/setup.bash
ros2 launch robot_bringup zl_robot.launch.py enable_mujoco_sim:=true robot_type:=humanoid
```

**第 6 步：确认数据进来了。** 看 `teleop_sbus_node` 日志应有 `opened /dev/g12_sbus ...`、没有 `no SBUS data` 警告；拨摇杆/滚轮，另开终端 `ros2 topic echo /robot/input/intent` 能看到 `motion`/`config` 意图。详细校验见[六、如何验证已接好](#六如何验证已接好)。

---

## 二、硬件接入与串口固定

### 硬件清单

| 部件 | 说明 |
|------|------|
| G12 遥控器 | 主控遥控设备，需与接收机对频 |
| GR01 接收机 | 输出 SBUS（单线，反相电平，串口 100000bps 8E1） |
| USB 转串口线（CH340） | 厂商/产品号 `1a86:7523`，内核自带驱动，免安装 |
| 电脑 | Ubuntu 22.04 + ROS 2 Humble |

### 串口名为什么要固定

USB 转串口在 Linux 下叫 `/dev/ttyUSB0/1/2...`，**编号按插入顺序分配，每次插拔/换口都可能变**。因此节点 `teleop_sbus_node` 不再假设固定的 ttyUSB 编号，按以下优先级解析串口：

1. **固定链接 `/dev/g12_sbus`（推荐）**：由 udev 规则（`scripts/udev/99-robot-usb.rules`）生成，只跟转接线芯片识别号挂钩，跟 ttyUSB 编号、插哪个口无关。
2. **`/dev/serial/by-id/...`（无需 sudo）**：系统自带静态链接，也跟芯片走；名字形如 `usb-1a86_USB_Serial-if00-port0`。
3. **`auto`/自动探测（兜底）**：扫描 `by-id`/`ttyUSB*`/`ttyACM*`，以「能否解析出标准 SBUS 帧（0x0F 头 / 0x00 尾 / 25 字节）」判定哪个是遥控器，认到即用。

优先级 1/2 失效（没装规则、换转接线、链接不存在）时会自动落到 3，**无需手动改 `teleop_params.yaml`**。

### 接线注意

- GR01 SBUS 信号接 USB 转串口的 **RX**，GND 共地；SBUS 是反相 TTL，若转接线带电平转换更稳妥。
- 波特率固定 `100000`，`8 数据位 / 偶校验 / 1 停止位`（`teleop_params.yaml` 里 `baudrate: 100000`）。
- 接好后首次解析依赖遥控器**开机发数据**：若开机晚于节点，节点会每 0.5s 自动重试，遥控器一发数据就能接入，无需重启。

### 其他电脑接入（多机部署）

换一台电脑接同样的转接线，只需做这几件 **一次性** 的事：

```bash
# 1) 加串口组权限，然后重新登录
sudo usermod -aG dialout $USER
# 2) 装 udev 规则，使遥控器固定为 /dev/g12_sbus
sudo cp <仓库路径>/scripts/udev/99-robot-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=tty
# 3) 验证
ls -l /dev/g12_sbus
```

然后部署好 ROS 2 workspace（`robot_teleop`、`robot_interfaces`），`source install/setup.bash` 即可使用。`teleop_params.yaml` 里已有 `/dev/g12_sbus`，不用改。

---

## 三、什么是 CH（通道）

SBUS 每个样例帧固定携带 **16 个通道**（CH1~CH16）。每个通道是一个 **11-bit** 数值，**默认范围 0 ~ 1024/1023 = 中心，2047 = 满量程**。

- **中心值**：约 **1002 / 1023.5**（摇杆回中、按键不按时）
- **正向满量程**：**1722** 左右
- **反向满量程**：**282** 左右
- 三段开关低档也在 **282** 附近（原因：开关本身是脉冲序列，实际档位值见下表）

11-bit 归一化公式：`(raw - 1023.5) / 1023.5`，范围约 `-1.0 ~ +1.0`，随后按 `axis_invert` 反转方向。

### 通道值范围速查

| 含义 | 值 |
|------|----|
| 反向满量程 | ~282 |
| 中心 | ~1002 (1023.5) |
| 正向满量程 | ~1722 |
| 理论满量程 | 0 ~ 2047 |
| 三段开关档位阈值 | 中心下方 650 / 中心上方 1350 |
| 按键按下阈值 | 1350 |

---

## 四、各 CH 对应的按键 / 功能

以下为 G12 实测标定的通道映射（`teleop_sbus.py` / `teleop_params.yaml` 定义）。

| 通道 | 物理控件 | 类型 | 功能 |
|------|----------|------|------|
| **CH1** | 右摇杆 X（左右） | 模拟 | **转身 angular_z**（invert=true） |
| **CH2** | 右摇杆 Y（前后） | 模拟 | 预留（未启用） |
| **CH3** | 左摇杆 X（左右） | 模拟 | **横向平移 linear_x**（invert=false） |
| **CH4** | 左摇杆 Y（前后） | 模拟 | **前进/后退 linear_y**（invert=true） |
| **CH5** | 三段开关 **E** | 三档 | **总开关**：高档时生效（配合 F 低档） |
| **CH6** | 三段开关 **F** | 三档 | **总开关**：低档时生效（配合 E 高档） |
| **CH7** | 键 **A** | 瞬时 | 连按 3 次 = ready；单独长按 = finish_loop；A+B 长按 = 运动停止/恢复 |
| **CH8** | 键 **B** | 瞬时 | 连按 3 次 = walk；A+B 长按 = 运动停止/恢复 |
| **CH9** | 键 **C** | 瞬时 | 连按 3 次 = reinit；action 菜单内长按 = 触发动作 |
| **CH10** | 键 **D** | 瞬时 | 连按 3 次 = damp（阻尼，最高优先） |
| **CH11** | 滚轮 **G** | 弹簧回中 | 一级菜单滚动（右=下/左=上/循环） |
| **CH12** | 滚轮 **H** | 弹簧回中 | 二级菜单滚动（右=下/左=上/循环） |
| CH13~CH16 | — | — | 未使用 |

> A/B/C/D 为瞬时非自锁键：按下~1722、松开~282，判断阈值 `button_press_threshold=1350.0`。
> E/F 为三段开关：低~282 / 中~1002 / 高~1722，总开关按 E 高档(≥1350)、F 低档(≤650) 判定。

---

## 五、按键逻辑

### 总开关（master switch）

**仅当 E(CH5) 高档 且 F(CH6) 低档** 时遥控器才可控制；其它 E/F 组合下遥控器**完全无控制**
（菜单、动作、状态切换、运动全部失效，唯一例外是失联停车 timer）。

### 状态切换 —— 连按 3 次（按下沿计数）

| 操作 | 意图 | 说明 |
|------|------|------|
| 连按 3 次 A | `status/ready` | 准备（起身站立） |
| 连按 3 次 B | `status/walk` | 行走（授权移动） |
| 连按 3 次 C | `status/reinit` | 待机（仅在阻尼模式有意义） |
| 连按 3 次 D | `status/damp` | 阻尼（最高优先，立即停车） |

> 连按 = 快速点按 3 下（每次点完松开），3 次需在 `triple_timeout_sec`(1.5s) 内完成。
> 实现上用「按下沿（0→1）」计数，长按不会误判为连按。

### 长按 —— 按住满 `long_press_timeout_sec`(3s) 触发

| 操作 | 意图 | 说明 |
|------|------|------|
| A+B 同时长按 | `model/motion_stop`（stop → resume → stop 循环） | 运动停止/恢复 |
| A 单独长按 | `model/finish_loop` | 循环结束（优雅退出循环动作） |

### 配置菜单 —— G/H 滚轮（受总开关限制）

- **G 滚轮(CH11)** 拨动 = 一级菜单步进（`config_primaries`：getup → speed → nav → mode → action 循环）。
- **H 滚轮(CH12)** 拨动 = 当前一级的二级选项步进。
- 切二级选项发 `config/<一级>/<选项>` 意图，由 `robot_rl_manager` 播报语音。
- **action 菜单**：二级为动作清单，滚到目标动作后**长按 C(CH9) 3 秒**触发 `model/switch_model/<动作>`。

> 滚轮为弹簧回中，以「偏转 -> 回中」那一路沿计一次拨动，方向取偏转方向
> （偏移判定 `gh_center=1002` / `gh_active_threshold=120.0`）。

### 运动轴

- CH4 左摇杆前后 → linear_y（前进/后退）
- CH3 左摇杆左右 → linear_x（横移）
- CH1 右摇杆左右 → angular_z（转身）
- CH2 右摇杆前后 → 未启用
- 仅总开关开启且已由「连按 3 次 B」进入 walk 授权后输出运动。

安全机制：
- 信号丢失/串口断连超时后发布全零停止意图。
- walk 授权后要求摇杆回中一次才允许输出运动，避免异常初值。
- 订阅 `/robot/state`，仅在本机状态为 RL_ACTIVE 时启用失联停车。

---

## 六、如何验证已接好

### 1) 串口名存在且指向设备

```bash
ls -l /dev/g12_sbus        # 装机规则后：应指向 /dev/ttyUSB*
ls -l /dev/serial/by-id/   # 无规则时：能看到 usb-1a86_USB_Serial-*
```

### 2) 节点在收数据

启动后看 `teleop_sbus_node` 日志：

- `opened /dev/g12_sbus @ 100000 bps 8E1` → 串口打开成功（自动探测成功还会先打 `auto-detected SBUS receiver on ...`）。
- 反复出现 `no SBUS data on ...` → 没收到数据：检查遥控器是否开机、E/F 总开关、接线（仅当节点启动时遥控器还没发数据，会自动重试，一般不用处理）。

### 3) 意图在发布

```bash
ros2 topic echo /robot/input/intent
```

E 高档 + F 低档后拨动 G/H 滚轮或摇杆，应能实时看到 `config`/`motion` 意图。

### 4) 调试：直接看原始 16 通道值（可选）

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 - <<'EOF'
import serial
from robot_teleop.teleop_sbus import sbus_frame_to_channels, _SbusDecoder
p = serial.Serial("/dev/g12_sbus", 100000, bytesize=8, parity='E', stopbits=1, timeout=0.1)
dec = _SbusDecoder()
while True:
    for f in dec.feed(p.read(64)):
        print(sbus_frame_to_channels(f))
EOF
```

---

## 七、关键文件

| 文件 | 作用 |
|------|------|
| `src/robot_teleop/robot_teleop/teleop_sbus.py` | 读 SBUS → 通道分类（摇杆/按键/滚轮/总开关/配置菜单）→ 生成 RobotIntent；含串口固定链接解析与自动探测 |
| `src/robot_bringup/config/teleop_params.yaml` | `teleop_sbus_node` 参数（`serial_port=/dev/g12_sbus` / axis_map / invert / auth_channel / gh 滚轮 / E/F 总开关 / config_primaries / config_menu_options / action_options） |
| `scripts/udev/99-robot-usb.rules` | udev 规则：IMU → `/dev/robot_imu`、电池 → `/dev/robot_battery`、G12 CH340 → `/dev/g12_sbus` |
| `src/robot_teleop/setup.py` | 三个 entry_points：`teleop_sbus_node` / `robot_teleop_node` / `robot_keyboard_node` |
| `src/robot_bringup/launch/zl_robot.launch.py` | `enable_sbus` 参数（默认 true）与 SBUS 节点启动逻辑 |

---

## 八、启动与配置

`enable_sbus`（默认 true）自动启动 `teleop_sbus_node`。如需禁用可加 `enable_sbus:=false`。

```bash
cd /home/zoomlion/work/01_code/zl_robot_01
source install/setup.bash
ros2 launch robot_bringup zl_robot.launch.py enable_mujoco_sim:=true robot_type:=humanoid
```

### 串口配置（teleop_params.yaml）

```yaml
serial_port: /dev/g12_sbus   # 固定链接；不存在时节点自动探测，也填 /dev/serial/by-id/... 或 auto
baudrate: 100000             # 标准 SBUS 波特率；转换器有差异需现场核对
```

### 其余关键配置位置

- `e_channel=5` / `f_channel=6`：总开关（E 高档 + F 低档生效）。
- `gh_channel_g=11` / `gh_channel_h=12`：G/H 滚轮；`gh_center` / `gh_active_threshold` 判定偏转。
- `config_primaries`：一级菜单列表（yaml 数组）。
- `config_menu_options`：yaml 块字符串 `{一级: [...选项]}`；`mode` 已含 `app`（应用模式）。
- `action_options`：动作配置菜单动作清单（动作类 + 舞蹈类 + 特技类）。

---

## 九、注意事项

- **A/B/C/D 需为非自锁（瞬时）**，否则长按/连按语义会失效。
- 若某轴方向反了，改 `teleop_params.yaml` 中 `axis_invert` 对应项，重启节点。
- E/F 三段开关档位按阈值判定（高档 `>=1350`、低档 `<=650`）；若信号基准漂移，用第六节第 4 条的调试脚本复测再调整。
- 授权后第一帧会强制清零轴（walk 授权要求先回中），无需刻意先松手。
- 自动探测需要遥控器开机发数据才能「认」到；若节点先启动、遥控器后开，会自动重试接入，不必重启。