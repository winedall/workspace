# mj_sim URDF 文件规范

适用于 `mj_sim/description/*/urdf/` 下所有 URDF 模型描述文件（Z01_edm、Z01_edm_add_head、D015_poc 等）。
本文档定义连杆/关节命名、元素与属性值、碰撞体、文件顺序与排版格式的约定，新增或修改
URDF 时必须遵守。

> 注意：`description/*/xml/*.xml` 是仿真运行时实际加载的 MJCF 文件，URDF 为参考模型。
> 修改 URDF 后若涉及机器人结构变化，需同步确认对应 MJCF 的一致性。

---

## 1. 机器人命名

命名形式：`{产品型号}_{阶段}_{特殊硬件改动}`（特殊硬件改动可省略）。

| 元素 | 说明 |
|------|------|
| 产品型号 | 直接使用机器人产品名称：D2→**D015**、D4→**D040**、T4→**Z01** |
| 阶段 | 无后缀 / `std` = 量产版本；`poc` = 原理样机；`edm` = 工程样机 |
| 特殊硬件改动 | 仅指出该版本与标版的最大区别（如 `add_head`、`parallel_ankle`） |

- 文件夹名、URDF 文件名、URDF 内 `<robot name>` 三者保持一致，如
  `Z01_edm_add_head/urdf/Z01_edm_add_head.urdf` + `<robot name="Z01_edm_add_head">`。
- 当前模型：`D015_poc`、`D040_poc`（原理样机）；`Z01_edm`、`Z01_edm_add_head`、
  `Z01_edm_parallel_ankle`（工程样机）。

## 2. 文件结构

```
<?xml version="1.0" encoding="utf-8"?>
<robot name="<模型名>">
  <material>…</material>          # 可选，全局材质定义（引用式，放最前）
  <mujoco>…</mujoco>              # 可选，MuJoCo 编译器扩展（如 <compiler meshdir=…/>）
  连杆 → 关节 → 下一连杆 …        # 按 §5 顺序展开
</robot>
```

- 每个元素前空一行分隔，文件以单个换行符结尾。
- 模型名（`<robot name>`）与目录名一致，如 `T4_std`、`D2`。

## 3. 命名规范

命名基本形式（下划线分隔、全称英文小写，见 §3.7 禁止简写）：

- **关节**：`{侧别}_{部位}_{转轴}`；固定关节无转轴，以 `_fixed` 后缀标识
- **连杆**：`{关节名}_link`（连杆名 = 其入链关节名 + `_link`）

### 2.1 基座

- 基座统一命名为 `torso_link`，**不允许别名**（不得使用 `base_link`、`trunk_link` 等）。
- URDF 中**不建模浮动基关节**（如 `floating_base_joint`）；漂浮自由度由仿真端
  （MJCF freejoint）处理。

### 2.2 侧别（第一元素）

| 侧别 | 名称 |
|------|------|
| 左 / 右 | `left` / `right` |
| 左前 / 左后 | `left_front` / `left_rear` |
| 右前 / 右后 | `right_front` / `right_rear` |
| 上左 / 下左（上下前后不常用） | `upper_left` / `lower_left` |

- 侧别限定词仅允许 **上/下、左/右、前/后**（`upper`/`lower`、`left`/`right`、
  `front`/`rear`），不得出现其它侧别。
- 单侧同型分支超过 3 个（如六足机器人）时改用数字编号：`left_1`、`left_2`、`left_3`。
  同型 = 多条支干构型基本相同（人形的手臂与腿不视为同型）。

### 2.3 部位（第二元素）

- 常用：`shoulder` 肩、`elbow` 肘、`wrist` 腕、`hip` 髋、`knee` 膝、`ankle` 踝。
- 不常用：`toe` 趾、`finger` 指、`hoof` 蹄。
- 无侧别的躯干部位：`waist` 腰、`head` 头；标准解剖部位（如 `foot` 脚）同样允许。
- 附肢不具备生物学意义上的关节部位名时，直接以编号命名：`left_1`、
  `upper_left_1`、`left_1_1`。

### 2.4 转轴（第三元素，仅关节）

- 第三元素指示关节转轴（rpy）：绕 x = `roll`、绕 y = `pitch`、绕 z = `yaw`。
- 转轴与 xyz 不完全平行时，按最近的轴**近似**（如 `(0, 0.961, 0.276)` → `pitch`）。
- 转轴与任一 xyz 轴都不接近时，以数字编号替代（如 `left_hip_1`）。
- 一个部位仅含单个关节时，转轴**可省略**（如 `left_knee`、`left_elbow`）。
- 固定关节无转轴，不写第三元素，以 `_fixed` 后缀标识（如 `left_front_foot_fixed`）。

### 2.5 连杆

- 连杆名 = 其入链关节名 + `_link`：`left_front_hip_pitch` → `left_front_hip_pitch_link`。
- 仅含碰撞体的哑连杆：`{关节名}_collision_{序号}_link`，如 `left_front_knee_collision_1_link`。
- 占位/空连杆遵循同一规则（如 `left_front_hip_fixed_link`，允许无子元素）。

### 2.6 传感器/载荷

- 不遵循侧别/部位规则，直接用器件全名：`imu_link`、`radar_link`、`camera_link`；
  其固定关节与连杆同名（`imu`、`radar`、`camera`）。

### 2.7 禁止简写

- 禁止任何形式的简写——`r` 既可能是 right 也可能是 roll，`f` 既可能是 front 也可能
  是 foot，一律写全称。

### 2.8 材质命名（`<material name>`）

- 引用式材质（`<link>` 外定义，`<visual>` 内以 `name` 引用）：小写颜色名，如 `black`、`silver`、`red`、`white`。
- 内联材质（SolidWorks 导出风格）：`name=""`，并携带 `<color>` 子元素。两种风格只取其一，同一文件内保持一致。

## 4. 元素与属性值规范

### 3.1 元素顺序

- **joint 子元素顺序**：`origin` → `parent` → `child` → `axis` → `dynamics` → `limit` →（`mimic`、`safety_controller`）
- **link 子元素顺序**：`inertial` → `visual`(可多个) → `collision`(可多个)
- **inertial 子元素顺序**：`origin` → `mass` → `inertia`
- **visual / collision 子元素顺序**：`origin` → `geometry` → `material`（collision 的 material 可省略）

### 3.2 属性顺序

| 元素 | 属性顺序 |
|------|----------|
| `origin` | `xyz` 在前，`rpy` 在后 |
| `joint` | `name`, `type`, (`dont_collapse`) |
| `limit` | `lower`, `upper`, `effort`, `velocity` |
| `dynamics` | `damping`, `friction` |
| `inertia` | `ixx`, `ixy`, `ixz`, `iyy`, `iyz`, `izz` |
| `mesh` | `filename`, `scale` |
| `cylinder` | `radius`, `length` |

### 3.3 数值规范

- **单位**：国际单位制 —— 长度 m、质量 kg、角度 rad（`rpy`）、力矩 N·m（`effort`）、
  角速度 rad/s（`velocity`）、惯量 kg·m²。
- **格式**：普通小数，不写 `-0`、不留无意义尾随零（`-0.0` → `0`，`0.1803340` → `0.180334`，
  `1.0` → `1`）。科学计数法仅用于极端量级（如 `3.67328e-22`）；由 CAD 导出器产生的
  `E-3`/`E-6` 记法应改为普通小数（`180.3340E-3` → `0.180334`）。
- **`xyz` / `rpy`**：三个分量间用单个空格分隔。
- **`axis`**：单位向量（如 `0 1 0`、`0 0.961261695 0.275637356`）。
- **`limit`**：`lower < upper`；固定关节不写 `limit`（`fixed` 关节的 `axis`/`limit` 一并省略）。
  既有文件中的遗留例外（如 `t4_std_add_head` 的 `waist_pitch`/`waist_roll` 固定关节仍带
  `axis`/`limit`，`d2_align` 的固定关节带 `axis`）保持原状，新模型不再引入。
- **`inertia`**：对称正定矩阵；`mass value > 0`。
- **`rgba`**：各分量 ∈ [0, 1]。
- **网格 `scale`**：CAD 网格以 mm 导出时统一写 `0.001 0.001 0.001`（mm→m）。
- **网格路径**：优先相对路径（如 `../dae/base.dae`）；`package://` URI 仅在构建系统支持时使用。

## 5. 顺序规范（连杆-关节-下一连杆）

文件顶层元素按**从基座出发的 DFS 前序**排列：输出一个连杆后，立即按序输出以它为
`parent` 的每个关节，每个关节紧跟其 `child` 连杆并递归展开：

```xml
<link name="torso_link">…</link>

<joint name="left_shoulder_pitch" type="revolute">
  <origin xyz="0 0.0983462 0.2484974" rpy="0 0 0"/>
  <parent link="torso_link"/>
  <child link="left_shoulder_pitch_link"/>
  <axis xyz="0 0.961261695 0.275637356"/>
  <limit lower="-3.07" upper="2.67" effort="25" velocity="18.84"/>
</joint>

<link name="left_shoulder_pitch_link">
  <inertial>
    <origin xyz="0.0051983 0.0442771 -0.0094234" rpy="0 0 0"/>
    <mass value="0.7097"/>
    <inertia ixx="0.0006492613" ixy="-0.0000599272" ixz="-0.0000103097" iyy="0.0005852764" iyz="0.0000092251" izz="0.0006093734"/>
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0.27925 0 0"/>
    <geometry><mesh filename="package://T4_v2/meshes/left_shoulder_pitch_link.stl" scale="0.001 0.001 0.001"/></geometry>
    <material name=""><color rgba="0.1 0.1 0.1 1"/></material>
  </visual>
</link>
```

- 同一父连杆的多个关节按原定义顺序排列（先左后右、先上肢后下肢）。
- 禁止「连杆-下一连杆-关节」的导出器风格排列。
- 该 DFS 顺序决定了关节在文件中的出现次序；**除非确有必要，不得更改既有模型的关节
  相对顺序**（部分解析器按出现顺序建立执行器/qpos 索引）。

## 6. 碰撞规范

- **几何体选择**：优先使用简单几何体（`box`、`cylinder`、`sphere`）近似外形，禁止直接
  使用网格做碰撞（`<collision><geometry><mesh>`）。
- **外形近似**：碰撞体以包围或简化真实外形为准，宁大勿小、宁简勿繁。
- **多个碰撞体**：一个连杆允许多个 `collision`，按从大到小排列（示例：T4 踝部由
  3 个 box 近似脚掌轮廓）。
- **独立碰撞连杆**：多个小碰撞体可放入独立哑连杆
  （`{关节名}_collision_{n}_link` + `type="fixed"` 关节挂载），如 D2 膝部
  （`left_front_knee_collision_1_link`）。
- **material**：collision 中不参与渲染，`material` 可省略；若保留（如 T4 踝部
  `rgba="0.76 0.76 0.76 0"`），仅作调试用途。

## 7. 视觉规范

- `visual` 用 CAD 网格（`mesh`）或简单几何体；一个连杆可含多个 `visual`（如手部
  连杆 = 腕部网格 + 手掌网格）。
- 网格原点/朝向与连杆坐标系不一致时用 `origin` 修正。
- 颜色统一用 `<color rgba>`；全局复用的颜色定义为顶层 `<material>` 并在 visual 中引用。

## 8. 格式规范

- **缩进**：2 空格，不使用 Tab。
- **自闭合**：无子元素的标签一律自闭合（`<origin …/>`、`<mass value="1"/>`）。
- **同行书写**：`geometry`、`material` 与其唯一子元素同行
  （`<geometry><mesh …/></geometry>`、`<material name=""><color …/></material>`）。
- **行宽**：紧凑优先，单行尽量不超过 140 字符（惯性矩阵等长属性允许单行书写）。
- **空行**：顶层元素之间空一行，子元素之间不空行。
- **注释**：
  - 禁止保留被注释掉的元素（旧 visual/collision/joint 等死代码）与导出器自动生成的头部注释。
  - 允许少量**分节导航注释**（如 `<!-- ========== left_front_leg ========== -->`），
    仅在链条众多、难以定位时使用。
- **属性引号**：一律双引号。

## 9. 校验

提交前对每个修改过的 URDF 执行：

```bash
xmllint --noout <file>.urdf          # XML 语法
check_urdf <file>.urdf               # URDF 结构（连杆/关节/树完整性）
```

格式化重排应保证语义等价：连杆/关节名称集合、parent/child 映射、关节出现顺序、
全部数值（浮点级）与改动前一致。仓库提供两个工具（`mj_sim/description/utils/`）：

```bash
# 重排+紧凑化：不带参数时处理已登记的 4 个模型 URDF；也可指定任意 URDF 文件。
# 原地写回前自动做语义等价校验，校验不通过则不写回。
python3 mj_sim/description/utils/urdf_reformat.py [file.urdf ...]

# 语义等价逐项对比：任意两个 URDF（如手动修改前后）之间校验。
python3 mj_sim/description/utils/urdf_verify.py <orig.urdf> <new.urdf> [<orig2> <new2> ...]
```

新增机器人模型时，将其 URDF 路径登记到 `urdf_reformat.py` 的 `KNOWN` 列表。

## 10. generated XML（urdf_to_xml.py 生成物）

`mj_sim/description/utils/urdf_to_xml.py` 基于 MuJoCo 原生 URDF 导入，从 URDF 自动生成 MJCF 模板，
补齐官方编译缺失的内容（浮动基、物理参数、motor、地面、传感器）。

```bash
python3 mj_sim/description/utils/urdf_to_xml.py [urdf ...]   # 缺省处理 KNOWN 登记的模型
python3 mj_sim/description/utils/urdf_to_xml.py --check ...  # 仅内存生成+编译校验，不写盘
```

- **配置**：`description/<模型>/urdf_to_xml.yaml`（每模型一份，可用 `extends:` 复用其它
  模型的配置；文件不存在时使用脚本内置默认值）。可配置：基座高度、default 关节参数、
  逐关节 armature/actuatorfrcrange（fnmatch pattern）、motor 的 ctrlrange 策略
  （`frcrange` 跟随关节限幅 / `effort` 取 URDF limit effort / 显式值）、地面与灯光模板、
  传感器开关、collision_rgba、`extras` 手工 MJCF 片段注入（相机等）。
- **输出**：`<模型>/xml/<机器人名>_generated.xml`；CMake 的 `*/xml/*.xml` 扫描会自动为其
  生成同名头文件。生成物定位为**模板/新模型起点**，与手写 XML 并存，不接入现有仿真包。
- **生成规则**：基座取 URDF 根连杆（加 freejoint + 根连杆惯性合成）；fixed 关节折叠
  （子 body 合并进父 body）；关节/执行器命名与 URDF 一致（与手写 XML 的旧名不同）；
  网格缺失时脚本告警（MuJoCo 会静默丢弃）；并联踝、特殊相机等复杂手工内容仍走
  `extras` 或手写 XML。
- **校验**：生成后自动重新编译并断言关节集合、motor 数、传感器维度、基座质量与
  URDF 一致，校验失败不写盘。

### 10.1 适用范围与边界

脚本定位为 **URDF 直接转录 + 配置驱动的物理/场景参数**，超出以下范围的内容需手工补充：

| 内容 | 处理方式 |
|------|----------|
| body/joint 树、惯性、网格、URDF 碰撞几何 | 自动转录（与 URDF 一一对应） |
| 浮动基、根连杆惯性、default 物理参数、motor、平面地面、imu site、传感器三件套 | 自动生成（参数来自 `urdf_to_xml.yaml`） |
| 复杂地形（楼梯/斜坡/障碍物等） | ❌ 手工编写 MJCF 片段 |
| 与 URDF 不同的碰撞体、接触排除对（`<contact>`/`exclude`） | ❌ 手工编写 MJCF 片段 |
| 相机、foot site、额外传感器、tendon/并联机构等 | ❌ 手工编写 MJCF 片段 |

手工内容统一写进本模型 `urdf_to_xml.yaml` 的 `patches` 段（仅在 `--final` 模式应用），
支持四种操作：

```yaml
patches:
  - xpath: "worldbody/body[@name='torso_link']"   # 在命中元素末尾追加 MJCF 片段
    append: |
      <geom pos="0 0 0.12" type="box" .../>
  - {op: remove, xpath: "worldbody/.//body[@name='x']/geom[not(@type)]"}   # 删除命中元素
  - {op: set, xpath: "...", set: {pos: "0.00155 0 0.3155"}}                # 覆盖属性
  - {op: insert, insert: 2, xpath: "...", xml: |                            # 指定位置插入
      ...}
```

补丁随生成一起注入，重新生成不丢失；**不要直接编辑 `*_generated.xml`**——该文件是
纯自动结果，下次运行脚本时会被覆盖。

### 10.2 最终 XML（不含 `_generated` 后缀）

`<模型>/xml/<模型>.xml`（如 `Z01_edm/xml/Z01_edm.xml`）是运行时使用的最终文件：

```bash
python3 mj_sim/description/utils/urdf_to_xml.py --final   # 最终文件 = 自动结果 + patches 手工补丁
```

- 定位：**generated + 最小手工补丁**。结构、命名、数值格式以 URDF 自动生成结果为准；
  patches 只补齐自动无法覆盖的内容（装饰几何、脚底碰撞方案、复杂地形、独立 body、
  手工调优值等），物理性细节与原手写 XML 一致。
- 三个模型的最终文件（Z01_edm.xml、Z01_edm_add_head.xml、D015_poc.xml）已由各自
  `urdf_to_xml.yaml` 的 patches 完整复现；与手写原 XML 的预期差异仅：命名、
  `size`/`default` 等以 generated 为准的段、物理等价的表示形式（如根惯性
  `fullinertia` vs 主轴化写法）。
- 修改最终文件：优先改 `urdf_to_xml.yaml` 的 patches 后重新 `--final`；小改动也可
  直接编辑 `<模型>.xml`（编辑前确认对应 patches 是否需同步，避免下次生成覆盖）。

## 11. CSV 参数表（read_urdf.py / csv_to_urdf.py）

每个模型的 URDF 在 `description/<模型>/csv/` 下配套两份参数表（每份表一行一个关节/连杆）：

```bash
python3 mj_sim/description/utils/read_urdf.py <模型>/urdf/<模型>.urdf -o <模型>/csv/
python3 mj_sim/description/utils/csv_to_urdf.py <模型>_joints.csv <模型>_links.csv -o <输出.urdf>
```

### 11.1 关节表 `<模型>_joints.csv`

| 列 | 说明 |
|----|------|
| `name`, `type`, `parent_link`, `child_link` | 关节名/类型/父连杆/子连杆 |
| `origin_xyz_x/y/z`, `origin_rpy_r/p/y` | 原点位置与姿态（**向量拆三列**） |
| `axis_x/y/z` | 转轴（拆三列） |
| `effort_limit`, `velocity_limit`, `lower_limit`, `upper_limit` | 限位与力矩/速度限制 |
| `armature` | **留空待手工填写**（URDF 无此属性） |
| `damping`, `friction` | 动力学参数 |

### 11.2 连杆表 `<模型>_links.csv`

| 列 | 说明 |
|----|------|
| `name`, `mass` | 连杆名与质量 |
| `com_xyz_x/y/z`, `com_rpy_r/p/y` | 质心位置与姿态（拆三列） |
| `inertia_ixx/ixy/ixz/iyy/iyz/izz` | 惯性全矩阵 |
| `stl_file` | visual mesh 文件 basename（去 package:// 前缀）；多个 mesh 用 `;` 分隔 |
| `stl_xyz_x/y/z`, `stl_rpy_r/p/y` | 对应 mesh 的位姿（拆三列，多 mesh 用 `;` 分隔） |
| `stl_scale` | mesh scale（多 mesh 用 `;` 分隔） |

### 11.3 约定与限制

- **数值**：一律 float 解析后规整化输出（`-0` → `0`、无多余尾随零）。
- **多 mesh**：一个连杆多个 visual mesh（如手部连杆）时，`stl_*` 列按 `;` 分隔对应分量。
- **CSV 信息边界**：CSV 只含惯性/关节参数与 visual mesh 引用，不含 collision、
  `<mujoco>` 扩展、材质等信息；`csv_to_urdf.py` 重建的 URDF 无这些段，需要时按本规范补充。
- 兼容性：`csv_to_urdf.py` 同时接受旧 schema（向量单列，如 `origin_xyz`）与新 schema。
