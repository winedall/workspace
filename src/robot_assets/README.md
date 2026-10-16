# zl_robot_assets — 机器人模型描述文件资产包

存放各型号机器人的模型描述文件与配套工具链:

- **URDF**(`<模型>/urdf/`)为参考模型,是结构/参数维护的入口;
- **MJCF**(`<模型>/xml/`)是 MuJoCo 仿真运行时实际加载的模型,由 URDF 自动生成并叠加手工补丁;
- **CSV**(`<模型>/csv/`)为关节/连杆参数表,便于批量查看与修改;
- **网格**(`<模型>/meshes/`)为 STL 视觉网格(CAD 以 mm 导出,使用时统一 `scale="0.001 0.001 0.001"`)。

完整规范(命名、元素顺序、数值格式、碰撞/视觉、生成规则)见 [URDF_SPEC.md](URDF_SPEC.md)。

## 机器人命名

命名形式:`{产品型号}_{阶段}_{特殊硬件改动}`(特殊硬件改动可省略)。

| 元素 | 说明 |
|------|------|
| 产品型号 | D2→**D015**、D4→**D040**、T4→**Z01** |
| 阶段 | `poc` 原理样机 / `edm` 工程样机 / `std` 量产 |
| 特殊硬件改动 | 与标版的最大区别,如 `add_head`、`parallel_ankle` |

文件夹名、URDF 文件名、URDF 内 `<robot name>` 三者保持一致。

## 当前模型

| 模型 | 型号 | 说明 |
|------|------|------|
| `D015_poc` | D2 | 四足原理样机,最终 XML 含楼梯/斜坡地形 patches |
| `D040_poc` | D4 | 仅手写 XML(通过 `<include>` 组合),无 URDF |
| `Z01_edm` | T4 | 工程样机,URDF 与生成配置的基准 |
| `Z01_edm_add_head` | T4 | 加头部版本(配置继承 Z01_edm) |
| `Z01_edm_parallel_ankle` | T4 | 并联踝改造,仅手写 XML |
| `Z01_std` | T4 | 量产版,配置与 Z01_edm 相互独立(std 系列配置基准) |
| `Z01_std_add_head` | T4 | 量产加头版(配置继承 Z01_std) |
| `Z01_std_parallel_ankle` | T4 | 量产并联踝版(配置继承 Z01_std_add_head),patches 实现并联机构改造 |

每个完整模型目录的结构:

```
<模型>/
├── urdf_to_xml.yaml          # URDF → XML 生成配置(可 extends 复用其它模型)
├── urdf/<模型>.urdf          # 参考模型
├── csv/<模型>_joints.csv     # 关节参数表(一行一个关节)
├── csv/<模型>_links.csv      # 连杆参数表(一行一个连杆)
├── xml/<模型>_generated.xml  # 自动生成模板,勿手改
├── xml/<模型>.xml            # 运行时加载 = 自动结果 + patches 手工补丁
└── meshes/                   # STL 网格
```

## 工具链

`utils/` 下脚本形成完整流水线:

```
CSV ⇄ URDF ──生成──▶ *_generated.xml ──+ patches──▶ 最终 .xml ──▶ C++ 头文件
(手工维护)  (规范校验)   (自动,勿手改)    (yaml 手工补丁)    (运行时加载)
```

| 脚本 | 作用 |
|------|------|
| `read_urdf.py` | URDF → CSV 参数表(关节表含 armature 空列待手工填写) |
| `csv_to_urdf.py` | CSV → URDF 反向重建(兼容新旧 schema,不含 visual/collision) |
| `urdf_reformat.py` | 重排为「连杆→关节→下一连杆」DFS 紧凑格式并规整数值,写回前自动语义校验 |
| `urdf_verify.py` | 两个 URDF 语义等价对比(名称/映射/顺序/浮点级数值) |
| `urdf_to_xml.py` | **核心生成器**:URDF → MJCF(详见下节) |
| `header_generator.py` | MJCF → C++ 头文件(关节/执行器枚举等,已接入 CMake 的 `*/xml/*.xml` 扫描) |
| `align_all_joint.py` | 关节轴对齐 |

### urdf_to_xml.py 生成规则

基于 MuJoCo 原生 URDF 导入,自动补齐官方编译缺失的内容:浮动基与根连杆惯性、default
物理参数与逐关节规则(fnmatch pattern)、每关节 motor、imu site 与传感器三件套、地面/灯光。

每模型一份 `urdf_to_xml.yaml` 配置(支持 `extends:` 继承,如 `Z01_std_add_head` 继承
`Z01_std`)。注意合并语义:**`patches` 沿继承链累加**(增量补丁,不要在子配置里重复列出
父配置已有的补丁),而 `joints`/`motors`/`extras` 等**列表整体替换**,覆盖时需写全:

- **可配置项**:基座初始高度、逐关节 armature/actuatorfrcrange、motor ctrlrange 策略
  (`frcrange` 跟随限幅 / `effort` 取 URDF limit / 显式值)、地面与灯光模板、传感器开关、
  collision_rgba、`extras` 手工 MJCF 片段注入(相机等);
- **patches**(仅 `--final` 应用):xpath 定位的 `append`/`remove`/`set`/`insert` 四种补丁,
  补齐复杂地形、并联机构、脚底碰撞方案等自动无法覆盖的内容;重新生成不丢失;
- **校验**:生成后自动重新编译并断言关节集合、motor 数、传感器维度、基座质量与 URDF 一致,
  失败不写盘。

## 常用命令

在仓库根目录执行:

```bash
# URDF 格式化(缺省处理 KNOWN 登记的模型;写回前自动语义校验)
python3 mj_sim/description/utils/urdf_reformat.py [file.urdf ...]

# 语义等价对比(如手动修改前后)
python3 mj_sim/description/utils/urdf_verify.py <orig.urdf> <new.urdf>

# 生成 MJCF 模板(缺省处理 KNOWN 登记的模型)
python3 mj_sim/description/utils/urdf_to_xml.py [urdf ...]
python3 mj_sim/description/utils/urdf_to_xml.py --check ...   # 仅内存生成+编译校验,不写盘
python3 mj_sim/description/utils/urdf_to_xml.py --final       # 最终 XML = 自动结果 + patches

# CSV 参数表导出 / 重建
python3 mj_sim/description/utils/read_urdf.py <模型>/urdf/<模型>.urdf -o <模型>/csv/
python3 mj_sim/description/utils/csv_to_urdf.py <模型>_joints.csv <模型>_links.csv -o <输出.urdf>

# 语法 / 结构校验
xmllint --noout <file>.urdf
check_urdf <file>.urdf
```

## 修改流程

1. **结构与惯性参数**:修改 URDF(或经 CSV 重建),跑 `urdf_reformat.py` 规整并自动校验;
2. **生成 MJCF**:改 `urdf_to_xml.yaml`(物理参数、patches),跑 `urdf_to_xml.py --check`
   验证、`--final` 产出最终 XML;
3. **新增模型**:按命名规范建目录,补充 urdf/csv/yaml,并将 URDF 路径登记到
   `urdf_reformat.py` 的 `KNOWN` 列表;
4. **注意**:不要直接编辑 `*_generated.xml`(下次生成会覆盖);最终文件的修改优先走
   `patches`,小改动也可直接编辑 `<模型>.xml`(编辑前确认对应 patches 是否需同步)。
