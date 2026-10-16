#!/usr/bin/env python3
"""URDF → MuJoCo XML 生成脚本（说明见 description/URDF_SPEC.md「generated XML」一节）。

基于 mujoco.MjSpec.from_string 的原生 URDF 导入（mjXURDF），自动补齐官方编译缺失的内容：
1. 浮动基：freejoint + 基座初始高度 + 根连杆惯性合成（导入器会丢弃根连杆 inertial）
2. 物理参数：default 段（damping/armature/frictionloss）+ 逐关节规则（fnmatch pattern）
3. 执行器：每个非 free 关节一个 motor（{joint}_motor + ctrlrange，header_generator 硬约定）
4. imu site + 传感器三件套（gyro/framequat/accelerometer）
5. 地面/灯光/天空盒/材质模板
6. 手工内容扩展点：collision_rgba 覆盖、extras 原始 MJCF 片段注入（相机等）

参数来自每模型 YAML 配置（description/<模型>/urdf_to_xml.yaml，不存在时用内置默认值）。

用法：
    python3 urdf_to_xml.py [选项] [urdf_path ...]
      urdf_path ...  缺省 = urdf_reformat.KNOWN 登记的模型 URDF
      -c/--config    指定 YAML 配置（缺省 <模型目录>/urdf_to_xml.yaml）
      -o/--output    覆盖输出路径（缺省 <模型目录>/xml/<机器人名>_generated.xml）
      --final        生成最终文件 <模型>/xml/<机器人名>.xml = 自动结果 + patches 手工补丁
      --check        仅内存生成 + 编译校验，不写盘
      -v/--verbose   打印每关节最终参数、未命中规则关节、缺失网格等详情
"""
import argparse
import fnmatch
import os
import sys

import mujoco
import numpy as np
import yaml
from lxml import etree

from urdf_reformat import KNOWN

# ---------------------------------------------------------------------------
# 内置默认配置（t4_std 风格；各模型可用 YAML 覆盖任意字段）
# ---------------------------------------------------------------------------
BUILTIN_DEFAULTS = {
    "model_name": None,             # <mujoco model=...>，缺省 = URDF robot name
    "meshdir": "../meshes/",        # 输出 XML 的 compiler meshdir（仅当 URDF 带 meshdir 时回写）
    "freejoint": {"body": None,     # 基座 body，缺省 = URDF 根连杆名
                  "pos": "0 0 0.5"},  # 基座初始位姿
    "option": None,                 # 例 {cone: pyramidal, impratio: 5}
    "size": {"njmax": 500, "nconmax": 1000},
    "default": {"joint": {"damping": 0.05, "armature": 0.01, "frictionloss": 0.1}},
    "joints": [],                   # [{pattern, armature, actuatorfrcrange, damping, frictionloss}]
    "motors": {"enabled": True, "include": ["*"], "exclude": [], "gear": 1,
               "ctrlrange": "frcrange",   # frcrange|effort|<数字>|"-a b"
               "ctrllimited": True, "rules": [], "order": []},
    "sensors": {"enabled": True, "site": "imu",
                "gyro": {"name": "base_ang_vel", "noise": 0.005, "cutoff": 34.9},
                "framequat": {"name": "base_rotation", "noise": 0.001},
                "accelerometer": {"name": "base_acc", "noise": 0.005}},
    "ground": {"enabled": True,
               "texture": {"name": "texplane", "type": "2d", "builtin": "checker",
                           "width": 512, "height": 512, "rgb1": "0.2 0.3 0.4",
                           "rgb2": "0.1 0.15 0.2", "mark": "cross", "markrgb": "0.8 0.8 0.8"},
               "material": {"name": "matplane", "texture": "texplane",
                            "reflectance": 0.3, "texrepeat": "1 1", "texuniform": "true"},
               "skybox": {"type": "skybox", "builtin": "gradient", "rgb1": "0.3 0.5 0.7",
                          "rgb2": "0 0 0", "width": 512, "height": 512},
               "geom": {"name": "ground", "type": "plane", "pos": "0 0 0",
                        "size": "0 0 1", "material": "matplane", "condim": 1},
               "lights": [{"directional": "true", "diffuse": ".4 .4 .4",
                           "specular": "0.1 0.1 0.1", "pos": "0 0 5.0",
                           "dir": "0 0 -1", "castshadow": "false"},
                          {"directional": "true", "diffuse": ".6 .6 .6",
                           "specular": "0.2 0.2 0.2", "pos": "0 0 4",
                            "dir": "0 0 -1", "castshadow": "false"}]},
    "visuals": {"strip_content_type": True,   # 剥离 asset mesh 的 content_type
                "collision_rgba": None},      # 覆盖无 contype 属性的非 mesh geom
    "imu_site": {"body": None, "pos": "0 0 0",  # pos 可为 "urdf:imu"（取 URDF imu 关节 origin）
                 "size": 0.01},
    "extras": [],                   # 兼容旧配置：等价 patches 的 append 操作
    "patches": [],                  # 手工补丁，仅 --final 模式应用：[{op, xpath, ...}]
    "validate": {"strict": True},   # 结构改造模型（关节改名/被动化）设 false，跳过集合断言
}

# 顶层元素规范顺序（MJCF schema）
TOP_ORDER = ["compiler", "option", "size", "custom", "default", "asset", "worldbody",
             "actuator", "sensor", "keyframe", "contact", "visual", "statistic",
             "tendon", "equality", "extension"]

PARSER = etree.XMLParser(remove_blank_text=True)


# ---------------------------------------------------------------------------
# 通用工具
# ---------------------------------------------------------------------------
def fmt_val(v):
    if isinstance(v, bool):
        return "true" if v else "false"
    return str(v)


def strify(d):
    return {k: fmt_val(v) for k, v in d.items() if v is not None}


def deep_merge(base, override, append_keys=("patches",)):
    out = dict(base)
    for k, v in override.items():
        if v is None and k in append_keys:
            continue                     # 空的 patches:（仅剩注释）不覆盖继承的补丁列表
        if isinstance(v, list) and k in append_keys and isinstance(out.get(k), list):
            out[k] = out[k] + v          # patches 累加（extends 链上增量补丁）
        elif isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load_config(path):
    """加载 YAML 配置，支持 extends 递归深合并；文件不存在返回 {}。"""
    if not path or not os.path.exists(path):
        return {}
    with open(path) as f:
        cfg = yaml.safe_load(f) or {}
    ext = cfg.pop("extends", None)
    if ext:
        ext_path = os.path.normpath(os.path.join(os.path.dirname(path), ext))
        if not os.path.exists(ext_path):
            raise SystemExit(f"extends 指向的配置不存在: {ext} ({ext_path})")
        cfg = deep_merge(load_config(ext_path), cfg)
    return cfg


def norm_range(v):
    """数字 → 对称范围 "-v v"；字符串 → 原样。"""
    if isinstance(v, (int, float)):
        return f"-{v} {v}"
    return str(v)


def fmt_ctrlrange(s):
    """motor ctrlrange 的整数值保留 .0（对齐手写 XML 的 motor 写法）。"""
    out = []
    for t in s.split():
        try:
            f = float(t)
        except ValueError:
            out.append(t)
            continue
        out.append(f"{f:.1f}" if f.is_integer() else t)
    return " ".join(out)


# ---------------------------------------------------------------------------
# 导入层
# ---------------------------------------------------------------------------
def import_urdf(urdf_path):
    """解析 URDF 并调用 MuJoCo 原生导入，返回 MJCF 文本与 URDF 侧信息。"""
    text = open(urdf_path).read()
    root = etree.fromstring(text.encode(), PARSER)
    if root.tag != "robot":
        raise SystemExit(f"{urdf_path}: 根元素不是 <robot>")
    robot_name = root.get("name")

    # URDF 侧信息：effort 字典 / 根连杆 / imu origin / mesh 集合
    links = {l.get("name"): l for l in root.findall("link")}
    child_names = {j.find("child").get("link") for j in root.findall("joint")}
    root_names = [n for n in links if n not in child_names]
    efforts = {}
    imu_origin = None
    for j in root.findall("joint"):
        if j.get("name") == "imu":
            o = j.find("origin")
            imu_origin = o.get("xyz") if o is not None else "0 0 0"
        if j.get("type") != "fixed":
            lim = j.find("limit")
            if lim is not None and lim.get("effort"):
                efforts[j.get("name")] = float(lim.get("effort"))
    urdf_meshes = {os.path.splitext(os.path.basename(m.get("filename")))[0]
                   for m in root.iter("mesh")}

    # meshdir 相对路径按 cwd 解析且导入时立即加载网格 → 临时替换为绝对路径
    comp = root.find("mujoco/compiler")
    meshdir_rel = comp.get("meshdir") if comp is not None else None
    if meshdir_rel:
        meshdir_abs = os.path.normpath(
            os.path.join(os.path.dirname(os.path.abspath(urdf_path)), meshdir_rel))
        text = text.replace(f'meshdir="{meshdir_rel}"', f'meshdir="{meshdir_abs}"')

    try:
        spec = mujoco.MjSpec.from_string(text)
    except Exception as e:
        raise SystemExit(f"URDF 导入失败 {urdf_path}: {e}")
    xml_text = spec.to_xml()
    return {"robot_name": robot_name, "root_names": root_names, "root_link": links,
            "efforts": efforts, "imu_origin": imu_origin, "urdf_meshes": urdf_meshes,
            "meshdir_rel": meshdir_rel, "xml_text": xml_text}


# ---------------------------------------------------------------------------
# 后处理步骤
# ---------------------------------------------------------------------------
def clean_tree(root, cfg, urdf_meshes):
    """meshdir 回写相对值、剥离 content_type、缺失网格告警。"""
    comp = root.find("compiler")
    if comp is not None and cfg["meshdir"] and root.xpath(".//asset/mesh"):
        comp.set("meshdir", cfg["meshdir"])
    elif comp is not None:
        comp.attrib.pop("meshdir", None)  # 无网格资产时不写 meshdir
    if cfg["visuals"]["strip_content_type"]:
        for m in root.xpath(".//asset/mesh"):
            m.attrib.pop("content_type", None)
    asset_meshes = {m.get("name") for m in root.xpath(".//asset/mesh")}
    missing = sorted(urdf_meshes - asset_meshes)
    return missing


def add_default_option_size(root, cfg):
    """按 schema 顺序插入 default/option/size（顺序最后由 sort_top 保证）。"""
    if cfg.get("default"):
        d = etree.Element("default")
        jd = cfg["default"].get("joint")
        if jd:
            d.append(etree.Element("joint", strify(jd)))
        root.insert(0, d)
    if cfg.get("option"):
        root.insert(0, etree.Element("option", strify(cfg["option"])))
    if cfg.get("size"):
        root.insert(0, etree.Element("size", strify(cfg["size"])))


def synthesize_inertial(link):
    """从 URDF 根连杆合成 <inertial>（fullinertia 不带 quat，rpy 折进矩阵）。"""
    inert = link.find("inertial")
    if inert is None:
        return None
    o = inert.find("origin")
    xyz = o.get("xyz") if o is not None else "0 0 0"
    rpy = [float(x) for x in (o.get("rpy") if o is not None else "0 0 0").split()]
    mass = float(inert.find("mass").get("value"))
    ia = inert.find("inertia")
    i = np.array([[float(ia.get("ixx")), float(ia.get("ixy")), float(ia.get("ixz"))],
                  [float(ia.get("ixy")), float(ia.get("iyy")), float(ia.get("iyz"))],
                  [float(ia.get("ixz")), float(ia.get("iyz")), float(ia.get("izz"))]])
    if any(rpy):
        q = np.zeros(4)
        rm = np.zeros(9)
        mujoco.mju_euler2Quat(q, np.array(rpy), "xyz")
        mujoco.mju_quat2Mat(rm, q)
        r = rm.reshape(3, 3)
        i = r.T @ i @ r
    full = f"{i[0, 0]:.9g} {i[1, 1]:.9g} {i[2, 2]:.9g} {i[0, 1]:.9g} {i[0, 2]:.9g} {i[1, 2]:.9g}"
    return etree.Element("inertial", {"pos": xyz, "mass": str(mass), "fullinertia": full})


def add_freejoint_base(root, cfg, info):
    """浮动基：导入器不生成根连杆 body（其 geom 直接挂在 worldbody），
    新建基座 body 包住 worldbody 全部子元素，并插入 freejoint/合成惯性/imu site。"""
    base_name = cfg["freejoint"]["body"] or info["root_names"][0]
    if len(info["root_names"]) > 1:
        print(f"警告: URDF 有多个根连杆 {info['root_names']}，仅 '{base_name}' 作为浮动基")
    world = root.find("worldbody")
    body = etree.Element("body", {"name": base_name, "pos": cfg["freejoint"]["pos"]})
    for child in list(world):
        world.remove(child)
        body.append(child)
    world.insert(0, body)

    body.insert(0, etree.Element("joint", {"type": "free"}))
    link = info["root_link"].get(base_name)
    if link is not None:
        inert = synthesize_inertial(link)
        if inert is not None:
            body.insert(1, inert)
        else:
            print(f"警告: 根连杆 '{base_name}' 无 <inertial>，模型可能无法编译")

    pos = cfg["imu_site"]["pos"]
    if pos == "urdf:imu":
        pos = info["imu_origin"] or "0 0 0"
    body.insert(2, etree.Element("site", {"name": cfg["sensors"]["site"],
                                          "size": fmt_val(cfg["imu_site"]["size"]),
                                          "pos": pos}))


def apply_joint_rules(root, rules, verbose):
    """fnmatch 逐关节规则（按序首个命中生效），返回未命中关节清单。"""
    joints = root.xpath(".//worldbody//joint[@name]")
    unmatched = []
    for j in joints:
        name = j.get("name")
        for rule in rules:
            if fnmatch.fnmatchcase(name, rule["pattern"]):
                for k in ("armature", "damping", "frictionloss"):
                    if rule.get(k) is not None:
                        j.set(k, fmt_val(rule[k]))
                fr = rule.get("actuatorfrcrange")
                if fr is not None:
                    j.set("actuatorfrcrange", norm_range(fr))
                break
        else:
            unmatched.append(name)
    if verbose and unmatched:
        print(f"  [verbose] 未命中任何关节规则的关节: {', '.join(unmatched) or '(无)'}")
    return unmatched


def resolve_ctrlrange(name, joint_el, mc, efforts):
    for rule in mc["rules"]:
        if fnmatch.fnmatchcase(name, rule["pattern"]) and rule.get("ctrlrange") is not None:
            return norm_range(rule["ctrlrange"])
    s = mc["ctrlrange"]
    if isinstance(s, str):
        if s == "frcrange":
            fr = joint_el.get("actuatorfrcrange")
            return fr if fr else "-1 1"
        if s == "effort":
            e = efforts.get(name)
            return f"-{e} {e}" if e is not None else "-1 1"
        return s
    return norm_range(s)


def add_motors(root, cfg, efforts):
    """每个非 free 关节一个 motor（默认文档顺序 = URDF 关节顺序；motors.order 可调整）。"""
    mc = cfg["motors"]
    if not mc["enabled"]:
        return []
    joints = root.xpath(".//worldbody//joint[@name]")
    act = root.find("actuator")
    if act is None:
        act = etree.Element("actuator")
        root.append(act)
    made = []                      # [(关节名, motor 属性)]，先按文档顺序收集
    for j in joints:
        name = j.get("name")
        if not any(fnmatch.fnmatchcase(name, p) for p in mc["include"]):
            continue
        if any(fnmatch.fnmatchcase(name, p) for p in mc["exclude"]):
            continue
        attrs = {"name": f"{name}_motor", "joint": name,
                 "ctrlrange": fmt_ctrlrange(resolve_ctrlrange(name, j, mc, efforts))}
        if mc["ctrllimited"]:
            attrs["ctrllimited"] = "true"
        gear = mc["gear"]
        for rule in mc["rules"]:
            if fnmatch.fnmatchcase(name, rule["pattern"]) and rule.get("gear") is not None:
                gear = rule["gear"]
                break
        if gear != 1:
            attrs["gear"] = fmt_val(gear)
        made.append((name, attrs))
    order = mc.get("order") or []
    if order:
        def rank(item):
            for i, p in enumerate(order):
                if fnmatch.fnmatchcase(item[0], p):
                    return i
            return len(order)
        made.sort(key=rank)        # 稳定排序：组内保持文档顺序，未匹配组在末尾
    for name, attrs in made:
        act.append(etree.Element("motor", attrs))
    return [n for n, _ in made]


def add_sensors(root, cfg):
    """传感器三件套，挂 imu site（每类一个，header_generator 约定）。"""
    sc = cfg["sensors"]
    if not sc["enabled"]:
        return
    s = etree.Element("sensor")
    root.append(s)
    s.append(etree.Element("gyro", strify({"name": sc["gyro"]["name"],
                                           "site": sc["site"], **{k: v for k, v in sc["gyro"].items()
                                                                  if k != "name"}})))
    s.append(etree.Element("framequat", strify({"name": sc["framequat"]["name"],
                                                "objtype": "site", "objname": sc["site"],
                                                **{k: v for k, v in sc["framequat"].items()
                                                   if k != "name"}})))
    s.append(etree.Element("accelerometer", strify({"name": sc["accelerometer"]["name"],
                                                    "site": sc["site"],
                                                    **{k: v for k, v in sc["accelerometer"].items()
                                                       if k != "name"}})))


def add_ground(root, cfg):
    """地面/灯光/天空盒/材质模板（地面与灯光直接挂 worldbody，与基座 body 平级）。"""
    gc = cfg["ground"]
    if not gc.get("enabled", True):
        return
    asset = root.find("asset")
    if asset is None:
        asset = etree.Element("asset")
        root.append(asset)
    if gc.get("skybox"):
        asset.append(etree.Element("texture", strify(gc["skybox"])))
    if gc.get("texture"):
        asset.append(etree.Element("texture", strify(gc["texture"])))
    if gc.get("material"):
        asset.append(etree.Element("material", strify(gc["material"])))
    world = root.find("worldbody")
    for light in gc.get("lights") or []:
        world.append(etree.Element("light", strify(light)))
    world.append(etree.Element("geom", strify(gc["geom"])))


def apply_visuals(root, cfg):
    rgba = cfg["visuals"].get("collision_rgba")
    if not rgba:
        return 0
    n = 0
    for g in root.xpath(".//worldbody/body//geom"):
        if "contype" not in g.attrib and "mesh" not in g.attrib:
            g.set("rgba", rgba)
            n += 1
    return n


def apply_patches(root, cfg):
    """手工补丁（仅在 --final 模式应用，使 *_generated.xml 保持纯自动结果）。

    patches: [{op, xpath, ...}]，按序执行；op ∈
      append  在命中元素末尾追加 xml 片段（即原 extras，向后兼容）
      insert  在命中元素第 insert 个子元素处插入 xml 片段
      remove  删除命中的全部元素
      set     覆盖命中元素的属性（set: {attr: value}）
    """
    items = list(cfg.get("patches") or [])
    items += [dict(e, op="append") for e in cfg.get("extras") or []]  # 兼容旧 extras
    n = 0
    for item in items:
        op = item.get("op", "append")
        xp = item["xpath"]
        if xp.startswith("mujoco/"):  # 路径相对 <mujoco> 根元素评估，去掉冗余前缀
            xp = xp[len("mujoco/"):]
        targets = root.xpath(xp)
        if op == "remove":
            for t in targets:
                t.getparent().remove(t)
            n += len(targets)
            if not targets:
                print(f"警告: patches remove 未命中: {xp}")
            continue
        if len(targets) != 1:
            print(f"警告: patches {op} 命中 {len(targets)} 个节点（应为 1）: {xp}，跳过")
            continue
        t = targets[0]
        if op == "set":
            for k, v in item["set"].items():
                t.set(k, fmt_val(v))
        else:
            frag = etree.fromstring(f"<tmp>{item.get('xml') or item.get('append')}</tmp>".encode(),
                                    PARSER)
            if op == "insert":
                idx = int(item.get("insert", 0))
                for i, e in enumerate(frag):
                    t.insert(idx + i, e)
            else:
                for e in frag:
                    t.append(e)
        n += 1
    return n


def sort_top(root):
    rank = {t: i for i, t in enumerate(TOP_ORDER)}
    children = list(root)
    children.sort(key=lambda c: rank.get(c.tag, 100))
    root[:] = children


# ---------------------------------------------------------------------------
# 验证层
# ---------------------------------------------------------------------------
def validate(text, model_dir, cfg, info, sensors_enabled):
    """重新导入 + 编译 + 结构断言，返回错误清单。"""
    errors = []
    try:
        vtext = text
        if cfg["meshdir"]:
            # cfg 的 meshdir 相对输出文件（xml/ 目录）解析
            meshdir_abs = os.path.normpath(os.path.join(model_dir, "xml", cfg["meshdir"]))
            vtext = vtext.replace(f'meshdir="{cfg["meshdir"]}"', f'meshdir="{meshdir_abs}"')
        spec = mujoco.MjSpec.from_string(vtext)
        model = spec.compile()
    except Exception as e:
        return [f"编译失败: {e}"]

    revolute = set(info["efforts"])  # efforts 只含非 fixed 关节
    named = set()
    for i in range(model.njnt):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if n:
            named.add(n)
    if cfg["validate"]["strict"]:
        if named != revolute:
            errors.append(f"关节集合不一致: 缺 {sorted(revolute - named)}，多 {sorted(named - revolute)}")
        if model.nu != len(revolute):
            errors.append(f"motor 数 {model.nu} != 非 free 关节数 {len(revolute)}")
    if sensors_enabled and model.nsensordata != 10:
        errors.append(f"nsensordata = {model.nsensordata}（启用传感器时应为 10）")
    base_name = cfg["freejoint"]["body"] or info["root_names"][0]
    link = info["root_link"].get(base_name)
    if link is not None and link.find("inertial") is not None:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, base_name)
        if bid >= 0:
            m = float(link.find("inertial/mass").get("value"))
            if abs(model.body_mass[bid] - m) > 1e-6:
                errors.append(f"基座质量 {model.body_mass[bid]} != URDF 根连杆质量 {m}")
    return errors


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def process(urdf_path, args):
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
    model_dir = os.path.dirname(urdf_dir)
    cfg_path = args.config or os.path.join(model_dir, "urdf_to_xml.yaml")
    cfg = deep_merge(BUILTIN_DEFAULTS, load_config(cfg_path))
    if not os.path.exists(cfg_path):
        print(f"提示: 未找到配置 {cfg_path}，使用内置默认值")

    info = import_urdf(urdf_path)
    root = etree.fromstring(info["xml_text"].encode(), PARSER)
    if cfg["model_name"]:
        root.set("model", cfg["model_name"])

    missing = clean_tree(root, cfg, info["urdf_meshes"])
    add_default_option_size(root, cfg)
    add_freejoint_base(root, cfg, info)
    unmatched = apply_joint_rules(root, cfg["joints"], args.verbose)
    motors = add_motors(root, cfg, info["efforts"])
    add_sensors(root, cfg)
    add_ground(root, cfg)
    n_rgba = apply_visuals(root, cfg)
    n_patch = apply_patches(root, cfg) if args.final else 0  # 手工补丁只进最终文件
    sort_top(root)

    out_text = etree.tostring(root, pretty_print=True, encoding="unicode") + "\n"
    errors = validate(out_text, model_dir, cfg, info, cfg["sensors"]["enabled"])
    if errors:
        print(f"FAIL {urdf_path}（校验未通过）")
        for e in errors:
            print("  -", e)
        return False

    if args.output:
        out_path = args.output
    else:
        xml_dir = os.path.join(model_dir, "xml")
        if not os.path.isdir(xml_dir):
            raise SystemExit(f"输出目录不存在: {xml_dir}（不自动创建，请先建目录）")
        suffix = "" if args.final else "_generated"
        out_path = os.path.join(xml_dir, f"{info['robot_name']}{suffix}.xml")
    if not args.check:
        with open(out_path, "w") as f:
            f.write(out_text)
    print(f"OK   {urdf_path} -> {out_path}"
          f"（{len(motors)} motors，{len(unmatched)} 关节未命中规则，"
          f"{len(missing)} 网格缺失，patches {n_patch} 处，collision_rgba {n_rgba} 个）")
    if missing:
        print(f"  警告: URDF 引用但仓库缺失的网格（已被静默丢弃）: {', '.join(missing)}")
    if args.verbose:
        for j in root.xpath(".//worldbody//joint[@name]"):
            print(f"  [verbose] {j.get('name')}: armature={j.get('armature', '-')} "
                  f"frcrange={j.get('actuatorfrcrange', '-')}")
    return True


def main():
    ap = argparse.ArgumentParser(description="URDF → MuJoCo XML 生成（含浮动基/物理参数/motor/地面）")
    ap.add_argument("urdf_paths", nargs="*", help="URDF 文件（缺省：urdf_reformat.KNOWN）")
    ap.add_argument("-c", "--config", help="YAML 配置（缺省 <模型目录>/urdf_to_xml.yaml）")
    ap.add_argument("-o", "--output", help="输出路径（缺省 <模型目录>/xml/<机器人名>_generated.xml）")
    ap.add_argument("--final", action="store_true",
                    help="生成最终文件 <模型目录>/xml/<机器人名>.xml = 自动结果 + patches 手工补丁")
    ap.add_argument("--check", action="store_true", help="仅内存生成+校验，不写盘")
    ap.add_argument("-v", "--verbose", action="store_true", help="打印每关节最终参数等详情")
    args = ap.parse_args()

    if args.output and len(args.urdf_paths) > 1:
        raise SystemExit("-o 只能配合单个 URDF 使用")
    if not args.urdf_paths:
        base = os.path.dirname(os.path.abspath(__file__))
        args.urdf_paths = [os.path.normpath(os.path.join(base, p)) for p in KNOWN]
    failed = sum(1 for p in args.urdf_paths if not process(p, args))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
