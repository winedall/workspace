#!/usr/bin/env python3
"""将 URDF 重排为紧凑的「连杆→关节→下一连杆」格式（见 description/URDF_SPEC.md）。

规则：
1. 顶层元素按 DFS 前序遍历：先输出连杆，再依次输出以它为 parent 的关节（按原文件顺序），
   每个关节后面紧跟其 child 连杆并递归。关节的先后顺序与原文件完全一致（基座→各链条）。
2. 删除全部注释（包括被注释掉的旧 visual/collision 块、导出器头部注释）；
   四足模型的 `{left|right}_{front|rear}_hip_roll` 关节前保留一条分节导航注释。
3. 子元素紧凑排版：2 空格缩进，无子元素的标签自闭合，单行书写；
   geometry / material 与其唯一子元素同行。
4. 数值规范化：删除 -0 / 无意义尾随零；E-3 / E-6 等科学计数法按小数点移位精确转为
   普通小数（极端量级的科学计数法保留）。数值与原始文件完全相等（浮点级）。
5. 属性顺序规范化：origin 为 xyz 在前 rpy 在后，其余按 URDF_SPEC.md 顺序。

用法：
    python3 urdf_reformat.py [file.urdf ...]
不带参数时处理 description/ 下已登记的模型 URDF。写回前自动做语义等价校验
（见 urdf_verify.py），校验不通过则不写回。
"""
import os
import re
import sys
from lxml import etree
from xml.sax.saxutils import quoteattr

from urdf_verify import compare_roots

# 已登记的模型 URDF（相对本脚本目录）
KNOWN = [
    "../Z01_edm/urdf/Z01_edm.urdf",
    "../Z01_edm_add_head/urdf/Z01_edm_add_head.urdf",
    "../D015_poc/urdf/D015_poc.urdf",
    "../D015_poc/urdf/D015_poc_align.urdf",
]

# 需要数值规范化的属性
NUMERIC_ATTRS = {
    "xyz", "rpy", "rgba", "size", "radius", "length", "scale", "value",
    "lower", "upper", "effort", "velocity", "damping", "friction",
    "ixx", "ixy", "ixz", "iyy", "iyz", "izz",
}

# 各标签的属性规范顺序（未列出的属性保持原相对顺序跟在后面）
ATTR_ORDER = {
    "robot":     ["name", "{http://www.ros.org/wiki/xacro}xacro"],
    "link":      ["name"],
    "joint":     ["name", "type", "dont_collapse"],
    "parent":    ["link"],
    "child":     ["link"],
    "origin":    ["xyz", "rpy"],
    "axis":      ["xyz"],
    "limit":     ["lower", "upper", "effort", "velocity"],
    "dynamics":  ["damping", "friction"],
    "mass":      ["value"],
    "inertia":   ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"],
    "mesh":      ["filename", "scale"],
    "box":       ["size"],
    "cylinder":  ["radius", "length"],
    "sphere":    ["radius"],
    "material":  ["name"],
    "color":     ["rgba"],
}

# 各容器的子元素规范顺序
CHILD_ORDER = {
    "joint":    ["origin", "parent", "child", "axis", "dynamics", "limit"],
    "link":     ["inertial", "visual", "collision"],
    "inertial": ["origin", "mass", "inertia"],
    "visual":   ["origin", "geometry", "material"],
    "collision":["origin", "geometry", "material"],
    "material": ["color", "texture"],
}

# 只有一个子元素且无文本的容器，与子元素同行书写
INLINE = {"geometry", "material"}

NUM_RE = re.compile(r"^([+-]?(?:\d+(?:\.\d*)?|\.\d+))(?:[eE]([+-]?\d+))?$")
NS_RE = re.compile(r"^\{([^}]*)\}(.+)$")


def shift_point(mant: str, exp: int) -> str:
    """将 mant × 10^exp 精确转为普通十进制字符串（仅靠小数点移位）。"""
    neg = mant.startswith("-")
    if neg:
        mant = mant[1:]
    i, _, d = mant.partition(".")
    digits = i + d
    p = len(i) + exp
    if p <= 0:
        out = "0." + "0" * (-p) + digits
    elif p >= len(digits):
        out = digits + "0" * (p - len(digits))
    else:
        out = digits[:p] + "." + digits[p:]
    if "." in out:
        ii, dd = out.split(".")
        dd = dd.rstrip("0")
        out = (ii.lstrip("0") or "0") + ("." + dd if dd else "")
    else:
        out = out.lstrip("0") or "0"
    return ("-" if neg else "") + out


def norm_num(tok: str) -> str:
    m = NUM_RE.match(tok)
    if not m:
        return tok
    mant, exp = m.group(1), int(m.group(2) or "0")
    if exp != 0 and -6 <= exp <= 4:
        return shift_point(mant, exp)
    if exp != 0:
        return tok  # 极端量级的科学计数法，原样保留
    if float(tok) == 0:
        return "0"
    if "." in tok:  # 仅删除无意义尾随零
        i, d = tok.split(".")
        d = d.rstrip("0")
        return f"{i}.{d}" if d else i
    return tok


def norm_value(tag: str, attr: str, val: str) -> str:
    if attr in NUMERIC_ATTRS:
        return " ".join(norm_num(t) for t in val.split())
    return val


def sort_children(elem):
    order = CHILD_ORDER.get(elem.tag)
    if not order:
        return
    rank = {t: i for i, t in enumerate(order)}
    # 注意：lxml 迭代可能返回不同的代理对象，须先物化，保证 id() 稳定
    children = list(elem)
    pos = {id(c): i for i, c in enumerate(children)}
    children.sort(key=lambda c: rank.get(c.tag, 1000 + pos[id(c)]))
    elem[:] = children


def fmt_attrs(elem):
    attrs = [(k, v) for k, v in elem.attrib.items()]
    order = ATTR_ORDER.get(elem.tag, [])
    rank = {k: i for i, k in enumerate(order)}
    attrs.sort(key=lambda kv: rank.get(kv[0], 1000))
    parts = []
    for k, v in attrs:
        m = NS_RE.match(k)
        if m:  # 命名空间声明，还原为 xmlns: 前缀
            parts.append(f"xmlns:{m.group(2)}={quoteattr(m.group(1))}")
        else:
            parts.append(f"{k}={quoteattr(norm_value(elem.tag, k, v))}")
    return (" " + " ".join(parts)) if parts else ""


def serialize(elem, level, out):
    ind = "  " * level
    sort_children(elem)
    a = fmt_attrs(elem)
    if len(elem) == 0:
        out.append(f"{ind}<{elem.tag}{a}/>")
    elif elem.tag in INLINE and len(elem) == 1 and not (elem.text or "").strip():
        parts = [f"{ind}<{elem.tag}{a}>"]
        serialize(elem[0], 0, parts)
        out.append("".join(parts) + f"</{elem.tag}>")
    else:
        out.append(f"{ind}<{elem.tag}{a}>")
        for child in elem:
            serialize(child, level + 1, out)
        out.append(f"{ind}</{elem.tag}>")


def reorder_top(robot):
    """返回重排后的顶层元素列表：materials、mujoco，然后 DFS 连杆-关节-连杆。"""
    materials = [c for c in robot if c.tag == "material"]
    mujoco = [c for c in robot if c.tag == "mujoco"]
    links = {c.get("name"): c for c in robot if c.tag == "link"}
    joints = [c for c in robot if c.tag == "joint"]

    parent_joints = {}
    for j in joints:
        parent_joints.setdefault(j.find("parent").get("link"), []).append(j)
    child_of = {j.get("name"): j.find("child").get("link") for j in joints}

    roots = [l for l in links.values() if l.get("name") not in set(child_of.values())]
    visited = set()
    emitted_joints = set()
    ordered = []

    def dfs(link):
        name = link.get("name")
        if name in visited:
            return
        visited.add(name)
        ordered.append(link)
        for j in parent_joints.get(name, []):
            ordered.append(j)
            emitted_joints.add(j.get("name"))
            child = child_of[j.get("name")]
            if child in links and child not in visited:
                dfs(links[child])

    for r in roots:
        dfs(r)
    for l in links.values():  # 兜底：未被引用的连杆
        if l.get("name") not in visited:
            ordered.append(l)
    for j in joints:  # 兜底：未被引用的关节
        if j.get("name") not in emitted_joints:
            ordered.append(j)
    return materials + mujoco + ordered


def format_text(robot) -> str:
    """把 <robot> 根元素格式化为规范文本。"""
    out = ['<?xml version="1.0" encoding="utf-8"?>']
    out.append(f"<robot{fmt_attrs(robot)}>")
    blocks = reorder_top(robot)
    leg_re = re.compile(r"^(left|right)_(front|rear)_hip_roll$")
    for blk in blocks:
        out.append("")
        if blk.tag == "joint" and leg_re.match(blk.get("name")):
            leg = blk.get("name").replace("_hip_roll", "")
            out.append(f"  <!-- ========== {leg}_leg ========== -->")
        serialize(blk, 1, out)
    out.append("")
    out.append("</robot>")
    return "\n".join(out) + "\n"


def process(path: str) -> bool:
    parser = etree.XMLParser(remove_comments=True)
    orig = etree.parse(path, parser).getroot()
    text = format_text(orig)
    new = etree.fromstring(text.encode())
    errors, stats = compare_roots(orig, new)
    if errors:
        print(f"FAIL {path}（等价校验未通过，未写回）")
        for e in errors:
            print("  -", e)
        return False
    with open(path, "w") as f:
        f.write(text)
    print(f"OK   {path}  ({stats['links']} links, {stats['joints']} joints, 已写回)")
    return True


def main():
    args = sys.argv[1:]
    if not args:
        base = os.path.dirname(os.path.abspath(__file__))
        args = [os.path.normpath(os.path.join(base, p)) for p in KNOWN]
    failed = [p for p in args if not process(p)]
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
