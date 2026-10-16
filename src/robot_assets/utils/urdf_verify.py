#!/usr/bin/env python3
"""校验两个 URDF 是否语义等价：
- 连杆/关节名称集合一致，parent/child 映射一致，关节出现顺序一致
- 每个连杆/关节的子元素树结构一致（同 tag、同属性；数值按 float 精确比较，注释除外）

用法：
    python3 urdf_verify.py <orig.urdf> <new.urdf> [<orig2> <new2> ...]
"""
import sys
from lxml import etree

NUMERIC_ATTRS = {
    "xyz", "rpy", "rgba", "size", "radius", "length", "scale", "value",
    "lower", "upper", "effort", "velocity", "damping", "friction",
    "ixx", "ixy", "ixz", "iyy", "iyz", "izz",
}


def canonical(elem):
    """元素的语义签名：tag + 规范化属性 + 子元素多重集（顺序无关）。"""
    attrs = []
    for k, v in elem.attrib.items():
        if k in NUMERIC_ATTRS:
            attrs.append((k, tuple(float(t) for t in v.split())))
        else:
            attrs.append((k, v))
    attrs.sort()
    children = sorted(canonical(c) for c in elem)
    return (elem.tag, tuple(attrs), tuple(children))


def compare_roots(a, b):
    """比较两个已解析的 <robot> 根元素，返回 (errors, stats)。"""
    errors = []
    if a.tag != "robot" or b.tag != "robot":
        return [f"根元素不是 <robot>: {a.tag} vs {b.tag}"], {}

    if a.get("name") != b.get("name"):
        errors.append(f"robot name 不一致: {a.get('name')} vs {b.get('name')}")
    ma = [c for c in a if c.tag == "mujoco"]
    mb = [c for c in b if c.tag == "mujoco"]
    if [canonical(m) for m in ma] != [canonical(m) for m in mb]:
        errors.append("mujoco 扩展元素不一致")
    mata = {m.get("name"): canonical(m) for m in a if m.tag == "material"}
    matb = {m.get("name"): canonical(m) for m in b if m.tag == "material"}
    if mata != matb:
        errors.append(f"material 定义不一致: {mata} vs {matb}")

    links_a = {l.get("name"): l for l in a if l.tag == "link"}
    links_b = {l.get("name"): l for l in b if l.tag == "link"}
    joints_a = [j for j in a if j.tag == "joint"]
    joints_b = [j for j in b if j.tag == "joint"]

    if set(links_a) != set(links_b):
        errors.append(f"连杆集合不一致: {set(links_a) ^ set(links_b)}")
    names_a = [j.get("name") for j in joints_a]
    names_b = [j.get("name") for j in joints_b]
    if set(names_a) != set(names_b):
        errors.append(f"关节集合不一致: {set(names_a) ^ set(names_b)}")
    if names_a != names_b:
        errors.append("关节顺序发生变化!（可能影响按出现顺序建立索引的解析器）")
    pa = {j.get("name"): (j.find("parent").get("link"), j.find("child").get("link"))
          for j in joints_a}
    pb = {j.get("name"): (j.find("parent").get("link"), j.find("child").get("link"))
          for j in joints_b}
    for n in pa:
        if pa[n] != pb.get(n):
            errors.append(f"关节 {n} parent/child 不一致: {pa[n]} vs {pb.get(n)}")

    for name, l in links_a.items():
        if canonical(l) != canonical(links_b[name]):
            errors.append(f"连杆 {name} 内容不一致")
    ja = {j.get("name"): j for j in joints_a}
    jb = {j.get("name"): j for j in joints_b}
    for n, j in ja.items():
        if canonical(j) != canonical(jb[n]):
            errors.append(f"关节 {n} 内容不一致")

    return errors, {"links": len(links_a), "joints": len(joints_a)}


def main():
    if len(sys.argv) < 3 or len(sys.argv) % 2 != 1:
        print(__doc__)
        sys.exit(2)
    parser = etree.XMLParser(remove_comments=True)
    ok = True
    for orig, new in zip(sys.argv[1::2], sys.argv[2::2]):
        a = etree.parse(orig, parser).getroot()
        b = etree.parse(new, parser).getroot()
        errors, stats = compare_roots(a, b)
        if errors:
            ok = False
            print(f"FAIL {orig}")
            for e in errors:
                print("  -", e)
        else:
            print(f"OK   {orig}  ({stats['links']} links, {stats['joints']} joints)")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
