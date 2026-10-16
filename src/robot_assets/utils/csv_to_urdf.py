#!/usr/bin/env python3
"""CSV → URDF：从关节/连杆 CSV 重建 URDF（read_urdf.py 的反向工具）。

兼容新旧两种 CSV schema：
- 新 schema：origin_xyz_x/y/z 三列
- 旧 schema：origin_xyz 单列（含尾部空列、数值前后空格、科学计数法）

重建内容：连杆 <inertial>（质心/质量/惯性全矩阵）与关节（origin/parent/child/axis/
limit/dynamics）。CSV 不含 visual/collision/mujoco 信息，重建 URDF 无这些段。
顶层按 连杆→关节→下一连杆 的 DFS 顺序排列（URDF_SPEC §5）。

用法：
    python3 csv_to_urdf.py <joints.csv> <links.csv> -o <输出.urdf> [--robot-name NAME]
"""
import argparse
import csv
import os


def fnum(s):
    """CSV 单元格 → float（strip 空格、科学计数法容错，空/非法 → 0.0）。"""
    s = (s or "").strip()
    if not s:
        return 0.0
    try:
        return float(s)
    except ValueError:
        # 分号分隔的多值单元格（如多 mesh）取第一个分量
        return float(s.split(";")[0].strip())


def fmt(v):
    """float → 规整化字符串（-0 归一为 0，去掉无意义 .0）。"""
    if v == 0:
        return "0"
    s = repr(v)
    if s.endswith(".0"):
        s = s[:-2]
    return s


def vec3(row, key, prefix):
    """读三分量：新 schema 用 prefix_x/y/z 三列，旧 schema 用 key 单列。"""
    new_cols = [f"{prefix}_{s}" for s in ("x", "y", "z")]
    if all(c in row for c in new_cols):
        return [fnum(row[c]) for c in new_cols]
    parts = (row.get(key) or "0 0 0").split()
    out = [fnum(p) for p in parts[:3]]
    while len(out) < 3:
        out.append(0.0)
    return out


def load(path):
    with open(path, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def build_urdf(joints_csv, links_csv, robot_name):
    joints = load(joints_csv)
    links = load(links_csv)
    link_names = [r.get("name", "").strip() for r in links]

    def link_block(row):
        c = vec3(row, "com_xyz", "com_xyz")
        cr = vec3(row, "com_rpy", "com_rpy")
        ixx = fnum(row.get("inertia_ixx"))
        ixy = fnum(row.get("inertia_ixy"))
        ixz = fnum(row.get("inertia_ixz"))
        iyy = fnum(row.get("inertia_iyy"))
        iyz = fnum(row.get("inertia_iyz"))
        izz = fnum(row.get("inertia_izz"))
        lines = [
            f'  <link name="{row.get("name", "").strip()}">',
            "    <inertial>",
            f'      <origin xyz="{fmt(c[0])} {fmt(c[1])} {fmt(c[2])}" '
            f'rpy="{fmt(cr[0])} {fmt(cr[1])} {fmt(cr[2])}"/>',
            f'      <mass value="{fmt(fnum(row.get("mass")))}"/>',
            f'      <inertia ixx="{fmt(ixx)}" ixy="{fmt(ixy)}" ixz="{fmt(ixz)}" '
            f'iyy="{fmt(iyy)}" iyz="{fmt(iyz)}" izz="{fmt(izz)}"/>',
            "    </inertial>",
            "  </link>",
        ]
        return "\n".join(lines)

    def joint_block(row):
        xyz = vec3(row, "origin_xyz", "origin_xyz")
        rpy = vec3(row, "origin_rpy", "origin_rpy")
        ax = vec3(row, "axis", "axis")
        lines = [
            f'  <joint name="{row.get("name", "").strip()}" '
            f'type="{row.get("type", "revolute").strip() or "revolute"}">',
            f'    <origin xyz="{fmt(xyz[0])} {fmt(xyz[1])} {fmt(xyz[2])}" '
            f'rpy="{fmt(rpy[0])} {fmt(rpy[1])} {fmt(rpy[2])}"/>',
            f'    <parent link="{row.get("parent_link", "").strip()}"/>',
            f'    <child link="{row.get("child_link", "").strip()}"/>',
            f'    <axis xyz="{fmt(ax[0])} {fmt(ax[1])} {fmt(ax[2])}"/>',
            f'    <limit lower="{fmt(fnum(row.get("lower_limit")))}" '
            f'upper="{fmt(fnum(row.get("upper_limit")))}" '
            f'effort="{fmt(fnum(row.get("effort_limit")))}" '
            f'velocity="{fmt(fnum(row.get("velocity_limit")))}"/>',
        ]
        damping = fnum(row.get("damping"))
        friction = fnum(row.get("friction"))
        if damping or friction:
            lines.append(f'    <dynamics damping="{fmt(damping)}" friction="{fmt(friction)}"/>')
        lines.append("  </joint>")
        return "\n".join(lines)

    # DFS 重排：连杆 → 关节 → 下一连杆
    link_by_name = {r.get("name", "").strip(): r for r in links}
    parent_joints = {}
    for j in joints:
        parent_joints.setdefault(j.get("parent_link", "").strip(), []).append(j)
    child_of = {j.get("name", "").strip(): j.get("child_link", "").strip() for j in joints}
    roots = [n for n in link_names if n not in set(child_of.values())]

    out = ['<?xml version="1.0" encoding="utf-8"?>',
           '<!-- 由 csv_to_urdf.py 从 CSV 重建；CSV 不含 visual/collision/mujoco 信息，',
           '     如需网格与 MuJoCo 扩展请按 URDF_SPEC.md 补充。 -->',
           f'<robot name="{robot_name}">']
    visited = set()
    emitted_joints = set()

    def dfs(name):
        if name in visited or name not in link_by_name:
            return
        visited.add(name)
        out.append(link_block(link_by_name[name]))
        for j in parent_joints.get(name, []):
            out.append(joint_block(j))
            emitted_joints.add(j.get("name", "").strip())
            dfs(child_of.get(j.get("name", "").strip(), ""))

    for r in roots:
        dfs(r)
    for n in link_names:          # 兜底：未被引用的连杆
        dfs(n)
    for j in joints:              # 兜底：未被引用的关节
        if j.get("name", "").strip() not in emitted_joints:
            out.append(joint_block(j))
    out.append("</robot>")
    return "\n".join(out) + "\n"


def main():
    ap = argparse.ArgumentParser(description="从关节/连杆 CSV 重建 URDF")
    ap.add_argument("joints_csv", help="关节 CSV 文件")
    ap.add_argument("links_csv", help="连杆 CSV 文件")
    ap.add_argument("-o", "--output", required=True, help="输出 URDF 路径")
    ap.add_argument("--robot-name", default=None, help="<robot name>（缺省为 links CSV 文件名主体）")
    args = ap.parse_args()

    for p in (args.joints_csv, args.links_csv):
        if not os.path.exists(p):
            raise SystemExit(f"错误: 文件 '{p}' 不存在")
    robot_name = args.robot_name or os.path.splitext(os.path.basename(args.links_csv))[0].replace("_links", "")

    text = build_urdf(args.joints_csv, args.links_csv, robot_name)
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"已生成: {args.output}")


if __name__ == "__main__":
    main()
