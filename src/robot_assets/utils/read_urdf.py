#!/usr/bin/env python3
"""URDF → CSV：导出关节/连杆参数（新 schema，见 description/URDF_SPEC.md）。

规则：
- 向量属性拆分为三个独立单元格（origin_xyz_x/y/z、origin_rpy_r/p/y、axis_x/y/z 等）
- 关节表含 armature 列（URDF 无此属性，留空待手工填写）
- 连杆表含 STL 文件及其位姿（多个 mesh 用 ; 分隔对应分量）
- 数值统一 float 解析后规整化输出

用法：
    python3 read_urdf.py <urdf_file> [-o 输出目录]
输出 <base>_joints.csv 与 <base>_links.csv。
"""
import argparse
import os
import xml.etree.ElementTree as ET

import pandas as pd


def split3(v, default="0 0 0"):
    """'x y z' 字符串 → [x, y, z]（float 规整化，不足补 0）。"""
    parts = (v or default).split()
    out = [str(float(p)) for p in parts[:3]]
    while len(out) < 3:
        out.append("0")
    return out


def join_component(rows, idx):
    """多个 mesh 的同一分量用 ; 分隔聚合（如 stl_xyz_x: '0;0'）。"""
    return ";".join(split3(r)[idx] for r in rows)


def parse_link(link_element):
    """解析连杆：惯性参数 + STL 文件与位姿。"""
    link_name = link_element.get("name", "")

    mass = 0.0
    inertia = {"ixx": 0.0, "ixy": 0.0, "ixz": 0.0,
               "iyy": 0.0, "iyz": 0.0, "izz": 0.0}
    com_xyz = "0 0 0"
    com_rpy = "0 0 0"

    inertial = link_element.find("inertial")
    if inertial is not None:
        origin = inertial.find("origin")
        if origin is not None:
            com_xyz = origin.get("xyz", "0 0 0")
            com_rpy = origin.get("rpy", "0 0 0")
        mass_elem = inertial.find("mass")
        if mass_elem is not None:
            mass = float(mass_elem.get("value", 0.0))
        inertia_elem = inertial.find("inertia")
        if inertia_elem is not None:
            for k in inertia:
                inertia[k] = float(inertia_elem.get(k, 0.0))

    # STL：link 的全部 <visual><geometry><mesh>（无 mesh 的 visual 跳过）
    stl_files, stl_xyz, stl_rpy, stl_scales = [], [], [], []
    for visual in link_element.findall("visual"):
        geometry = visual.find("geometry")
        mesh = geometry.find("mesh") if geometry is not None else None
        if mesh is None:
            continue
        origin = visual.find("origin")
        stl_files.append(os.path.basename(mesh.get("filename", "")))
        stl_xyz.append(origin.get("xyz", "0 0 0") if origin is not None else "0 0 0")
        stl_rpy.append(origin.get("rpy", "0 0 0") if origin is not None else "0 0 0")
        stl_scales.append(mesh.get("scale", "1 1 1"))

    com = split3(com_xyz)
    comr = split3(com_rpy)
    return {
        "name": link_name,
        "mass": mass,
        "com_xyz_x": com[0], "com_xyz_y": com[1], "com_xyz_z": com[2],
        "com_rpy_r": comr[0], "com_rpy_p": comr[1], "com_rpy_y": comr[2],
        "inertia_ixx": inertia["ixx"], "inertia_ixy": inertia["ixy"],
        "inertia_ixz": inertia["ixz"], "inertia_iyy": inertia["iyy"],
        "inertia_iyz": inertia["iyz"], "inertia_izz": inertia["izz"],
        "stl_file": ";".join(stl_files),
        "stl_xyz_x": join_component(stl_xyz, 0) if stl_xyz else "",
        "stl_xyz_y": join_component(stl_xyz, 1) if stl_xyz else "",
        "stl_xyz_z": join_component(stl_xyz, 2) if stl_xyz else "",
        "stl_rpy_r": join_component(stl_rpy, 0) if stl_rpy else "",
        "stl_rpy_p": join_component(stl_rpy, 1) if stl_rpy else "",
        "stl_rpy_y": join_component(stl_rpy, 2) if stl_rpy else "",
        "stl_scale": ";".join(stl_scales),
    }


def parse_joint(joint_element):
    """解析关节：父/子、原点、轴、限位、阻尼（armature 留空）。"""
    joint_name = joint_element.get("name", "")
    joint_type = joint_element.get("type", "revolute")

    parent = joint_element.find("parent")
    child = joint_element.find("child")
    parent_link = parent.get("link", "") if parent is not None else ""
    child_link = child.get("link", "") if child is not None else ""

    origin = joint_element.find("origin")
    origin_xyz = origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
    origin_rpy = origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"

    axis_elem = joint_element.find("axis")
    axis = axis_elem.get("xyz", "1 0 0") if axis_elem is not None else "1 0 0"

    limit_elem = joint_element.find("limit")
    effort = velocity = lower = upper = 0.0
    if limit_elem is not None:
        effort = float(limit_elem.get("effort", 0.0))
        velocity = float(limit_elem.get("velocity", 0.0))
        lower = float(limit_elem.get("lower", 0.0))
        upper = float(limit_elem.get("upper", 0.0))

    dynamics_elem = joint_element.find("dynamics")
    damping = friction = 0.0
    if dynamics_elem is not None:
        damping = float(dynamics_elem.get("damping", 0.0))
        friction = float(dynamics_elem.get("friction", 0.0))

    xyz = split3(origin_xyz)
    rpy = split3(origin_rpy)
    ax = split3(axis, "1 0 0")
    return {
        "name": joint_name,
        "type": joint_type,
        "parent_link": parent_link,
        "child_link": child_link,
        "origin_xyz_x": xyz[0], "origin_xyz_y": xyz[1], "origin_xyz_z": xyz[2],
        "origin_rpy_r": rpy[0], "origin_rpy_p": rpy[1], "origin_rpy_y": rpy[2],
        "axis_x": ax[0], "axis_y": ax[1], "axis_z": ax[2],
        "effort_limit": effort,
        "velocity_limit": velocity,
        "lower_limit": lower,
        "upper_limit": upper,
        "armature": "",       # URDF 无此属性，留空待手工填写
        "damping": damping,
        "friction": friction,
    }


def parse_urdf_to_csv(urdf_file, output_dir="."):
    tree = ET.parse(urdf_file)
    root = tree.getroot()

    links_data = [parse_link(l) for l in root.findall("link")]
    joints_data = [parse_joint(j) for j in root.findall("joint")]

    base_name = os.path.splitext(os.path.basename(urdf_file))[0]
    joints_file = os.path.join(output_dir, f"{base_name}_joints.csv")
    links_file = os.path.join(output_dir, f"{base_name}_links.csv")

    pd.DataFrame(joints_data).to_csv(joints_file, index=False, encoding="utf-8")
    pd.DataFrame(links_data).to_csv(links_file, index=False, encoding="utf-8")

    print(f"关节参数已保存到: {joints_file}")
    print(f"连杆参数已保存到: {links_file}")
    print(f"共解析 {len(joints_data)} 个关节和 {len(links_data)} 个连杆")


def main():
    parser = argparse.ArgumentParser(description="将URDF文件转换为关节和连杆的CSV表格")
    parser.add_argument("urdf_file", help="URDF文件路径")
    parser.add_argument("-o", "--output", default=".", help="输出目录 (默认: 当前目录)")
    args = parser.parse_args()

    if not os.path.exists(args.urdf_file):
        print(f"错误: 文件 '{args.urdf_file}' 不存在")
        return
    os.makedirs(args.output, exist_ok=True)

    parse_urdf_to_csv(args.urdf_file, args.output)


if __name__ == "__main__":
    main()
