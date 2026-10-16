import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation as R

def parse_rpy(rpy_str):
    """解析RPY字符串为NumPy数组"""
    if rpy_str is None:
        return np.zeros(3)
    return np.array([float(x) for x in rpy_str.split()])

def parse_xyz(xyz_str):
    """解析XYZ字符串为NumPy数组"""
    if xyz_str is None:
        return np.zeros(3)
    return np.array([float(x) for x in xyz_str.split()])

def format_rpy(rpy):
    """格式化RPY数组为字符串"""
    return " ".join([f"{x:.6f}" for x in rpy])

def format_xyz(xyz):
    """格式化XYZ数组为字符串"""
    return " ".join([f"{x:.6f}" for x in xyz])

def rotation_matrix_from_rpy(rpy):
    """从RPY创建旋转矩阵"""
    r, p, y = rpy
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(r), -np.sin(r)],
                   [0, np.sin(r), np.cos(r)]])
    
    Ry = np.array([[np.cos(p), 0, np.sin(p)],
                   [0, 1, 0],
                   [-np.sin(p), 0, np.cos(p)]])
    
    Rz = np.array([[np.cos(y), -np.sin(y), 0],
                   [np.sin(y), np.cos(y), 0],
                   [0, 0, 1]])
    
    return Rz @ Ry @ Rx

def rpy_from_rotation_matrix(rot_matrix):
    """从旋转矩阵获取RPY"""
    theta_z = np.arctan2(rot_matrix[1, 0], rot_matrix[0, 0])
    theta_y = np.arctan2(-1 * rot_matrix[2, 0], np.sqrt(rot_matrix[2, 1] * rot_matrix[2, 1] + rot_matrix[2, 2] * rot_matrix[2, 2]))
    theta_x = np.arctan2(rot_matrix[2, 1], rot_matrix[2, 2])
    return [theta_x, theta_y, theta_z]

def transform_inertia_tensor(inertia, rotation):
    """转换惯性张量"""
    # 惯性张量转换公式: I_new = R * I_old * R^T
    return rotation @ inertia @ rotation.T

def parse_inertia(inertia_elem):
    """解析惯性张量元素"""
    return np.array([
        [float(inertia_elem.get('ixx', 0)), float(inertia_elem.get('ixy', 0)), float(inertia_elem.get('ixz', 0))],
        [float(inertia_elem.get('ixy', 0)), float(inertia_elem.get('iyy', 0)), float(inertia_elem.get('iyz', 0))],
        [float(inertia_elem.get('ixz', 0)), float(inertia_elem.get('iyz', 0)), float(inertia_elem.get('izz', 0))]
    ])

def format_inertia(inertia):
    """格式化惯性张量为XML属性"""
    return {
        'ixx': f"{inertia[0, 0]:.6g}",
        'ixy': f"{inertia[0, 1]:.6g}",
        'ixz': f"{inertia[0, 2]:.6g}",
        'iyy': f"{inertia[1, 1]:.6g}",
        'iyz': f"{inertia[1, 2]:.6g}",
        'izz': f"{inertia[2, 2]:.6g}"
    }

def process_urdf(input_file, output_file):
    """处理URDF文件，将关节RPY归零并保持世界坐标系属性不变"""
    # 解析URDF文件
    tree = ET.parse(input_file)
    root = tree.getroot()
    
    # 创建链接名称到元素的映射
    links = {link.get('name'): link for link in root.findall('link')}
    
    # 创建关节名称到元素的映射
    joints = {joint.get('name'): joint for joint in root.findall('joint')}
    
    # 创建父子关系映射
    parent_map = {}
    children_map = {}  # 新增：创建子链接映射
    for joint in joints.values():
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        parent_map[child] = (parent, joint.get('name'))
        
        # 新增：构建子链接映射
        if parent not in children_map:
            children_map[parent] = []
        children_map[parent].append((child, joint.get('name')))
    
    # 按层次顺序处理关节（从根到叶）
    processed_joints = set()
    
    # 找到根链接（没有父关节的链接）
    all_children = set(parent_map.keys())
    all_parents = set(parent for parent, _ in parent_map.values())
    root_links = all_parents - all_children
    
    # 递归处理链接
    def process_link(link_name):
        print(f"Start process {link_name}")
        
        # 处理当前链接的所有子链接
        if link_name in children_map:
            for child_link, joint_name in children_map[link_name]:
                # 如果已经处理过这个关节，跳过
                if joint_name in processed_joints:
                    print(f"Joint {joint_name} already processed, skip.")
                    # 继续处理子链接
                    process_link(child_link)
                    continue
                    
                # 获取关节元素
                joint = joints[joint_name]
                
                # 获取关节origin
                origin_elem = joint.find('origin')
                if origin_elem is None:
                    origin_elem = ET.SubElement(joint, 'origin')
                    origin_elem.set('xyz', '0 0 0')
                    origin_elem.set('rpy', '0 0 0')
                
                # 获取当前RPY
                current_rpy = parse_rpy(origin_elem.get('rpy'))
                
                # 如果RPY已经是零，跳过
                if np.allclose(current_rpy, 0):
                    processed_joints.add(joint_name)
                    print(f"Joint {joint_name} RPY already 0, skip.")
                    # 继续处理子链接
                    process_link(child_link)
                    continue
                    
                print(f"Processing joint {joint_name} with RPY {current_rpy}")
                
                # 获取轴向量
                axis_elem = joint.find('axis')
                if axis_elem is None:
                    axis_elem = ET.SubElement(joint, 'axis')
                    axis_elem.set('xyz', '0 0 1')
                
                current_axis = parse_xyz(axis_elem.get('xyz'))
                
                # 计算旋转矩阵
                rot_matrix = rotation_matrix_from_rpy(current_rpy)
                
                # 更新轴向量
                new_axis = rot_matrix @ current_axis
                axis_elem.set('xyz', format_xyz(new_axis))
                
                # 将关节RPY置零
                origin_elem.set('rpy', '0 0 0')
                
                # 获取子链接
                child_link_elem = links[child_link]
                
                # 处理惯性属性
                inertial_elem = child_link_elem.find('inertial')
                if inertial_elem is not None:
                    origin_inertial = inertial_elem.find('origin')
                    if origin_inertial is None:
                        origin_inertial = ET.SubElement(inertial_elem, 'origin')
                        origin_inertial.set('xyz', '0 0 0')
                        origin_inertial.set('rpy', '0 0 0')
                    
                    # 转换惯性原点
                    current_xyz = parse_xyz(origin_inertial.get('xyz'))
                    new_xyz = rot_matrix @ current_xyz
                    origin_inertial.set('xyz', format_xyz(new_xyz))
                    
                    # 转换惯性旋转
                    current_rpy_inertial = parse_rpy(origin_inertial.get('rpy'))
                    combined_rot = rot_matrix @ rotation_matrix_from_rpy(current_rpy_inertial)
                    new_rpy_inertial = rpy_from_rotation_matrix(combined_rot)
                    origin_inertial.set('rpy', format_rpy(new_rpy_inertial))
                    
                    # 转换惯性张量
                    inertia_elem = inertial_elem.find('inertia')
                    if inertia_elem is not None:
                        inertia_tensor = parse_inertia(inertia_elem)
                        new_inertia = transform_inertia_tensor(inertia_tensor, rot_matrix)
                        inertia_attrs = format_inertia(new_inertia)
                        for key, value in inertia_attrs.items():
                            inertia_elem.set(key, value)
                
                # 处理视觉属性
                visual_elems = child_link_elem.findall('visual')
                for visual_elem in visual_elems:
                    origin_visual = visual_elem.find('origin')
                    if origin_visual is None:
                        origin_visual = ET.SubElement(visual_elem, 'origin')
                        origin_visual.set('xyz', '0 0 0')
                        origin_visual.set('rpy', '0 0 0')
                    
                    # 转换视觉原点
                    current_xyz = parse_xyz(origin_visual.get('xyz'))
                    new_xyz = rot_matrix @ current_xyz
                    origin_visual.set('xyz', format_xyz(new_xyz))
                    
                    # 转换视觉旋转
                    current_rpy_visual = parse_rpy(origin_visual.get('rpy'))
                    combined_rot = rot_matrix @ rotation_matrix_from_rpy(current_rpy_visual)
                    new_rpy_visual = rpy_from_rotation_matrix(combined_rot)
                    origin_visual.set('rpy', format_rpy(new_rpy_visual))
                
                # 处理碰撞属性
                collision_elems = child_link_elem.findall('collision')
                for collision_elem in collision_elems:
                    origin_collision = collision_elem.find('origin')
                    if origin_collision is None:
                        origin_collision = ET.SubElement(collision_elem, 'origin')
                        origin_collision.set('xyz', '0 0 0')
                        origin_collision.set('rpy', '0 0 0')
                    
                    # 转换碰撞原点
                    current_xyz = parse_xyz(origin_collision.get('xyz'))
                    new_xyz = rot_matrix @ current_xyz
                    origin_collision.set('xyz', format_xyz(new_xyz))
                    
                    # 转换碰撞旋转
                    current_rpy_collision = parse_rpy(origin_collision.get('rpy'))
                    combined_rot = rot_matrix @ rotation_matrix_from_rpy(current_rpy_collision)
                    new_rpy_collision = rpy_from_rotation_matrix(combined_rot)
                    origin_collision.set('rpy', format_rpy(new_rpy_collision))
                
                # 处理后续关节的origin
                if child_link in children_map:
                    for grandchild_link, grandchild_joint_name in children_map[child_link]:
                        print("Handle grandchild_link joint origin.")
                        grandchild_joint = joints[grandchild_joint_name]
                        grandchild_origin = grandchild_joint.find('origin')
                        if grandchild_origin is None:
                            grandchild_origin = ET.SubElement(grandchild_joint, 'origin')
                            grandchild_origin.set('xyz', '0 0 0')
                            grandchild_origin.set('rpy', '0 0 0')
                        
                        # 转换后续关节的origin
                        current_grandchild_xyz = parse_xyz(grandchild_origin.get('xyz'))
                        new_grandchild_xyz = rot_matrix @ current_grandchild_xyz
                        grandchild_origin.set('xyz', format_xyz(new_grandchild_xyz))
                        
                        current_grandchild_rpy = parse_rpy(grandchild_origin.get('rpy'))
                        combined_grandchild_rot = rot_matrix @ rotation_matrix_from_rpy(current_grandchild_rpy)
                        new_grandchild_rpy = rpy_from_rotation_matrix(combined_grandchild_rot)
                        grandchild_origin.set('rpy', format_rpy(new_grandchild_rpy))
                
                # 标记为已处理
                processed_joints.add(joint_name)
                
                # 递归处理子链接
                process_link(child_link)
    
    # 处理所有根链接
    for root_link in root_links:
        process_link(root_link)
    
    # 保存修改后的URDF
    tree.write(output_file, encoding='utf-8', xml_declaration=True)
    print(f"Processed URDF saved to {output_file}")

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Process URDF to zero joint RPY while maintaining world properties')
    parser.add_argument('input', help='Input URDF file')
    parser.add_argument('output', help='Output URDF file')
    
    args = parser.parse_args()
    
    process_urdf(args.input, args.output)