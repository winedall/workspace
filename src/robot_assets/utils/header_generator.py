#!/usr/bin/env python3
import xml.etree.ElementTree as ET
import argparse
import os
import re


def _load_expanded(xml_file):
    """加载 XML 并展开 <include file="..."/>（同目录递归，如 D040_poc.xml）。"""
    base = os.path.dirname(os.path.abspath(xml_file))
    text = open(xml_file, encoding="utf-8").read()

    def expand(t, bdir):
        out, pos = [], 0
        for m in re.finditer(r'<include\s+file="([^"]+)"\s*/>', t):
            out.append(t[pos:m.start()])
            sub = open(os.path.join(bdir, m.group(1)), encoding="utf-8").read()
            sub = re.sub(r'^\s*<\?xml[^>]*\?>', '', sub)
            sub = re.sub(r'<mujoco\b[^>]*>', '', sub, count=1)
            sub = re.sub(r'</mujoco>\s*$', '', sub)
            out.append(expand(sub, bdir))
            pos = m.end()
        out.append(t[pos:])
        return "".join(out)

    return ET.fromstring(expand(text, base))


class MuJoCoConfigGenerator:
    def __init__(self, xml_file):
        self.tree = ET.ElementTree(_load_expanded(xml_file))
        self.root = self.tree.getroot()
        
    def generate_config(self):
        config = {
            'actuators': self._extract_actuators(),
            'joints_from_actuators': self._extract_joints_from_actuators(),
            'all_joints': self._extract_all_joints(),
            'base_body': self._find_base_body(),
            'sensors': self._extract_sensors(),
            'contact_pairs': self._extract_contact_pairs(),
            'actuators_torque_limits': self._extract_actuators_torque_limits(),
            'cameras': self._extract_cameras() 
        }
        return config
    
    def _extract_actuators(self):
        """提取所有actuator名称，保持原始顺序"""
        actuators = []
        actuator_elem = self.root.find('actuator')
        if actuator_elem is not None:
            for motor in actuator_elem.findall('motor'):
                motor_name = motor.get('name', '')
                if motor_name:  
                    actuators.append(motor_name)
        return actuators
    
    def _extract_joints_from_actuators(self):
        """根据actuator名称获取对应的关节名称"""
        joints = []
        actuator_elem = self.root.find('actuator')
        if actuator_elem is not None:
            for motor in actuator_elem.findall('motor'):
                joint_name = motor.get('joint', '')
                if joint_name:  
                    joints.append(joint_name)
        return joints
    
    def _extract_all_joints(self):
        """提取所有关节名称（备用）"""
        joints = []
        for joint in self.root.findall('.//joint'):
            joint_name = joint.get('name')
            if joint_name and joint.get('type') != 'free':  # 排除自由关节
                joints.append(joint_name)
        return joints
    
    def _find_base_body(self):
        """查找基座体，支持两种浮动基座表示方式"""
        for body in self.root.findall('.//body'):
            # 方式1: 检查是否有 type="free" 的 joint 元素
            for joint in body.findall('joint'):
                if joint.get('type') == 'free':
                    return body.get('name', 'Trunk')
            
            # 方式2: 检查是否有 freejoint 元素
            freejoint = body.find('freejoint')
            if freejoint is not None:
                return body.get('name', 'Trunk')
        
        return "Trunk"
    
    def _extract_sensors(self):
        """提取传感器配置"""
        sensors = {}
        sensor_elem = self.root.find('sensor')
        if sensor_elem is not None:
            for sensor in sensor_elem:
                sensor_type = sensor.tag
                sensor_name = sensor.get('name', '')
                sensors[sensor_type] = sensor_name
        return sensors
    
    def _extract_contact_pairs(self):
        """提取接触对"""
        pairs = []
        contact_elem = self.root.find('contact')
        if contact_elem is not None:
            for pair in contact_elem.findall('pair'):
                geom1 = pair.get('geom1')
                geom2 = pair.get('geom2')
                if geom1 and geom2:
                    pairs.append((geom1, geom2))
        return pairs
    
    def _extract_actuators_torque_limits(self):
        """从 <motor ctrlrange> 提取力矩限制，按 XML 顺序返回 (min_list, max_list)"""
        min_torques = []
        max_torques = []
        actuator_elem = self.root.find('actuator')
        if actuator_elem is not None:
            for motor in actuator_elem.findall('motor'):
                ctrlrange = motor.get('ctrlrange')
                if ctrlrange:
                    parts = ctrlrange.split()
                    if len(parts) == 2:
                        min_torques.append(float(parts[0]))
                        max_torques.append(float(parts[1]))
                        continue
                # 未定义 ctrlrange 时使用默认值
                min_torques.append(-1.0)
                max_torques.append(1.0)
        return min_torques, max_torques
    
    def _extract_cameras(self):
        """提取所有相机名称和分辨率，按在 XML 中出现的顺序返回 [(name, width, height)]"""
        cameras = []
        for cam in self.root.findall('.//camera'):
            cam_name = cam.get('name')
            if cam_name:
                width, height = 0, 0
                resolution = cam.get('resolution', '')
                parts = resolution.split()
                if len(parts) == 2:
                    try:
                        width = int(parts[0])
                        height = int(parts[1])
                    except ValueError:
                        width, height = 0, 0
                cameras.append((cam_name, width, height))
        return cameras

def generate_header(config, output_file, robot_name):
    actuators = config['actuators']
    joints = config['joints_from_actuators']
    torque_min = config['actuators_torque_limits'][0]
    torque_max = config['actuators_torque_limits'][1]
    
    # 验证actuator和joints数量匹配
    if len(actuators) != len(joints):
        print(f"警告: actuator数量({len(actuators)})与关节数量({len(joints)})不匹配!")
    
    # 创建actuator到joint的映射
    actuator_joint_map = list(zip(actuators, joints))
    
    # 构建头文件内容，避免在f-string中使用反斜杠
    header_parts = []
    header_parts.append('#pragma once')
    header_parts.append('')
    header_parts.append('#include <chrono>')
    header_parts.append('#include <iostream>')
    header_parts.append('#include <string>')
    header_parts.append('#include <vector>')
    header_parts.append('#include <array>')
    header_parts.append('')
    header_parts.append('#include "Eigen/Core"')
    header_parts.append('')
    header_parts.append(f'namespace {robot_name} {{')
    header_parts.append('')
    header_parts.append('/**')
    header_parts.append(f' * @brief Auto-generated configuration for {robot_name}')
    header_parts.append(' * @note Generated from MuJoCo XML file')
    header_parts.append(' */')
    header_parts.append('')
    header_parts.append('// Actuator and Joint configuration')
    header_parts.append('// Actuator names in the order they appear in XML')
    header_parts.append(f'const std::array<std::string, {len(joints)}> kActuatorsName' ' = {')
    header_parts.append(',\n'.join(f'    "{actuator}"' for actuator in actuators))
    header_parts.append('};')
    header_parts.append('')
    header_parts.append('// Corresponding joint names in the same order as actuators')
    header_parts.append(f'const std::array<std::string, {len(joints)}> kJointsName' ' = {')
    header_parts.append(',\n'.join(f'    "{joint}"' for joint in joints))
    header_parts.append('};')
    header_parts.append('')
    header_parts.append('// Joint torque limits (Nm)')
    header_parts.append(f'const std::array<double, {len(joints)}> kJointsMaxTorque' ' = {')
    header_parts.append(',\n'.join(f'    {t:.6g}' for t in torque_max))
    header_parts.append('};')
    header_parts.append(f'const std::array<double, {len(joints)}> kJointsMinTorque' ' = {')
    header_parts.append(',\n'.join(f'    {t:.6g}' for t in torque_min))
    header_parts.append('};')
    header_parts.append(f'const uint8_t kJointsNum = {len(joints)};')
    header_parts.append(f'const std::string kBaseName = "{config["base_body"]}";')
    header_parts.append('')
    
    # 传感器配置
    sensors = config['sensors']
    header_parts.append('// Sensor configuration')
    header_parts.append(f'const std::string kOrientationSensorName = "{sensors.get("framequat", "")}";')
    header_parts.append(f'const std::string kVelSensorName = "{sensors.get("velocimeter", "")}";')
    header_parts.append(f'const std::string kGyroSensorName = "{sensors.get("gyro", "")}";')
    header_parts.append(f'const std::string kAccSensorName = "{sensors.get("accelerometer", "")}";')
    header_parts.append('')

    # 相机配置 — 分辨率直接取自 XML 中 camera 的 resolution 属性
    cameras = config['cameras']
    cam_count = len(cameras)
    cam_names = [c[0] for c in cameras]
    cam_widths = [c[1] for c in cameras]
    cam_heights = [c[2] for c in cameras]
    header_parts.append('// Camera configuration')
    header_parts.append(f'const uint8_t kCameraNum = {cam_count};')
    if cam_count > 0:
        header_parts.append(f'const std::array<std::string, {cam_count}> kCameraNames = {{')
        header_parts.append(',\n'.join(f'    "{c}"' for c in cam_names))
        header_parts.append('};')
        header_parts.append(f'const std::array<int, {cam_count}> kCameraWidths = {{')
        header_parts.append(',\n'.join(f'    {w}' for w in cam_widths))
        header_parts.append('};')
        header_parts.append(f'const std::array<int, {cam_count}> kCameraHeights = {{')
        header_parts.append(',\n'.join(f'    {h}' for h in cam_heights))
        header_parts.append('};')
    else:
        header_parts.append('const std::array<std::string, 0> kCameraNames = {};')
        header_parts.append('const std::array<int, 0> kCameraWidths = {};')
        header_parts.append('const std::array<int, 0> kCameraHeights = {};')
    header_parts.append('')
    
    header_parts.append(f'using SystemTau = Eigen::Vector<double, {len(joints) + 6}>;')
    header_parts.append(f'using SystemDdq = Eigen::Vector<double, {len(joints) + 6}>;')
    header_parts.append(f'using SystemDq = Eigen::Vector<double, {len(joints) + 6}>;')
    header_parts.append(f'using SystemQ = Eigen::Vector<double, {len(joints) + 7}>;')
    header_parts.append(f'using JointTau = Eigen::Vector<double, {len(joints)}>;')
    header_parts.append(f'using JointDdq = Eigen::Vector<double, {len(joints)}>;')
    header_parts.append(f'using JointDq = Eigen::Vector<double, {len(joints)}>;')
    header_parts.append(f'using JointTau = Eigen::Vector<double, {len(joints)}>;')
    
    header_parts.append(f'}} // namespace {robot_name}')
    
    # 写入文件
    with open(output_file, 'w') as f:
        f.write('\n'.join(header_parts))

def main():
    parser = argparse.ArgumentParser(description='Generate C++ header from MuJoCo XML')
    parser.add_argument('xml_file', help='Input MuJoCo XML file')
    parser.add_argument('-o', '--output', help='Output header file name')
    parser.add_argument('--namespace', default='Robot', help='Namespace for the configuration')
    
    args = parser.parse_args()
    
    generator = MuJoCoConfigGenerator(args.xml_file)
    config = generator.generate_config()
    
    output_file = args.output or f"{args.namespace.lower()}_config.h"
    generate_header(config, output_file, args.namespace)
    
    print(f"Successfully generated configuration:")
    print(f"  - Actuators: {len(config['actuators'])}")
    print(f"  - Joints: {len(config['joints_from_actuators'])}")
    print(f"  - Base body: {config['base_body']}")
    
    # 打印actuator和joint的对应关系
    print("\nActuator to Joint mapping:")
    for actuator, joint in zip(config['actuators'], config['joints_from_actuators']):
        print(f"  {actuator} -> {joint}")

if __name__ == "__main__":
    main()