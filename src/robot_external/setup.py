from setuptools import setup

package_name = 'robot_external'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='maintainer@example.com',
    description='External control interface for the robot control system.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'robot_external_node = robot_external.external_node:main',
            'head_pose_node = robot_external.head_pose_node:main',
            # 数据记录（robot_external.recorder 子包）：独立调试工具，
            # 不被 head_pose 逻辑依赖，也不影响 /head_pose 发布。
            'head_pose_record_node = robot_external.recorder.record_node:main',
        ],
    },
)
