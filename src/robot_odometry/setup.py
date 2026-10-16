from glob import glob
from os.path import isfile

from setuptools import find_packages, setup

package_name = 'robot_odometry'

# model/ 下平铺放置 ONNX 图与说明文件。用 isfile 过滤，避免把目录交给
# setuptools（会报 "doesn't exist or not a regular file"）。
MODEL_FILES = [path for path in glob('model/*') if isfile(path)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 注册表与每模型配置平铺在 config/ 下。
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # ONNX 图平铺在 model/ 下，与注册表里的 model_path 一一对应。
        ('share/' + package_name + '/model', MODEL_FILES),

        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='maintainer@example.com',
    description='基于本体感觉与策略动作的 RL 里程计推理节点。',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_odometry_node = robot_odometry.odometry_node:main',
        ],
    },
)
