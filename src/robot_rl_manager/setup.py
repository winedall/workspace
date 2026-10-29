from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_rl_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/model', glob('model/*')),
    ],
    package_data={
        package_name: ['convert/*.yaml', 'replay/*.yaml'],
    },
    install_requires=['setuptools', 'PyYAML', 'numpy', 'cv_bridge'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='maintainer@example.com',
    description='Python RL model management package for the robot control system.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'robot_rl_manager_node = robot_rl_manager.rl_manager_node:main',
            'robot_rl_manager_rknn_convert = robot_rl_manager.convert.cli:main',
            'robot_rl_manager_rknn_replay = robot_rl_manager.replay.cli:main',
        ],
    },
)
