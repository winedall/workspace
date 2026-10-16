from setuptools import setup

package_name = 'robot_api'

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
    description='Python external API package for the robot control system.',
    license='TODO',
    entry_points={
        'console_scripts': [],
    },
)
