from setuptools import setup

package_name = "robot_mqtt"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "paho-mqtt", "PyYAML"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="maintainer@example.com",
    description="MQTT bridge for robot intent and status topics.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "robot_mqtt_node = robot_mqtt.robot_mqtt_node:main",
        ],
    },
)
