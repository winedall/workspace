from setuptools import find_packages, setup

package_name = "robot_ota"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    package_data={
        "robot_ota": [
            "templates/*.html",
            "agent/*.sh",
            "agent/*.service",
            "agent/*.yaml",
        ],
    },
    install_requires=[
        "setuptools",
        "uvicorn",
        "fastapi",
        "pydantic-settings",
        "jinja2",
        "pyyaml",
        "pynacl",
        "paramiko",
    ],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "robot_ota_node = robot_ota.ota_server_node:main",
        ],
    },
)
