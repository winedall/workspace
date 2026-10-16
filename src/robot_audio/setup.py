from glob import glob
import os
from setuptools import find_packages, setup

package_name = "robot_audio"
sherpa_model_files = glob("models/matcha-icefall-zh-baker/**/*", recursive=True)
sherpa_model_files = [path for path in sherpa_model_files if os.path.isfile(path)]

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        *[
            ("share/" + package_name + "/" + "/".join(path.split("/")[:-1]), [path])
            for path in sherpa_model_files
        ],
    ],
    install_requires=["setuptools", "sherpa-onnx", "numpy", "PyYAML"],
    zip_safe=True,
    maintainer="Developer",
    maintainer_email="developer@company.com",
    description="离线文本转语音播报模块",
    license="MIT",
    entry_points={
        "console_scripts": [
            "robot_audio_node = robot_audio.robot_audio_node:main",
        ],
    },
)
