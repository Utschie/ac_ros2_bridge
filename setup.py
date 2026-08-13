import os
from glob import glob
from setuptools import setup

package_name = "ac_ros2_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ac_sim",
    maintainer_email="todo@todo.local",
    description="AC sim <-> ROS2 relay (Zenoh subscriber -> ROS2 topics).",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "relay = ac_ros2_bridge.relay:main",
        ],
    },
)
