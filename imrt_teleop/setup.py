from setuptools import setup
from glob import glob
import os

package_name = "imrt_teleop"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Fetullah Atas",
    maintainer_email="fetulahatas1@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    # Actual ROS nodes
    entry_points={
        "console_scripts": [
            "imrt_teleop_node = imrt_teleop.imrt_teleop_node:main",
        ],
    },
)
