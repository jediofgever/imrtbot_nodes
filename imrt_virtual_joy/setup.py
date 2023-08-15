from setuptools import setup
from glob import glob
import os
from setuptools import setup, find_packages

package_name = "imrt_virtual_joy"
submodules = "imrt_virtual_joy/submodule"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name, submodules],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*")),
        (os.path.join("share", package_name), glob("params/*")),
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
            "virtual_gamepad = imrt_virtual_joy.virtual_gamepad:main",
        ],
    },
)
