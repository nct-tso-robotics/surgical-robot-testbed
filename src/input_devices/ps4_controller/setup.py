import os
from glob import glob
from setuptools import setup

package_name = "ps4_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "testbed-input-core>=0.1.0"],
    zip_safe=True,
    maintainer="Lorenzo Mazza, Ariel Rodriguez",
    maintainer_email="lorenzo.mazza@nct-dresden.de, ariel.rodriguezjimenez@nct-dresden.de",
    description="ROS 2 publisher for DualShock 4 controller state",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ps4_state_publisher = ps4_controller.ps4_state_publisher:main"
        ],
    },
)
