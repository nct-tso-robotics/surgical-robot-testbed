from glob import glob

from setuptools import find_packages, setup

package_name = "input_translation"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/web", glob("web/*.js")),
    ],
    install_requires=["setuptools", "testbed-input-core>=0.1.0"],
    zip_safe=True,
    maintainer="Lorenzo Mazza, Ariel Rodriguez",
    maintainer_email="lorenzo.mazza@nct-dresden.de, ariel.rodriguezjimenez@nct-dresden.de",
    description="ROS 2 input and policy adapters for testbed motion deltas",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ps4_robot_pose_setter = input_translation.ps4_robot_pose_setter:main",
            "lambda_robot_pose_setter = input_translation.lambda_robot_pose_setter:main",
            "model_robot_pose_setter = input_translation.model_robot_pose_setter:main",
            "language_publisher = input_translation.language_publisher:main",
            "gripper_server = input_translation.gripper_server:main",
        ]
    },
)
