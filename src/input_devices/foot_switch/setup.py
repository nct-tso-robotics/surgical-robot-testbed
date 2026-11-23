from setuptools import find_packages, setup

package_name = "foot_switch"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/foot_switch/launch", ["launch/foot_switch.launch.py"]),
    ],
    install_requires=["setuptools", "testbed-input-core>=0.1.0"],
    zip_safe=True,
    maintainer="Lorenzo Mazza, Ariel Rodriguez",
    maintainer_email="lorenzo.mazza@nct-dresden.de, ariel.rodriguezjimenez@nct-dresden.de",
    author="Ghana Prashanth Ramachandran",
    description="Listener for foot switch input",
    license="MIT",
    entry_points={
        "console_scripts": [
            "foot_switch = foot_switch.foot_switch_state_publisher:main",
        ],
    },
)
