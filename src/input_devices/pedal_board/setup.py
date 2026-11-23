from setuptools import find_packages, setup

package_name = "pedal_board"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/pedal_board/launch", ["launch/pedal_board.launch.py"]),
    ],
    install_requires=["setuptools", "testbed-input-core>=0.1.0"],
    zip_safe=True,
    maintainer="Lorenzo Mazza, Ariel Rodriguez",
    maintainer_email="lorenzo.mazza@nct-dresden.de, ariel.rodriguezjimenez@nct-dresden.de",
    author="Ghana Prashanth Ramachandran",
    description="Listener for pedal board input",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pedal_board = pedal_board.pedal_board_state_publisher:main",
        ],
    },
)
