from setuptools import setup

setup(
    name="testbed_transforms",
    version="0.1.0",
    packages=["testbed_transforms"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/testbed_transforms"]),
        ("share/testbed_transforms", ["package.xml"]),
        ("share/testbed_transforms/config", ["config/endoscope_mount.example.yaml"]),
    ],
    install_requires=["setuptools", "tso-sensorium"],
    maintainer="Lorenzo Mazza, Ariel Rodriguez",
    maintainer_email="lorenzo.mazza@nct-dresden.de, ariel.rodriguezjimenez@nct-dresden.de",
    license="MIT",
    entry_points={
        "console_scripts": [
            "camera_transform = testbed_transforms.camera_transform:main",
            "camera_mount = testbed_transforms.camera_mount:main",
        ],
    },
)
