from setuptools import find_packages, setup


PACKAGE_NAME = "forge_motor_control"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        ("share/" + PACKAGE_NAME + "/config", ["config/commissioning.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="FORGE developers",
    maintainer_email="forge@example.invalid",
    description="ROS 2 commissioning nodes for the FORGE ENCOS motor interface.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "forge_motor_driver = forge_motor_control.motor_driver:main",
            "forge_keyboard_teleop = forge_motor_control.keyboard_teleop:main",
        ],
    },
)
