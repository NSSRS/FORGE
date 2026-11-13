from setuptools import find_packages, setup

setup(
    name='teleop', version="0.1.0", packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/teleop']), ('share/teleop', ['package.xml'])],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="FORGE developers", maintainer_email="forge@example.invalid",
    description="Chassis keyboard/radio input and arm desktop joint jogging", license="Proprietary",
    entry_points={"console_scripts": ['keyboard_teleop = teleop.keyboard_teleop:main', 'arm_keyboard = teleop.arm_keyboard:main', 'arm_recording = teleop.arm_recording:main', 'arm_calibration = teleop.arm_calibration:main', 'pocket_teleop = teleop.pocket_teleop:main', 'pocket_gui = teleop.pocket_gui:main', 'calibrate_pocket = teleop.pocket_calibration:main']},
)
