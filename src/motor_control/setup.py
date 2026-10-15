from setuptools import find_packages, setup

setup(
    name='motor_control', version="0.1.0", packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/motor_control']), ('share/motor_control', ['package.xml'])],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="FORGE developers", maintainer_email="forge@example.invalid",
    description="ROS motor driver, differential chassis mixing, and bounded commissioning", license="Proprietary",
    entry_points={"console_scripts": ['motor_driver = motor_control.motor_driver:main']},
)
