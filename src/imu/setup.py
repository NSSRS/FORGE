from setuptools import find_packages, setup

setup(
    name='imu', version="0.1.0", packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/imu']), ('share/imu', ['package.xml'])],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="FORGE developers", maintainer_email="forge@example.invalid",
    description="HWT9052 Modbus serial acquisition, ROS sensor topics, and terminal reader", license="Proprietary",
    entry_points={"console_scripts": ['imu = imu.node:main', 'read_imu = imu.cli:main']},
)
