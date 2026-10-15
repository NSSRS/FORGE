from setuptools import find_packages, setup

setup(
    name='bringup', version="0.1.0", packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/bringup']), ('share/bringup', ['package.xml']), ('share/bringup/config', ['config/can2_bench.yaml', 'config/chassis.yaml', 'config/chassis_bench.yaml', 'config/radio_pocket.yaml', 'config/fastdds-udp-loopback.xml', 'config/radio.yaml', 'config/commissioning.yaml']), ('share/bringup/launch', ['launch/chassis.launch.py'])],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="FORGE developers", maintainer_email="forge@example.invalid",
    description="Shared robot launch files, configuration profiles, and commissioning sessions", license="Proprietary",
    entry_points={"console_scripts": []},
)
