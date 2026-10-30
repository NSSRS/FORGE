from setuptools import find_packages, setup

setup(
    name='bringup', version="0.1.0", packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/bringup']), ('share/bringup', ['package.xml']), ('share/bringup/config', ['config/can2_bench.yaml', 'config/chassis.yaml', 'config/chassis_bench.yaml', 'config/radio_pocket.yaml', 'config/fastdds-udp-loopback.xml', 'config/fastdds-camera-shm.xml', 'config/radio.yaml', 'config/commissioning.yaml', 'config/stereo_cam.yaml', 'config/seam_edges.yaml']), ('share/bringup/launch', ['launch/arm.launch.py', 'launch/chassis.launch.py', 'launch/stereo_cam.launch.py', 'launch/seam_edges.launch.py', 'launch/system_test.launch.py'])],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="FORGE developers", maintainer_email="forge@example.invalid",
    description="Shared robot launch files, configuration profiles, and commissioning sessions", license="Proprietary",
    entry_points={"console_scripts": ["check_system = bringup.check_system:main"]},
)
