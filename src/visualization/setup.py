from setuptools import find_packages, setup

setup(
    name='visualization', version="0.1.0", packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/visualization']), ('share/visualization', ['package.xml'])],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="FORGE developers", maintainer_email="forge@example.invalid",
    description="Read-only motor state relay and MuJoCo measured-state viewer", license="Proprietary",
    entry_points={"console_scripts": ['motor_state_relay = visualization.motor_state_relay:main', 'imu_viewer = visualization.imu_viewer:main']},
)
