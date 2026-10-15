from setuptools import find_packages, setup

setup(
    name='teleop', version="0.1.0", packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/teleop']), ('share/teleop', ['package.xml'])],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="FORGE developers", maintainer_email="forge@example.invalid",
    description="Keyboard and serial radio chassis command input", license="Proprietary",
    entry_points={"console_scripts": ['keyboard_teleop = teleop.keyboard_teleop:main']},
)
