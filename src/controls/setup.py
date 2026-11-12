from setuptools import find_packages, setup

setup(
    name="controls", version="0.1.0", packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/controls"]),
        ("share/controls", ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools", "numpy"], zip_safe=False,
    maintainer="FORGE developers", maintainer_email="forge@example.invalid",
    description="Coordinated motion for the FORGE chassis and arm", license="Proprietary",
    entry_points={"console_scripts": ["preview_weave = controls.simulation.weave_xy_z:main"]},
)
