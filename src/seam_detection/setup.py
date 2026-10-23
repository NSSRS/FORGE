from setuptools import find_packages, setup

setup(
    name='seam_detection', version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/seam_detection']),
                ('share/seam_detection', ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='FORGE developers', maintainer_email='forge@example.invalid',
    description='Observation-only RGB weld joint edge candidates', license='Proprietary',
    entry_points={'console_scripts': ['seam_edges = seam_detection.node:main']},
)
