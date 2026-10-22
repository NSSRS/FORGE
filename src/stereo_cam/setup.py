from setuptools import find_packages, setup

setup(
    name='stereo_cam', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/stereo_cam']),
        ('share/stereo_cam', ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='FORGE developers', maintainer_email='forge@example.invalid',
    description='Orbbec Gemini 338Le image streams and localization camera configuration',
    license='Proprietary',
    entry_points={'console_scripts': ['check_camera = stereo_cam.check_camera:main', 'view_camera = stereo_cam.viewer:main']},
)
