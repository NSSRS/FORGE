from glob import glob
from setuptools import setup
setup(name='robot_description', version='0.1.0', packages=[],
      data_files=[('share/ament_index/resource_index/packages',['resource/robot_description']),
                  ('share/robot_description',['package.xml']),
                  ('share/robot_description/config',glob('config/*.json')),
                  ('share/robot_description/urdf',glob('urdf/*.xacro'))],
      maintainer='FORGE developers', maintainer_email='forge@example.invalid',
      description='FORGE chassis/arm motor inventories and visual macros', license='Proprietary')
