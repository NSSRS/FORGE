from glob import glob
from setuptools import find_packages, setup
from pathlib import Path
# Explicit asset suffixes exclude credentials, caches, backups and export staging.
model_files = []
for folder in [Path('mujoco'), Path('mujoco/assets')]:
    files = [str(p) for p in sorted(folder.iterdir())
             if p.is_file() and p.suffix in ('.xml', '.json', '.obj', '.stl')
             and p.name != 'config.json']
    model_files.append(('share/robot_description/' + str(folder), files))

setup(name='robot_description', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/robot_description']), ('share/robot_description', ['package.xml']),
                  ('share/robot_description/config', glob('config/*.json')),
                  ('share/robot_description/urdf', glob('urdf/*.xacro'))] + model_files, install_requires=['setuptools'], zip_safe=False,
      maintainer='FORGE developers', maintainer_email='forge@example.invalid',
      description='Shared MJCF assets, motor inventories and visual macros', license='Proprietary',
      entry_points={'console_scripts': []})
