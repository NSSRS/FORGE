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
      data_files=[('share/ament_index/resource_index/packages', ['resource/robot_description']), ('share/robot_description', ['package.xml'])] + model_files, install_requires=['setuptools'], zip_safe=False,
      maintainer='FORGE developers', maintainer_email='forge@example.invalid',
      description='Shared full-robot MJCF models and mesh assets', license='Proprietary',
      entry_points={'console_scripts': []})
