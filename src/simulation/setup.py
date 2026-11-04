from setuptools import find_packages, setup

setup(name='simulation', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/simulation']), ('share/simulation', ['package.xml'])], install_requires=['setuptools'], zip_safe=False,
      maintainer='FORGE developers', maintainer_email='forge@example.invalid',
      description='MuJoCo modeling, CAD integration and robot previews', license='Proprietary',
      entry_points={'console_scripts': ['export_onshape = simulation.export_onshape:main', 'integrate_sim = simulation.integrate_sim:main', 'preview_arm = simulation.preview_arm:main', 'preview_native = simulation.preview_native:main', 'preview_sim = simulation.preview_sim:main', 'terrain = simulation.terrain:main']})
