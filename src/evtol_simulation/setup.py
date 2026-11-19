from glob import glob
from setuptools import find_packages, setup

setup(name='evtol_simulation', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/evtol_simulation']),
                  ('share/evtol_simulation', ['package.xml']),
                  ('share/evtol_simulation/config', glob('config/*'))],
      install_requires=['setuptools', 'numpy', 'PyYAML'], zip_safe=True,
      maintainer='EVTOL Simulation', maintainer_email='maintainer@example.com',
      description='AirSim sensor publishers and independent acceptance checks', license='Apache-2.0',
      entry_points={'console_scripts': [
          'sensor_node = evtol_simulation.sensor_node:main',
          'validate_sensors = evtol_simulation.validation:main',
          'preflight = evtol_simulation.preflight:main']})
