from glob import glob
from setuptools import setup
setup(name='evtol_bringup',version='0.1.0',packages=[],
 data_files=[('share/ament_index/resource_index/packages',['resource/evtol_bringup']),
             ('share/evtol_bringup',['package.xml']),
             ('share/evtol_bringup/launch',glob('launch/*.py')),
             ('share/evtol_bringup/rviz',glob('rviz/*')),
             ('share/evtol_bringup/config',glob('config/*'))],install_requires=['setuptools'],zip_safe=True)
