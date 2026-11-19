from setuptools import setup,find_packages
setup(name='evtol_safety',version='0.1.0',packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/evtol_safety']),('share/evtol_safety',['package.xml'])],
 install_requires=['setuptools'],zip_safe=True,entry_points={'console_scripts':['local_safety = evtol_safety.node:main', 'control_gateway = evtol_safety.gateway:main']})
