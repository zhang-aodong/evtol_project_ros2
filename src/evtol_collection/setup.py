from setuptools import setup,find_packages
setup(name='evtol_collection',version='0.1.0',packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/evtol_collection']),('share/evtol_collection',['package.xml'])],
 install_requires=['setuptools','numpy','PyYAML'],entry_points={'console_scripts':['collect = evtol_collection.recorder:main']})
