from setuptools import setup, find_packages
setup(name='evtol_scenario',version='0.1.0',packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/evtol_scenario']),
             ('share/evtol_scenario',['package.xml'])],install_requires=['setuptools'],zip_safe=True,
 entry_points={'console_scripts':['scenario_node = evtol_scenario.node:main',
                                 'actors_node = evtol_scenario.actors:main']})
