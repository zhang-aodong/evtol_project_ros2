from setuptools import setup,find_packages
setup(name='evtol_console',version='0.1.0',packages=find_packages(),include_package_data=True,
 package_data={'evtol_console':['web/*.html','web/*.js']},
 data_files=[('share/ament_index/resource_index/packages',['resource/evtol_console']),('share/evtol_console',['package.xml'])],
 install_requires=['setuptools'],entry_points={'console_scripts':['console_manager = evtol_console.server:main']})
