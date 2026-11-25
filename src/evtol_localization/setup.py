from setuptools import setup, find_packages
setup(name='evtol_localization', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages',['resource/evtol_localization']),
                  ('share/evtol_localization',['package.xml'])],
      install_requires=['setuptools','numpy','PyYAML'], zip_safe=True,
      entry_points={'console_scripts':['ego_localization = evtol_localization.node:main',
                                      'localization_evaluation = evtol_localization.evaluation_node:main',
                                      'absolute = evtol_localization.absolute_node:main',
                                      'vio = evtol_localization.vio_node:main',
                                      'accuracy = evtol_localization.accuracy:main']})
