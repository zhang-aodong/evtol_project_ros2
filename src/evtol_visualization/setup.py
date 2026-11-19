from setuptools import find_packages, setup

setup(name='evtol_visualization', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/evtol_visualization']),
                  ('share/evtol_visualization', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='EVTOL Simulation', maintainer_email='maintainer@example.com',
      description='Independent visualization previews and Foxglove transport', license='Apache-2.0',
      entry_points={'console_scripts': [
          'preview_node = evtol_visualization.preview:main',
          'foxglove_node = evtol_visualization.bridge:main']})
