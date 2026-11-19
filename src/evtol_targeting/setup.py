from setuptools import setup,find_packages
setup(name='evtol_targeting',version='0.1.0',packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/evtol_targeting']),('share/evtol_targeting',['package.xml'])],
 install_requires=['setuptools'],zip_safe=True,entry_points={'console_scripts':['ue_detection_boxes = evtol_targeting.ue_detection_boxes:main', 'ego_state = evtol_targeting.state_node:main', 'semantic_node = evtol_targeting.vision_node:main', 'target_localizer = evtol_targeting.localizer_node:main', 'target_evaluation = evtol_targeting.evaluation:main']})
