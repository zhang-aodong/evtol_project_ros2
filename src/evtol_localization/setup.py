from setuptools import setup

setup(
    name="evtol_localization",
    version="0.1.0",
    packages=["evtol_localization"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/evtol_localization"]),
        ("share/evtol_localization", ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    entry_points={"console_scripts": [
        "absolute = evtol_localization.absolute_node:main",
        "vio = evtol_localization.vio_node:main",
        "accuracy = evtol_localization.accuracy:main",
    ]},
)
