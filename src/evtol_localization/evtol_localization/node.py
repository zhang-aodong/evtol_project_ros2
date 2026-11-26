"""Compatibility entry point; every mode uses the imported branch implementation."""
from importlib import import_module

from .absolute_node import AbsoluteNode as LocalizationNode
from .qos import SENSOR_QOS


def main(args=None):
    from evtol_targeting.common import args_config
    from .settings import estimator_module
    config, _ = args_config(args)
    module = estimator_module(config)
    if module is None:
        raise ValueError('Localization is disabled in this configuration')
    import_module(module).main(args)


if __name__ == '__main__':
    main()
