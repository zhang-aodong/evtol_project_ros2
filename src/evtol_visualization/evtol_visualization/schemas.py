"""Expand installed ROS .msg definitions for Foxglove's ros2msg/CDR decoder."""
from pathlib import Path
from rosidl_adapter.parser import parse_message_string
from rosidl_runtime_py import get_interface_path


def expand_schema(type_name):
    package, kind, name = type_name.split('/')
    if kind != 'msg':
        raise ValueError('Expected package/msg/Type: ' + type_name)
    definitions = {}

    def visit(pkg, msg):
        key = pkg + '/' + msg
        if key in definitions:
            return
        content = Path(get_interface_path(pkg+'/msg/'+msg)).read_text(encoding='utf-8')
        definitions[key] = content.rstrip()+'\n'
        parsed = parse_message_string(pkg, msg, content)
        for field in parsed.fields:
            if not field.type.is_primitive_type():
                visit(field.type.pkg_name, field.type.type)

    visit(package, name)
    root = definitions.pop(package+'/'+name)
    return (root + ''.join('\n'+'='*80+'\nMSG: '+key+'\n'+value
                           for key, value in definitions.items())).encode('utf-8')
