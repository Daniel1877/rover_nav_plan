"""Main entry point for the `launch_helpers` package."""

import os
import re
import tempfile

from ament_index_python.packages import get_package_prefix

import xacro

import launch.substitutions

def add_namespace_to_yaml(namespace, yaml_path, ns_yaml_path=None):
    """Make config files reusable in multiple namespaces by generating a new yaml with a namespace appended.

    * yaml_path -- the path to the yaml file
    * ns_yaml_path -- the path to the namespaced yaml file
    """
    # If no namespace is given return original path
    if namespace == '':
        return yaml_path

    # If no YAML path is given, use a temporary file
    if ns_yaml_path is None:
        ns_yaml_path = tempfile.mktemp(prefix='%s_' % os.path.basename(yaml_path))

    # open and process file
    with open(yaml_path, 'r') as yaml_file, open(ns_yaml_path, 'w') as ns_yaml_file:
        ns_yaml_file.write('# Temporary Namespace\n')
        ns_yaml_file.write(namespace + ':\n')
        for line in yaml_file:
            ns_yaml_file.write('  ' + line)

    # Return path to the yaml file
    return ns_yaml_path


def to_urdf(xacro_path, urdf_path=None, mappings={}):
    """Convert the given xacro file to URDF file.

    * xacro_path -- the path to the xacro file
    * urdf_path -- the path to the urdf file
    """
    # If no URDF path is given, use a temporary file
    if urdf_path is None:
        urdf_path = tempfile.mktemp(prefix='%s_' % os.path.basename(xacro_path))

    # open and process file
    doc = xacro.process_file(xacro_path, mappings=mappings)
    # open the output file
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    robot_desc = launch.substitutions.Command('xacro %s' % xacro_path)

    return urdf_path, robot_desc  # Return robot description
    # return urdf_path  # Return path to the urdf file


def get_ws_src_directory(package_name):
    """Return the ros2 workspace src path determined by the install location of the given package.

    * package_name -- the name of the package, which is used to find the workspace.
    * ros_ws_src -- src directory appended to ros2 workspace.
    """
    package_prefix = get_package_prefix(package_name)
    result = re.search('.+?(?=install)', package_prefix)
    if result is None:
        print(
            'ERROR IN get_ws_src_directory: Could not find path.'
            'Make sure the package is installed from source.'
        )

    ros_ws_src = os.path.join(result.group(), 'src')

    return ros_ws_src
