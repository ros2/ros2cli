# Copyright 2020 Wayne Parrott
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ros2cli.entry_points import get_entry_points
from ros2cli.plugin_system import _instantiate_extension
from ros2cli.plugin_system import PLUGIN_SYSTEM_VERSION
from ros2cli.plugin_system import satisfies_version


class PackageTypeExtension():
    """
    The interface for package-type extensions.

    The following properties must be defined:
    * `NAME` (will be set to the entry point name)

    The following methods must be defined:
    * create_package
    """

    NAME = None
    EXTENSION_POINT_NAME = 'ros2pkg.package_type'
    EXTENSION_POINT_VERSION = '0.1'
    NATIVE_PACKAGE_TYPES = ['ament_cmake', 'ament_python', 'cmake']

    def __init__(self):
        super(PackageTypeExtension, self).__init__()
        satisfies_version(PLUGIN_SYSTEM_VERSION, '^0.1')

    def add_arguments(self, parser, cli_name):
        pass

    def create_package(self, args):
        """Create the ROS 2 package directory structure and content."""
        raise NotImplementedError


def get_package_type_names():
    """
    Get the names of the combined NATIVE_PACKAGE_TYPES and the package-type extensions.

    :return: Sorted list of package-type string names.
    """
    names = set(PackageTypeExtension.NATIVE_PACKAGE_TYPES)
    entry_pts = get_entry_points(PackageTypeExtension.EXTENSION_POINT_NAME).keys()
    if len(entry_pts) > 0:
        names.update(entry_pts)
    return sorted(names)


def get_package_type_extension(name):
    """
    Get the package-type extension with name.

    :param name: The name of the package-type extension to search.
    :return: The package-type extension or None if extension is not found.
    """
    entry_pts = get_entry_points(PackageTypeExtension.EXTENSION_POINT_NAME)
    if name not in entry_pts:
        return None
    extension = _instantiate_extension(
                                      PackageTypeExtension.EXTENSION_POINT_NAME,
                                      name,
                                      entry_pts[name].load())
    return extension
