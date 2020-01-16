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
from ros2cli.plugin_system import instantiate_extensions
from ros2cli.plugin_system import PLUGIN_SYSTEM_VERSION
from ros2cli.plugin_system import satisfies_version


class BuildTypeExtension:
    """
    The interface for build-type extensions.

    The following properties must be defined:
    * `NAME` (will be set to the entry point name)

    The following methods must be defined:
    * `main`
    """

    NAME = None
    EXTENSION_POINT_NAME = 'ros2pkg.build_type'
    EXTENSION_POINT_VERSION = '0.1'
    NATIVE_BUILD_TYPES = ['cmake', 'ament_cmake', 'ament_python']

    def __init__(self):
        super(BuildTypeExtension, self).__init__()
        satisfies_version(PLUGIN_SYSTEM_VERSION, '^0.1')

    def populate(self, package, package_directory, node_name, library_name):
        pass

    def main(self, *, args):
        raise NotImplementedError()


def get_build_type_extension_names():
    names = []
    names.extend(
      get_entry_points(BuildTypeExtension.EXTENSION_POINT_NAME).keys())
    names.sort()
    return names


def get_build_type_names():
    build_type_names = BuildTypeExtension.NATIVE_BUILD_TYPES.copy()
    build_type_names.extend(get_build_type_extension_names())
    return build_type_names


def has_build_type_extension(name):
    return name in get_build_type_extension_names()


def get_build_type_extensions():
    extensions = instantiate_extensions(BuildTypeExtension.EXTENSION_POINT_NAME)
    for name, extension in extensions.items():
        extension.NAME = name
    return extensions


def get_build_type_extension(name):
    extensions = get_build_type_extensions()
    return extensions.get(name)
