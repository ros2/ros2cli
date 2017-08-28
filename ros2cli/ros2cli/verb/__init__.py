# Copyright 2016-2017 Dirk Thomas
# Copyright 2017 Open Source Robotics Foundation, Inc.
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

from ros2cli.plugin_system import instantiate_extensions
from ros2cli.plugin_system import PLUGIN_SYSTEM_VERSION
from ros2cli.plugin_system import satisfies_version


class VerbExtension:
    """
    The interface for verb extensions.

    The following properties must be defined:
    * `NAME` (will be set to the entry point name)

    The following methods must be defined:
    * `main`
    """

    NAME = None
    EXTENSION_POINT_VERSION = '0.1'

    def __init__(self):
        super(VerbExtension, self).__init__()
        satisfies_version(PLUGIN_SYSTEM_VERSION, '^0.1')


def get_verb_extensions(name):
    extensions = instantiate_extensions(name)
    for name, extension in extensions.items():
        extension.NAME = name
    return extensions


def add_task_arguments(parser, task_name):
    plugins = get_verb_extensions(task_name)
    for plugin_name, plugin in plugins.items():
        group = parser.add_argument_group(
            title="Arguments for '{plugin_name}' packages"
            .format_map(locals()))
        func = getattr(plugin, 'add_%s_arguments' % task_name, None)
        if func:
            func(group)
