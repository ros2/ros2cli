# Copyright 2019 Canonical Ltd.
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

from collections import namedtuple

import os
import xml.etree.ElementTree as ET

from ament_index_python import get_package_prefix
from ament_index_python import PackageNotFoundError
from ros2cli.node.strategy import add_arguments

from ros2plugin.api import get_registered_plugin_ressources
from ros2plugin.verb import VerbExtension


PluginInfo = namedtuple('Plugin', ('name', 'type', 'base'))


class ListVerb(VerbExtension):
    """Output a list of plugins."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument(
            '--packages', action='store_true',
            help='List the packages that register plugins')

    def main(self, *, args):
        plugin_ressources = get_registered_plugin_ressources()

        if args.packages:
            for package_name, package_plugin_ressources in sorted(plugin_ressources):
                print(package_name + ':')
                if any(package_plugin_ressources):
                    print(*['\t' + r for r in package_plugin_ressources], sep='\n')
            return

        for package_name, package_plugin_ressources in sorted(plugin_ressources):
            plugins = []
            print(package_name + ':')
            for package_plugin_ressource in package_plugin_ressources:
                try:
                    package_prefix = get_package_prefix(package_name)
                except PackageNotFoundError:
                    print('Package ' + package_name + ' not found.')

                plugin_xml = os.path.join(package_prefix, package_plugin_ressource)
                if not os.path.isfile(plugin_xml):
                    print('XML manifest ' + os.path.basename(plugin_xml) + ' not found.')

                tree = ET.parse(plugin_xml)

                for e in tree.iter():
                    if e.tag == 'class':
                        try:
                            plugin_name = e.attrib['name']
                        except KeyError:
                            plugin_name = e.attrib['type']
                        plugins.append(PluginInfo(
                            plugin_name, e.attrib['type'], e.attrib['base_class_type'])
                        )

            if any(plugins):
                print(*['\t' + str(p) for p in plugins], sep='\n')
