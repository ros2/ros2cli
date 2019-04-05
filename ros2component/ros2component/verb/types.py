# Copyright 2019 Open Source Robotics Foundation, Inc.
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

from ros2cli.node.strategy import add_arguments
from ros2component.api import get_package_component_types
from ros2component.api import get_registered_component_types
from ros2component.verb import VerbExtension

from ros2pkg.api import get_package_names
from ros2pkg.api import package_name_completer


class TypesVerb(VerbExtension):
    """Output a list of available containers and components."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        argument = parser.add_argument(
            'package_name', nargs='?', default=None,
            help='Optional package name to narrow down the list'
        )
        argument.completer = package_name_completer

    def main(self, *, args):
        if args.package_name is not None:
            if args.package_name not in get_package_names():
                return "Unable to find package '" + args.package_name + "'"
            print(*get_package_component_types(package_name=args.package_name), sep='\n')
        else:
            for package_name, component_types in get_registered_component_types():
                print(package_name)
                print(*['  ' + type_name for type_name in component_types], sep='\n')
