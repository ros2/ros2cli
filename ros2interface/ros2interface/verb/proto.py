# Copyright 2019 Canonical Ldt.
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

import importlib

from ros2cli.node.strategy import add_arguments
from ros2interface.api import type_completer
from ros2interface.api import interface_to_yaml
from ros2interface.verb import VerbExtension


class ProtoVerb(VerbExtension):
    """Output an interface prototype."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
                'type',
                help='Show an interface definition (e.g. "std_msgs/msg/String")')
        arg.completer = type_completer
        parser.add_argument(
            '--no-hyphens', action='store_true',
            help='if true output has no outer hyphens.')

    def main(self, *, args):
        try:
            parts = args.type.split('/')
            package_name = parts[0]
            interface_type = parts[1]
            interface_name = parts[-1]
        except LookupError as e:
            return str(e)

        yaml = interface_to_yaml(package_name, interface_type, interface_name)

        if args.no_hyphens is True:
            print(yaml)
        else:
            print('"' + yaml + '"')
