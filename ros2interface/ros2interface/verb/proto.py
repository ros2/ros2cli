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

from ros2interface.api import interface_to_yaml
from ros2interface.api import type_completer
from ros2interface.verb import VerbExtension


class ProtoVerb(VerbExtension):
    """Output an interface prototype."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
                'type',
                help="Show an interface definition (e.g. 'std_msgs/msg/String')")
        arg.completer = type_completer
        parser.add_argument(
            '--no-quotes', action='store_true',
            help='if true output has no outer quotes.')

    def main(self, *, args):
        yaml = interface_to_yaml(args.type)

        if args.no_quotes is True:
            print(yaml)
        else:
            print('"' + yaml + '"')
