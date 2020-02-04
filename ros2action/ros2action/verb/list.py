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

from ros2action.api import get_action_names_and_types
from ros2action.verb import VerbExtension
from ros2cli.node.strategy import DirectNode


class ListVerb(VerbExtension):
    """Output a list of action names."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '-t', '--show-types', action='store_true',
            help='Additionally show the action type')
        parser.add_argument(
            '-c', '--count-actions', action='store_true',
            help='Only display the number of actions discovered')

    def main(self, *, args):
        with DirectNode(args) as node:
            action_names_and_types = get_action_names_and_types(node=node)

        if args.count_actions:
            print(len(action_names_and_types))
            return

        for name, types in action_names_and_types:
            if args.show_types:
                types_formatted = ', '.join(types)
                print(f'{name} [{types_formatted}]')
            else:
                print(f'{name}')
