# Copyright 2024 Sony Group Corporation.
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

from ros2action.api import action_type_completer
from ros2action.api import get_action_names_and_types
from ros2action.verb import VerbExtension
from ros2cli.node.strategy import NodeStrategy


class FindVerb(VerbExtension):
    """Find actions from type."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'action_type',
            help="Name of the ROS action type to find (e.g. 'test_msgs/action/Fibonacci')")
        arg.completer = action_type_completer
        parser.add_argument(
            '-c', '--count-actions', action='store_true',
            help='Only display the number of actions discovered')

    def main(self, *, args):
        with NodeStrategy(args) as node:
            action_names_and_types = get_action_names_and_types(node=node)

        filtered_actions = []
        for (action_name, action_types) in action_names_and_types:
            if args.action_type in action_types:
                filtered_actions.append(action_name)

        if args.count_actions:
            print(len(filtered_actions))
        else:
            for filtered_action in filtered_actions:
                print(filtered_action)
