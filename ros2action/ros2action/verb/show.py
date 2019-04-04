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

from ros2action.api import action_type_completer
from ros2action.api import get_action_path
from ros2action.verb import VerbExtension


class ShowVerb(VerbExtension):
    """Output the action definition."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'action_type',
            help="Type of the ROS action (e.g. 'examples_interfaces/Fibonacci')")
        arg.completer = action_type_completer

    def main(self, *, args):
        package_name, action_name = args.action_type.split('/', 2)
        if not package_name or not action_name:
            raise RuntimeError('The passed action type is invalid')
        try:
            path = get_action_path(package_name, action_name)
        except LookupError as e:
            return str(e)
        with open(path, 'r') as action_file:
            print(action_file.read(), end='')
