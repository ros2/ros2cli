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

from ros2interface.api import type_completer
from ros2interface.verb import VerbExtension
from rosidl_runtime_py import get_interface_path


class ShowVerb(VerbExtension):
    """Output the interface definition."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
                'type',
                help='Show an interface definition (e.g. "std_msgs/msg/String")')
        arg.completer = type_completer

    def main(self, *, args):
        try:
            file_path = get_interface_path(args.type)
        except LookupError as e:
            return str(e)
        with open(file_path, 'r', encoding='utf-8') as h:
            print(h.read().rstrip())
