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

from ros2msg.api import get_message_path
from ros2msg.api import message_type_completer
from ros2msg.verb import VerbExtension


class ShowVerb(VerbExtension):
    """Output the message definition."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'message_type',
            help="Type of the ROS message (e.g. 'std_msgs/String')")
        arg.completer = message_type_completer

    def main(self, *, args):
        # TODO(dirk-thomas) this logic should come from a rosidl related
        # package
        try:
            package_name, message_name = args.message_type.split('/', 2)
            if not package_name or not message_name:
                raise ValueError()
        except ValueError:
            raise RuntimeError('The passed message type is invalid')
        try:
            path = get_message_path(package_name, message_name)
        except LookupError as e:
            return str(e)
        with open(path, 'r') as h:
            print(h.read(), end='')
