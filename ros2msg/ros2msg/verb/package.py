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

import warnings

from ros2msg.api import get_message_types
from ros2msg.api import message_package_name_completer
from ros2msg.verb import VerbExtension


class PackageVerb(VerbExtension):
    """Output a list of available message types within one package."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'package_name',
            help="Name of the ROS package (e.g. 'std_msgs')")
        arg.completer = message_package_name_completer

    def main(self, *, args):
        warnings.warn(
            "'ros2 msg' is deprecated and will be removed in a future ROS release. "
            f"Instead use: 'ros2 interface package {args.package_name}'"
        )
        try:
            message_names = get_message_types(args.package_name)
        except LookupError as e:
            return str(e)
        for message_name in sorted(message_names):
            print('{args.package_name}/msg/{message_name}'.format_map(locals()))
