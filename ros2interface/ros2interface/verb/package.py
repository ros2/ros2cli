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

from ros2interface.api import package_name_completer
from ros2interface.verb import VerbExtension
from rosidl_runtime_py import get_interfaces


class PackageVerb(VerbExtension):
    """Output a list of available interface types within one package."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '-m', '--only-msgs', action='store_true',
            help='Print out only the message types')

        parser.add_argument(
            '-s', '--only-srvs', action='store_true',
            help='Print out only the service types')

        parser.add_argument(
            '-a', '--only-actions', action='store_true',
            help='Print out only the action types')

        arg = parser.add_argument(
            'package_name',
            help="Name of the ROS package (e.g. 'example_interfaces')")
        arg.completer = package_name_completer

    def main(self, *, args):
        types_to_filter = []
        if args.only_msgs or args.only_srvs or args.only_actions:
            if not args.only_msgs:
                types_to_filter.append('msg')
            if not args.only_srvs:
                types_to_filter.append('srv')
            if not args.only_actions:
                types_to_filter.append('action')

        try:
            interfaces = get_interfaces([args.package_name])
        except LookupError as e:
            return str(e)
        for package_name in sorted(interfaces):
            for interface_name in interfaces[package_name]:
                if types_to_filter:
                    interface_type = interface_name[:interface_name.index('/')]
                    if interface_type in types_to_filter:
                        continue
                print(f'{package_name}/{interface_name}')
