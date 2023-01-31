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
import collections
from ros2interface.api import package_name_completer
from ros2interface.verb import VerbExtension
from rosidl_runtime_py import get_action_interfaces, get_interfaces
from rosidl_runtime_py import get_message_interfaces, get_service_interfaces


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
        interfaces = collections.defaultdict(list)
        try:
            if not args.only_msgs and not args.only_srvs and not args.only_actions:
                interfaces = get_interfaces([args.package_name])
            else:
                get_commands = []
                if args.only_msgs:
                    get_commands.append(get_message_interfaces)
                if args.only_srvs:
                    get_commands.append(get_service_interfaces)
                if args.only_actions:
                    get_commands.append(get_action_interfaces)

                for get_interface_cmd in get_commands:
                    pkg_interfaces = get_interface_cmd([args.package_name])
                    for package_name, interface_names in pkg_interfaces.items():
                        interfaces[package_name] += interface_names
        except LookupError as e:
            return str(e)
        for package_name in sorted(interfaces):
            for interface_name in interfaces[package_name]:
                print(f'{package_name}/{interface_name}')
