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

from rosidl_runtime_py import get_action_interfaces
from rosidl_runtime_py import get_interface_packages
from rosidl_runtime_py import get_message_interfaces
from rosidl_runtime_py import get_service_interfaces
from ros2interface.verb import VerbExtension


def print_packages(packs):
    for name in sorted(packs):
        print((name))


class PackagesVerb(VerbExtension):
    """Output a list of packages that provide interfaces."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument('-m', '--only-msgs', action='count',
                            help='Only list packages that generate messages')

        parser.add_argument('-s', '--only-srvs', action='count',
                            help='Only list packages that generate services')

        parser.add_argument('-a', '--only-actions', action='count',
                            help='Only list packages that generate actions')

    def main(self, *, args):
        if args.only_msgs:
            print_packages(sorted(get_message_interfaces()))
        elif args.only_srvs:
            print_packages(sorted(get_service_interfaces()))
        elif args.only_actions:
            print_packages(sorted(get_action_interfaces()))
        else:
            print_packages(sorted(get_interface_packages()))
