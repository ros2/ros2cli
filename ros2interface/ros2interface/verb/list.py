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

from ros2interface.verb import VerbExtension
from rosidl_runtime_py import get_action_interfaces
from rosidl_runtime_py import get_message_interfaces
from rosidl_runtime_py import get_service_interfaces


def print_messages():
    print('Messages:')
    message_interfaces = get_message_interfaces()
    for package_name in sorted(message_interfaces):
        for message_name in sorted(message_interfaces[package_name]):
            print(f'    {package_name}/{message_name}')


def print_services():
    print('Services:')
    service_interfaces = get_service_interfaces()
    for package_name in sorted(service_interfaces):
        for service_name in sorted(service_interfaces[package_name]):
            print(f'    {package_name}/{service_name}')


def print_actions():
    print('Actions:')
    action_interfaces = get_action_interfaces()
    for package_name in sorted(action_interfaces):
        for action_name in sorted(action_interfaces[package_name]):
            print(f'    {package_name}/{action_name}')


class ListVerb(VerbExtension):
    """List all interface types available."""

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

    def main(self, *, args):
        if args.only_msgs or args.only_srvs or args.only_actions:
            if args.only_msgs:
                print_messages()
            if args.only_srvs:
                print_services()
            if args.only_actions:
                print_actions()
            return
        print_messages()
        print_services()
        print_actions()
