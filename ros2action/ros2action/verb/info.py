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

import rclpy
from ros2action.api import action_name_completer
from ros2action.api import get_action_clients_and_servers
from ros2action.verb import VerbExtension
from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy


class InfoVerb(VerbExtension):
    """Print information about an action."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'action_name',
            help="Name of the ROS action to get info (e.g. '/fibonacci')")
        arg.completer = action_name_completer
        parser.add_argument(
            '-t', '--show-types', action='store_true',
            help='Additionally show the action type')
        parser.add_argument(
            '-c', '--count', action='store_true',
            help='Only display the number of action clients and action servers')
        add_arguments(parser)

    def main(self, *, args):
        with NodeStrategy(args) as node:
            try:
                action_clients, action_servers = get_action_clients_and_servers(
                    node=node,
                    action_name=args.action_name,
                )
            except (ValueError, rclpy.exceptions.InvalidTopicNameException) as e:
                raise RuntimeError(e)

        print('Action: {}'.format(args.action_name))
        print('Action clients: {}'.format(len(action_clients)))
        if not args.count:
            for client_name, client_types in action_clients:
                if args.show_types:
                    types_formatted = ', '.join(client_types)
                    print(f'    {client_name} [{types_formatted}]')
                else:
                    print(f'    {client_name}')
        print('Action servers: {}'.format(len(action_servers)))
        if not args.count:
            for server_name, server_types in action_servers:
                if args.show_types:
                    types_formatted = ', '.join(server_types)
                    print(f'    {server_name} [{types_formatted}]')
                else:
                    print(f'    {server_name}')
