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

from ros2action.api import get_action_clients_and_servers
from ros2action.verb import VerbExtension
from ros2cli.node.direct import DirectNode


class InfoVerb(VerbExtension):
    """Print information about an action."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            'action_name',
            help="Name of the ROS action to get info (e.g. '/fibonacci')")
        parser.add_argument(
            '-c', '--count', action='store_true',
            help='Only display the number of action clients and action servers')

    def main(self, *, args):
        with DirectNode(args) as node:
            action_clients, action_servers = get_action_clients_and_servers(
                node=node,
                action_name=args.action_name,
            )

        print('Action: {}'.format(args.action_name))
        print('Action clients: {}'.format(len(action_clients)))
        if not args.count:
            for client in action_clients:
                print('    {}'.format(client))
        print('Action servers: {}'.format(len(action_servers)))
        if not args.count:
            for server in action_servers:
                print('    {}'.format(server))
