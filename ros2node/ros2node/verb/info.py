# Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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
import sys

from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2node.api import get_action_client_info
from ros2node.api import get_action_server_info
from ros2node.api import get_node_names
from ros2node.api import get_publisher_info
from ros2node.api import get_service_client_info
from ros2node.api import get_service_server_info
from ros2node.api import get_subscriber_info
from ros2node.api import INFO_NONUNIQUE_WARNING_TEMPLATE
from ros2node.api import NodeNameCompleter
from ros2node.verb import VerbExtension


def print_names_and_types(names_and_types):
    print(*[2 * '  ' + s.name + ': ' + ', '.join(s.types) for s in names_and_types], sep='\n')


class InfoVerb(VerbExtension):
    """Output information about a node."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        argument = parser.add_argument(
            'node_name',
            help='Node name to request information')
        argument.completer = NodeNameCompleter()
        parser.add_argument(
            '--include-hidden', action='store_true',
            help='Display hidden topics, services, and actions as well')

    def main(self, *, args):
        with NodeStrategy(args) as node:
            node_names = get_node_names(node=node, include_hidden_nodes=args.include_hidden)
        count = [n.full_name for n in node_names].count(args.node_name)
        if count > 1:
            print(
                INFO_NONUNIQUE_WARNING_TEMPLATE.format(num_nodes=count, node_name=args.node_name),
                file=sys.stderr)
        if count > 0:
            with DirectNode(args) as node:
                print(args.node_name)
                subscribers = get_subscriber_info(
                    node=node, remote_node_name=args.node_name, include_hidden=args.include_hidden)
                print('  Subscribers:')
                print_names_and_types(subscribers)
                publishers = get_publisher_info(
                    node=node, remote_node_name=args.node_name, include_hidden=args.include_hidden)
                print('  Publishers:')
                print_names_and_types(publishers)
                service_servers = get_service_server_info(
                    node=node, remote_node_name=args.node_name, include_hidden=args.include_hidden)
                print('  Service Servers:')
                print_names_and_types(service_servers)
                service_clients = get_service_client_info(
                    node=node, remote_node_name=args.node_name, include_hidden=args.include_hidden)
                print('  Service Clients:')
                print_names_and_types(service_clients)
                actions_servers = get_action_server_info(
                    node=node, remote_node_name=args.node_name, include_hidden=args.include_hidden)
                print('  Action Servers:')
                print_names_and_types(actions_servers)
                actions_clients = get_action_client_info(
                    node=node, remote_node_name=args.node_name, include_hidden=args.include_hidden)
                print('  Action Clients:')
                print_names_and_types(actions_clients)
        else:
            return "Unable to find node '" + args.node_name + "'"
