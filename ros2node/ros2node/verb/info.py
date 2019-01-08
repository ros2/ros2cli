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

from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2node.api import get_node_names
from ros2node.api import get_publisher_info
from ros2node.api import get_service_info
from ros2node.api import get_subscriber_info
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

    def main(self, *, args):
        with NodeStrategy(args) as node:
            node_names = get_node_names(node=node, include_hidden_nodes=True)
        if args.node_name in [n.full_name for n in node_names]:
            with DirectNode(args) as node:
                print(args.node_name)
                subscribers = get_subscriber_info(node=node, remote_node_name=args.node_name[1:])
                print('  Subscribers:')
                print_names_and_types(subscribers)
                publishers = get_publisher_info(node=node, remote_node_name=args.node_name[1:])
                print('  Publishers:')
                print_names_and_types(publishers)
                services = get_service_info(node=node, remote_node_name=args.node_name[1:])
                print('  Services:')
                print_names_and_types(services)
        else:
            return "Unable to find node '" + args.node_name + "'"
