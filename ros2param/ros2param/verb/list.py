# Copyright 2018 Open Source Robotics Foundation, Inc.
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

from rcl_interfaces.srv import ListParameters
import rclpy
from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2node.api import get_node_names
from ros2node.api import NodeNameCompleter
from ros2param.verb import VerbExtension


class ListVerb(VerbExtension):
    """Output a list of available parameters."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        add_arguments(parser)
        arg = parser.add_argument(
            'node_name', nargs='?', help='Name of the ROS node')
        arg.completer = NodeNameCompleter(
            include_hidden_nodes_key='include_hidden_nodes')
        parser.add_argument(
            '--include-hidden-nodes', action='store_true',
            help='Consider hidden nodes as well')

    def main(self, *, args):  # noqa: D102
        with NodeStrategy(args) as node:
            node_names = get_node_names(
                node=node, include_hidden_nodes=args.include_hidden_nodes)

        if args.node_name:
            if args.node_name not in node_names:
                return 'Node not found'
            node_names = [args.node_name]

        with DirectNode(args) as node:
            clients = {}
            futures = {}
            # create clients
            for node_name in node_names:
                client = node.create_client(
                    ListParameters,
                    '/{node_name}/list_parameters'.format_map(locals()))
                clients[node_name] = client

            # wait until all clients have been called
            while True:
                for node_name in [n for n in node_names if n not in futures]:
                    # call as soon as ready
                    client = clients[node_name]
                    if client.service_is_ready():
                        request = ListParameters.Request()
                        future = client.call_async(request)
                        futures[node_name] = future

                if len(futures) == len(clients):
                    break
                rclpy.spin_once(node, timeout_sec=1.0)

            # wait for all responses
            for node_name in node_names:
                rclpy.spin_until_future_complete(node, futures[node_name])

            # print responses
            for node_name in sorted(futures.keys()):
                future = futures[node_name]
                if future.result() is not None:
                    print('{node_name}:'.format_map(locals()))
                    response = future.result()
                    for name in sorted(response.result.names):
                        print('  {name}'.format_map(locals()))
                else:
                    e = future.exception()
                    print(
                        'Exception while calling service of node '
                        "'{node_name}': {e}".format_map(locals()),
                        file=sys.stderr)
