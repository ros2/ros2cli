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
from ros2node.api import get_absolute_node_name
from ros2node.api import get_node_names
from ros2node.api import NodeNameCompleter
from ros2param.api import call_describe_parameters
from ros2param.api import get_parameter_type_string
from ros2param.verb import VerbExtension
from ros2service.api import get_service_names


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
        parser.add_argument(
            '--param-prefixes', nargs='+', default=[],
            help='Only list parameters with the provided prefixes')
        parser.add_argument(
            '--param-type', action='store_true',
            help='Print parameter types with parameter names')

    def main(self, *, args):  # noqa: D102
        with NodeStrategy(args) as node:
            node_names = get_node_names(
                node=node, include_hidden_nodes=args.include_hidden_nodes)

        node_name = get_absolute_node_name(args.node_name)
        if node_name:
            if node_name not in [n.full_name for n in node_names]:
                return 'Node not found'
            node_names = [
                n for n in node_names if node_name == n.full_name]

        with DirectNode(args) as node:
            service_names = get_service_names(
                node=node, include_hidden_services=args.include_hidden_nodes)

            clients = {}
            futures = {}
            # create clients for nodes which have the service
            for node_name in node_names:
                service_name = f'{node_name.full_name}/list_parameters'
                if service_name in service_names:
                    client = node.create_client(ListParameters, service_name)
                    clients[node_name] = client

            # wait until all clients have been called
            while True:
                for node_name in [
                    n for n in clients.keys() if n not in futures
                ]:
                    # call as soon as ready
                    client = clients[node_name]
                    if client.service_is_ready():
                        request = ListParameters.Request()
                        for prefix in args.param_prefixes:
                            request.prefixes.append(prefix)
                        future = client.call_async(request)
                        futures[node_name] = future

                if len(futures) == len(clients):
                    break
                rclpy.spin_once(node, timeout_sec=1.0)

            # wait for all responses
            for future in futures.values():
                rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)

            # print responses
            for node_name in sorted(futures.keys()):
                future = futures[node_name]
                if future.result() is not None:
                    if not args.node_name:
                        print(f'{node_name.full_name}:')
                    response = future.result()
                    # get descriptors for the node if needs to print parameter type
                    if args.param_type is True:
                        resp = call_describe_parameters(
                            node=node, node_name=node_name.full_name,
                            parameter_names=sorted(response.result.names))
                    for name in sorted(response.result.names):
                        if args.param_type is True:
                            param_type_str = None
                            for descriptor in resp.descriptors:
                                if descriptor.name == name:
                                    param_type_str = get_parameter_type_string(descriptor.type)
                            print(f'  {name} (type: {param_type_str})')
                        else:
                            print(f'  {name}')
                else:
                    e = future.exception()
                    print(
                        'Exception while calling service of node '
                        f"'{node_name.full_name}': {e}", file=sys.stderr)
