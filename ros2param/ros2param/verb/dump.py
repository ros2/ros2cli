# Copyright 2019 Canonical Ltd.
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

from rclpy.parameter import PARAMETER_SEPARATOR_STRING
from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2node.api import get_absolute_node_name
from ros2node.api import NodeNameCompleter
from ros2node.api import parse_node_name
from ros2node.api import wait_for_node

from ros2param.api import call_get_parameters
from ros2param.api import call_list_parameters
from ros2param.api import get_value
from ros2param.verb import VerbExtension
import yaml


class DumpVerb(VerbExtension):
    """Show all of the parameters of a node in a YAML file format."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        add_arguments(parser)
        arg = parser.add_argument(
            'node_name', help='Name of the ROS node')
        arg.completer = NodeNameCompleter(
            include_hidden_nodes_key='include_hidden_nodes')
        parser.add_argument(
            '--include-hidden-nodes', action='store_true',
            help='Consider hidden nodes as well')
        parser.add_argument(
            '--timeout', metavar='N', type=int, default=1,
            help='Wait for N seconds until node becomes available (default %(default)s sec)')

    def insert_dict(self, dictionary, key, value):
        split = key.split(PARAMETER_SEPARATOR_STRING, 1)
        if len(split) > 1:
            if not split[0] in dictionary:
                dictionary[split[0]] = {}
            self.insert_dict(dictionary[split[0]], split[1], value)
        else:
            dictionary[key] = value

    def main(self, *, args):  # noqa: D102
        absolute_node_name = get_absolute_node_name(args.node_name)
        with NodeStrategy(args) as node:
            if not wait_for_node(node, absolute_node_name,
                                 args.include_hidden_nodes, args.timeout):
                return 'Node not found'

        node_name = parse_node_name(absolute_node_name)

        with DirectNode(args) as node:
            yaml_output = {node_name.full_name: {'ros__parameters': {}}}

            # retrieve parameter names
            response = None
            response = call_list_parameters(node=node, node_name=absolute_node_name)
            if response is None:
                print(
                    'Wait for service timed out waiting for '
                    f'parameter services for node {node_name.full_name}', file=sys.stderr)
                return
            elif response.result() is None:
                e = response.exception()
                print(
                    'Exception while calling list_parameters service of node '
                    f"'{node_name.full_name}': {e}", file=sys.stderr)
                return
            parameter_names = sorted(response.result().result.names)

            # retrieve parameter values
            response = None
            try:
                response = call_get_parameters(
                    node=node, node_name=absolute_node_name, parameter_names=parameter_names)
            except RuntimeError as e:
                print(
                    'Exception while calling get_parameters service of node '
                    f"'{node_name.full_name}': {e}", file=sys.stderr)
                return
            if response.values is None:
                # pass through here, no parameters are available with this node.
                # since this is not failure, it proceeds to print the yaml as consistent behavior.
                pass
            parameter_values = [get_value(parameter_value=i) for i in response.values]

            # create dictionary with parameter names and values
            for param_name, pval in zip(parameter_names, parameter_values):
                self.insert_dict(
                    yaml_output[node_name.full_name]['ros__parameters'], param_name, pval)

            print(yaml.dump(yaml_output, default_flow_style=False))
            return
