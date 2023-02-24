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

from rcl_interfaces.msg import Parameter
from rclpy.parameter import get_parameter_value
from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2node.api import get_absolute_node_name
from ros2node.api import NodeNameCompleter
from ros2node.api import wait_for_node

from ros2param.api import call_set_parameters
from ros2param.api import ParameterNameCompleter
from ros2param.verb import VerbExtension


class SetVerb(VerbExtension):
    """Set parameter."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        add_arguments(parser)
        arg = parser.add_argument(
            'node_name', help='Name of the ROS node')
        arg.completer = NodeNameCompleter(
            include_hidden_nodes_key='include_hidden_nodes')
        parser.add_argument(
            '--include-hidden-nodes', action='store_true',
            help='Consider hidden nodes as well')
        arg = parser.add_argument(
            'parameter_name', help='Name of the parameter')
        arg.completer = ParameterNameCompleter()
        parser.add_argument(
            'value', help='Value of the parameter')
        parser.add_argument(
            '--timeout', metavar='N', type=int, default=1,
            help='Wait for N seconds until node becomes available (default %(default)s sec)')

    def main(self, *, args):  # noqa: D102
        node_name = get_absolute_node_name(args.node_name)
        with NodeStrategy(args) as node:
            if not wait_for_node(node, node_name, args.include_hidden_nodes, args.timeout):
                return 'Node not found'

        with DirectNode(args) as node:
            parameter = Parameter()
            parameter.name = args.parameter_name
            parameter.value = get_parameter_value(string_value=args.value)

            response = call_set_parameters(
                node=node, node_name=args.node_name, parameters=[parameter])

            # output response
            assert len(response.results) == 1
            result = response.results[0]
            if result.successful:
                msg = 'Set parameter successful'
                if result.reason:
                    msg += ': ' + result.reason
                print(msg)
            else:
                msg = 'Setting parameter failed'
                if result.reason:
                    msg += ': ' + result.reason
                print(msg, file=sys.stderr)
