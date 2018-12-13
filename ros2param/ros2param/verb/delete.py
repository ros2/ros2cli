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
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2node.api import get_node_names
from ros2node.api import NodeNameCompleter
from ros2param.api import call_set_parameters
from ros2param.verb import VerbExtension


class DeleteVerb(VerbExtension):
    """Delete parameter."""

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
            'name', help='Name of the parameter')

    def main(self, *, args):  # noqa: D102
        with NodeStrategy(args) as node:
            node_names = get_node_names(
                node=node, include_hidden_nodes=args.include_hidden_nodes)

        node_name = args.node_name
        if node_name[0] != '/':
            node_name = '/' + node_name
        if node_name not in [n.full_name for n in node_names]:
            return 'Node not found'

        with DirectNode(args) as node:
            parameter = Parameter()
            Parameter.name = args.name
            value = ParameterValue()
            value.type = ParameterType.PARAMETER_NOT_SET
            parameter.value = value

            response = call_set_parameters(
                node=node, node_name=args.node_name, parameters=[parameter])

            # output response
            assert len(response.results) == 1
            result = response.results[0]
            if result.successful:
                msg = 'Deleted parameter successfully'
                if result.reason:
                    msg += ': ' + result.reason
                print(msg)
            else:
                msg = 'Deleting parameter failed'
                if result.reason:
                    msg += ': ' + result.reason
                print(msg, file=sys.stderr)
