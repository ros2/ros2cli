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

from rcl_interfaces.msg import ParameterType
from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2node.api import get_node_names
from ros2node.api import NodeNameCompleter
from ros2param.api import call_get_parameters
from ros2param.verb import VerbExtension


class GetVerb(VerbExtension):
    """Get parameter."""

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
        parser.add_argument(
            '--hide-type', action='store_true',
            help='Hide the type information')

    def main(self, *, args):  # noqa: D102
        with NodeStrategy(args) as node:
            node_names = get_node_names(
                node=node, include_hidden_nodes=args.include_hidden_nodes)

        node_name = args.node_name
        if node_name[0] != '/':
            node_name = '/' + node_name
        if node_name not in {n.full_name for n in node_names}:
            return 'Node not found'

        with DirectNode(args) as node:
            response = call_get_parameters(
                node=node, node_name=args.node_name,
                parameter_names=[args.name])

            assert len(response.values) <= 1

            # requested parameter not set
            if not response.values:
                return 'Parameter not set'

            # extract type specific value
            pvalue = response.values[0]
            if pvalue.type == ParameterType.PARAMETER_BOOL:
                label = 'Boolean value is:'
                value = pvalue.bool_value
            elif pvalue.type == ParameterType.PARAMETER_INTEGER:
                label = 'Integer value is:'
                value = pvalue.integer_value
            elif pvalue.type == ParameterType.PARAMETER_DOUBLE:
                label = 'Double value is:'
                value = pvalue.double_value
            elif pvalue.type == ParameterType.PARAMETER_STRING:
                label = 'String value is:'
                value = pvalue.string_value
            elif pvalue.type == ParameterType.PARAMETER_BYTE_ARRAY:
                label = 'Byte values are:'
                value = pvalue.byte_array_value
            elif pvalue.type == ParameterType.PARAMETER_BOOL_ARRAY:
                label = 'Boolean values are:'
                value = pvalue.bool_array_value
            elif pvalue.type == ParameterType.PARAMETER_INTEGER_ARRAY:
                label = 'Integer values are:'
                value = pvalue.integer_array_value
            elif pvalue.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                label = 'Double values are:'
                value = pvalue.double_array_value
            elif pvalue.type == ParameterType.PARAMETER_STRING_ARRAY:
                label = 'String values are:'
                value = pvalue.string_array_value
            elif pvalue.type == ParameterType.PARAMETER_NOT_SET:
                label = 'Parameter not set.'
                value = None
            else:
                return "Unknown parameter type '{pvalue.type}'" \
                    .format_map(locals())

            # output response
            if not args.hide_type:
                print(label, value) if value is not None else print(label)
            else:
                print(value)
