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

from rcl_interfaces.msg import ParameterType
from ros2cli.helpers import interactive_select
from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2node.api import get_absolute_node_name
from ros2node.api import get_node_names
from ros2node.api import NodeNameCompleter
from ros2node.api import wait_for_node
from ros2param.api import call_get_parameters
from ros2param.api import call_list_parameters
from ros2param.api import ParameterNameCompleter
from ros2param.verb import VerbExtension


class GetVerb(VerbExtension):
    """Get parameter."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        add_arguments(parser)
        arg = parser.add_argument(
            'node_name', nargs='?',
            help='Name of the ROS node (optional). '
                 'If only one argument is provided, it is treated as a parameter name '
                 'to query across all nodes.')
        arg.completer = NodeNameCompleter(
            include_hidden_nodes_key='include_hidden_nodes')
        parser.add_argument(
            '--include-hidden-nodes', action='store_true',
            help='Consider hidden nodes as well')
        arg = parser.add_argument(
            'parameter_name', nargs='*',
            help='Name of the parameter(s) (optional). '
                 'Multiple parameter names can be specified when a node name is given. '
                 'If neither node nor parameter is provided, interactive selection is used.')
        arg.completer = ParameterNameCompleter()
        parser.add_argument(
            '--hide-type', action='store_true',
            help='Hide the type information')
        parser.add_argument(
            '--timeout', metavar='N', type=int, default=1,
            help='Wait for N seconds until node becomes available (default %(default)s sec)')
        parser.add_argument(
            '--service-timeout', metavar='N', type=float,
            help='Maximum time to wait for service response in seconds '
                 '(default: waits indefinitely)')

    def main(self, *, args):  # noqa: D102
        # If both node and parameter are None/empty, use interactive selection for both
        if args.node_name is None and not args.parameter_name:
            with NodeStrategy(args) as node:
                node_names = get_node_names(
                    node=node,
                    include_hidden_nodes=args.include_hidden_nodes)
                node_name_list = [n.full_name for n in node_names]

                if not node_name_list:
                    return 'No nodes available to select from.'

                selected_node = interactive_select(
                    node_name_list,
                    prompt='Select node:')

                if selected_node is None:
                    return 'No node selected'

                args.node_name = selected_node

            # Now select parameter from the chosen node
            with DirectNode(args) as node:
                response = call_list_parameters(
                    node=node, node_name=args.node_name)

                if response is None:
                    return 'Unable to get parameters: service call timed out.'

                if response.result() is None:
                    return 'Unable to get parameters: service call failed.'

                parameter_names = response.result().result.names

                if not parameter_names:
                    return 'No parameters available to select from.'

                selected_param = interactive_select(
                    parameter_names,
                    prompt='Select parameter:')

                if selected_param is None:
                    return None

                args.parameter_name = [selected_param]

        # If only one argument provided, treat it as parameter name (query all nodes)
        elif args.node_name is not None and not args.parameter_name:
            args.parameter_name = [args.node_name]
            args.node_name = None

        # Determine which nodes to query
        if args.node_name:
            # Single node mode (existing behavior)
            node_name = get_absolute_node_name(args.node_name)
            with NodeStrategy(args) as node:
                if not wait_for_node(node, node_name, args.include_hidden_nodes, args.timeout):
                    return 'Node not found'
            nodes_to_query = [(node_name, args.node_name)]
        else:
            # Query all nodes mode (new feature)
            with NodeStrategy(args) as node:
                node_names = get_node_names(
                    node=node, include_hidden_nodes=args.include_hidden_nodes)
            nodes_to_query = [
                (n.full_name, n.full_name)
                for n in sorted(node_names, key=lambda x: x.full_name)
            ]

            if not nodes_to_query:
                return 'No nodes found'

        with DirectNode(args) as node:
            found_any = False
            # In multi-node mode only one parameter is supported; use the first element
            multi_param = len(args.parameter_name) > 1
            for node_name, node_name_arg in nodes_to_query:
                try:
                    response = call_get_parameters(
                        node=node, node_name=node_name_arg,
                        parameter_names=args.parameter_name,
                        timeout=args.service_timeout)

                    # requested parameter(s) not available
                    if not response.values:
                        # In multi-node mode, skip nodes without the parameter
                        if not args.node_name:
                            continue
                        param_names_str = ', '.join(
                            f"'{p}'" for p in args.parameter_name)
                        return f'Parameter(s) {param_names_str} not available on node'

                    found_any = True

                    # Process each parameter value in the response
                    for param_idx, pvalue in enumerate(response.values):
                        queried_param_name = args.parameter_name[param_idx]

                        # extract type specific value
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
                            value = pvalue.integer_array_value.tolist()
                        elif pvalue.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                            label = 'Double values are:'
                            value = pvalue.double_array_value.tolist()
                        elif pvalue.type == ParameterType.PARAMETER_STRING_ARRAY:
                            label = 'String values are:'
                            value = pvalue.string_array_value
                        elif pvalue.type == ParameterType.PARAMETER_NOT_SET:
                            label = 'Parameter not set.'
                            value = None
                        else:
                            if not args.node_name:
                                print(
                                    f"{node_name}: Unknown parameter type '{pvalue.type}'",
                                    file=sys.stderr)
                                continue
                            print(
                                f"Unknown parameter type '{pvalue.type}' "
                                f"for parameter '{queried_param_name}'",
                                file=sys.stderr)
                            continue

                        # output response
                        if not args.node_name:
                            # Multi-node mode: prefix with node name
                            prefix = f'{node_name}:\n  '
                            if not args.hide_type:
                                if value is not None:
                                    print(prefix + label, value)
                                else:
                                    print(prefix + label)
                            else:
                                print(f'{node_name}: {value}')
                        else:
                            # Single node mode: prefix with param name when multiple params
                            if multi_param:
                                prefix = f'{queried_param_name}:\n  '
                            else:
                                prefix = ''
                            if not args.hide_type:
                                if value is not None:
                                    print(prefix + label, value)
                                else:
                                    print(prefix + label)
                            else:
                                if multi_param:
                                    print(f'{queried_param_name}: {value}')
                                else:
                                    print(value)
                except Exception as e:
                    if not args.node_name:
                        print(f'{node_name}: Error: {e}', file=sys.stderr)
                        continue
                    else:
                        raise

            # If we're in multi-node mode and found no parameters
            if not args.node_name and not found_any:
                print(f"Parameter '{args.parameter_name[0]}' not set on any node")
