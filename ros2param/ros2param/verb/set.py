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
from ros2param.api import call_set_parameters_atomically
from ros2param.api import ParameterNameCompleter
from ros2param.verb import VerbExtension


class _NameValueCompleter:
    """
    Completer for interleaved name/value pairs.

    Delegates to ParameterNameCompleter for name positions (even index)
    and returns no completions for value positions (odd index).
    """

    def __call__(self, prefix, parsed_args, **kwargs):
        already = getattr(parsed_args, 'params_and_values', None) or []
        # Even count of already-collected tokens → completing a *name*
        if len(already) % 2 == 0:
            return ParameterNameCompleter()(prefix, parsed_args, **kwargs)
        # Odd count → completing a *value*; no suggestions
        return []


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
            'params_and_values', nargs='+',
            metavar='name value',
            help='Parameter name and value pair(s): name value [name value ...]. '
                 'Provide one or more name/value pairs to set multiple parameters at once.')
        arg.completer = _NameValueCompleter()
        parser.add_argument(
            '--atomic', action='store_true',
            help='Set all parameters atomically using the SetParametersAtomically service. '
                 'Either all parameters are set successfully or none are changed.')
        parser.add_argument(
            '--timeout', metavar='N', type=int, default=1,
            help='Wait for N seconds until node becomes available (default %(default)s sec)')

    def main(self, *, args):  # noqa: D102
        # Validate that parameters were provided as name/value pairs
        if len(args.params_and_values) % 2 != 0:
            return (
                'Parameters must be provided as name/value pairs: '
                'name value [name value ...]. '
                f'Got {len(args.params_and_values)} argument(s).'
            )

        pairs = list(zip(args.params_and_values[::2], args.params_and_values[1::2]))
        multi_param = len(pairs) > 1

        node_name = get_absolute_node_name(args.node_name)
        with NodeStrategy(args) as node:
            if not wait_for_node(node, node_name, args.include_hidden_nodes, args.timeout):
                return 'Node not found'

        with DirectNode(args) as node:
            parameters = []
            for param_name, param_value_str in pairs:
                parameter = Parameter()
                parameter.name = param_name
                parameter.value = get_parameter_value(string_value=param_value_str)
                parameters.append(parameter)

            if args.atomic:
                # Set all parameters in a single atomic transaction
                response = call_set_parameters_atomically(
                    node=node, node_name=args.node_name, parameters=parameters)

                result = response.result
                if result.successful:
                    msg = 'Set parameters atomically successful'
                    if result.reason:
                        msg += ': ' + result.reason
                    print(msg)
                else:
                    msg = 'Setting parameters atomically failed'
                    if result.reason:
                        msg += ': ' + result.reason
                    print(msg, file=sys.stderr)
            else:
                response = call_set_parameters(
                    node=node, node_name=args.node_name, parameters=parameters)

                # output response
                assert len(response.results) == len(parameters)
                for (param_name, _), result in zip(pairs, response.results):
                    if result.successful:
                        msg = 'Set parameter successful'
                        if result.reason:
                            msg += ': ' + result.reason
                        if multi_param:
                            print(f'{param_name}: {msg}')
                        else:
                            print(msg)
                    else:
                        msg = 'Setting parameter failed'
                        if result.reason:
                            msg += ': ' + result.reason
                        if multi_param:
                            print(f'{param_name}: {msg}', file=sys.stderr)
                        else:
                            print(msg, file=sys.stderr)
