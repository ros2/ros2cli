# Copyright 2019 Open Source Robotics Foundation, Inc.
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
from ros2node.api import get_absolute_node_name
from ros2node.api import NodeNameCompleter
from ros2node.api import wait_for_node
from ros2param.api import call_describe_parameters
from ros2param.api import get_parameter_type_string
from ros2param.api import ParameterNameCompleter
from ros2param.verb import VerbExtension


class DescribeVerb(VerbExtension):
    """Show descriptive information about declared parameters."""

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
            'parameter_names', nargs='+', help='Names of the parameters')
        parser.add_argument(
            '--timeout', metavar='N', type=int, default=1,
            help='Wait for N seconds until node becomes available (default %(default)s sec)')
        arg.completer = ParameterNameCompleter()

    def main(self, *, args):  # noqa: D102
        node_name = get_absolute_node_name(args.node_name)
        with NodeStrategy(args) as node:
            if not wait_for_node(node, node_name, args.include_hidden_nodes, args.timeout):
                return 'Node not found'

        with DirectNode(args) as node:
            response = call_describe_parameters(
                node=node, node_name=args.node_name,
                parameter_names=args.parameter_names or None)

            # output response
            for descriptor in response.descriptors:
                self._print_descriptor(descriptor)

    def _print_descriptor(self, descriptor):
        print('Parameter name:', descriptor.name)
        print('  Type:', get_parameter_type_string(descriptor.type))
        if descriptor.description:
            print('  Description:', descriptor.description)
        if True:
            print('  Constraints:')
            if descriptor.read_only:
                print('    Read only: true')
            ranges = descriptor.floating_point_range + descriptor.integer_range
            if ranges:
                range_ = ranges[0]
                print('    Min value:', range_.from_value)
                print('    Max value:', range_.to_value)
                if range_.step:
                    print('    Step:', range_.step)
            if descriptor.additional_constraints:
                print('    Additional constraints:',
                      descriptor.additional_constraints)
