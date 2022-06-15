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

import re
import sys

from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2node.api import get_absolute_node_name
from ros2node.api import get_node_names
from ros2node.api import NodeNameCompleter

from ros2param.api import call_describe_parameters
from ros2param.api import call_list_parameters
from ros2param.api import get_parameter_type_string
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
            '--filter', nargs=1,
            help=(
                'Only parameters matching the regex expression will be showed.'
                ' Supports `re` regex syntax.'))
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

        regex_filter = getattr(args, 'filter')
        if regex_filter is not None:
            regex_filter = re.compile(regex_filter[0])

        with DirectNode(args) as node:
            responses = {}
            for node_name in node_names:
                responses[node_name] = call_list_parameters(node=node,
                                                            node_name=node_name.full_name,
                                                            prefixes=args.param_prefixes)
            # print responses
            for node_name in sorted(responses.keys()):
                response = responses[node_name]
                if response is None:
                    e = response.exception()
                    print(
                        'Exception while calling service of node '
                        f"'{node_name.full_name}': {e}", file=sys.stderr)
                    continue
                sorted_names = sorted(response)
                if regex_filter is not None:
                    sorted_names = [name for name in sorted_names if regex_filter.match(name)]
                if not args.node_name and sorted_names:
                    print(f'{node_name.full_name}:')
                # get descriptors for the node if needs to print parameter type
                name_to_type_map = {}
                if args.param_type is True:
                    resp = call_describe_parameters(
                        node=node, node_name=node_name.full_name,
                        parameter_names=sorted_names)
                    for descriptor in resp.descriptors:
                        name_to_type_map[descriptor.name] = get_parameter_type_string(
                            descriptor.type)

                for name in sorted_names:
                    if args.param_type is True:
                        param_type_str = name_to_type_map[name]
                        print(f'  {name} (type: {param_type_str})')
                    else:
                        print(f'  {name}')
