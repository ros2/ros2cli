# Copyright 2026 Tim Wendt
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
from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2node.api import get_node_names
from ros2node.api import has_duplicates
from ros2param.api import call_get_parameters
from ros2param.verb import VerbExtension


class CheckSimTimeVerb(VerbExtension):
    """Check for all node the use_sim_time parameter."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        add_arguments(parser)
        parser.add_argument(
            '--include-hidden-nodes',
            action='store_true',
            help='Consider hidden nodes as well',
        )
        parser.add_argument(
            '--per-node-timeout', metavar='N', type=float, default=5.0,
            help=(
                'Maximum wait time per node for list_parameters service call '
                'in seconds (default: %(default)s)'))

    def main(self, *, args):  # noqa: D102
        with NodeStrategy(args) as node:
            node_names = get_node_names(
                node=node, include_hidden_nodes=args.include_hidden_nodes
            )

        sorted_names = sorted(n.full_name for n in node_names)
        if has_duplicates(sorted_names):
            print(
                'WARNING: Multiple nodes in the graph share the same exact name. '
                'This can hide the actual use_sim_time value.',
                file=sys.stderr,
            )

        if len(node_names) == 0:
            return 'No nodes found'

        items = []
        with DirectNode(args) as node:
            for node_name in sorted_names:
                try:
                    response = call_get_parameters(
                        node=node,
                        node_name=node_name,
                        parameter_names=['use_sim_time'],
                        spin_timeout_sec=args.per_node_timeout,
                    )

                    # parameter not set
                    if not response.values:
                        items.append((node_name, '<not declared>'))
                        continue

                    pvalue = response.values[0]

                    if pvalue.type == ParameterType.PARAMETER_BOOL:
                        items.append((node_name, str(pvalue.bool_value)))
                        continue
                    elif pvalue.type == ParameterType.PARAMETER_NOT_SET:
                        items.append((node_name, '<not set>'))
                        continue
                    else:
                        items.append((node_name, f'<unexpected type {pvalue.type}>'))
                        continue

                except Exception as e:
                    items.append((node_name, f'<error: {e}>'))

        print('use_sim_time:')
        pad = max((len(k) for k, _ in items), default=0)
        for k, v in items:
            print(f'{k:<{pad}}: {v}')
