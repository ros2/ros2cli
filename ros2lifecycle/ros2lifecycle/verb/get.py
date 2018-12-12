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

from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2lifecycle.api import call_get_states
from ros2lifecycle.api import get_node_names
from ros2lifecycle.verb import VerbExtension
from ros2node.api import NodeNameCompleter


class GetVerb(VerbExtension):
    """Get lifecycle state."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        add_arguments(parser)
        arg = parser.add_argument(
            'node_name', nargs='?', help='Name of the ROS node')
        arg.completer = NodeNameCompleter(
            include_hidden_nodes_key='include_hidden_nodes')
        parser.add_argument(
            '--include-hidden-nodes', action='store_true',
            help='Consider hidden nodes as well')

    def main(self, *, args):  # noqa: D102
        with NodeStrategy(args) as node:
            node_names = get_node_names(
                node=node, include_hidden_nodes=args.include_hidden_nodes)
            node_names = [n.full_name for n in node_names]

        if args.node_name:
            node_name = args.node_name
            if node_name[0] != '/':
                node_name = '/' + node_name
            if node_name not in node_names:
                return 'Node not found'
            node_names = [node_name]

        with DirectNode(args) as node:
            states = call_get_states(node=node, node_names=node_names)

            # output exceptions
            for node_name in sorted(states.keys()):
                state = states[node_name]
                if isinstance(state, Exception):
                    print(
                        'Exception while calling service of node '
                        "'{node_name}': {state}".format_map(locals()),
                        file=sys.stderr)
                    del states[node_name]
                    if args.node_name:
                        return 1

            # output current states
            for node_name in sorted(states.keys()):
                state = states[node_name]
                prefix = ''
                if not args.node_name:
                    prefix = '{node_name}: '.format_map(locals())
                print(
                    prefix + '{state.label} [{state.id}]'.format_map(locals()))
