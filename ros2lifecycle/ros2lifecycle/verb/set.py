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

from ros2lifecycle.api import call_change_states
from ros2lifecycle.api import call_get_available_transitions
from ros2lifecycle.api import get_node_names
from ros2lifecycle.verb import VerbExtension

from ros2node.api import get_absolute_node_name
from ros2node.api import NodeNameCompleter


class SetVerb(VerbExtension):
    """Trigger lifecycle state transition."""

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
            'transition', help='The lifecycle transition')

    def main(self, *, args):  # noqa: D102
        with NodeStrategy(args) as node:
            node_names = get_node_names(
                node=node, include_hidden_nodes=args.include_hidden_nodes)

        node_name = get_absolute_node_name(args.node_name)
        if node_name not in {n.full_name for n in node_names}:
            return 'Node not found'

        with DirectNode(args) as node:
            transitions = call_get_available_transitions(
                node=node, states={node_name: None})
            transitions = transitions[node_name]
            if isinstance(transitions, Exception):
                return 'Exception while calling service of node ' \
                    f"'{args.node_name}': {transitions}"

            # identify requested transition
            for transition in [t.transition for t in transitions]:
                if transition.label == args.transition:
                    break
            else:
                for transition in [t.transition for t in transitions]:
                    if str(transition.id) == args.transition:
                        break
                else:
                    return \
                        'Unknown transition requested, available ones are:' + \
                        ''.join(
                            f'\n- {t.transition.label} [{t.transition.id}]'
                            for t in transitions)

            results = call_change_states(
                node=node, transitions={node_name: transition})
            result = results[node_name]

            # output response
            if isinstance(result, Exception):
                print(
                    'Exception while calling service of node '
                    f"'{args.node_name}': {result}", file=sys.stderr)
            elif result:
                print('Transitioning successful')
            else:
                print('Transitioning failed', file=sys.stderr)
