# Copyright 2026 Tomoya Fujita, Fumiya Ohnishi
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

from ros2log.api import call_get_logger_levels
from ros2log.api import get_logger_name_for_node
from ros2log.api import get_target_node_names
from ros2log.verb import VerbExtension
from ros2log.verb.levels import LEVEL_VALUE_TO_NAME
from ros2node.api import NodeNameCompleter


class GetVerb(VerbExtension):
    """Get a node's current log level."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        add_arguments(parser)
        parser.add_argument(
            '--all', '-a', action='store_true',
            help='Get log levels for all nodes with logger services enabled')
        arg = parser.add_argument(
            'node_name', nargs='?',
            help='Name of the ROS node')
        arg.completer = NodeNameCompleter()

    def main(self, *, args):  # noqa: D102
        validation_error = _validate_arguments(args)
        if validation_error:
            return validation_error

        with NodeStrategy(args) as node:
            nodes_to_query, error = get_target_node_names(
                node=node,
                node_name=args.node_name,
                all_nodes=args.all,
            )

        if error:
            return error

        if not nodes_to_query:
            return 'No nodes with logger services found'

        logger_names_by_node = {
            node_name: [get_logger_name_for_node(node_name)]
            for node_name in nodes_to_query
        }

        with DirectNode(args) as node:
            levels_by_node = call_get_logger_levels(
                node=node,
                logger_names_by_node=logger_names_by_node,
            )

        had_error = False
        for node_name in nodes_to_query:
            levels = levels_by_node[node_name]
            if isinstance(levels, Exception):
                message = f"Exception while calling service of node '{node_name}': {levels}"
                if args.all:
                    print(message, file=sys.stderr)
                    had_error = True
                    continue
                return message

            if len(levels) != 1:
                message = f"Unexpected response while getting logger level for node '{node_name}'"
                if args.all:
                    print(message, file=sys.stderr)
                    had_error = True
                    continue
                return message

            level_name = LEVEL_VALUE_TO_NAME.get(levels[0].level, str(levels[0].level))
            prefix = f'{node_name}: ' if args.all else ''
            print(prefix + level_name)

        return 1 if had_error else 0


def _validate_arguments(args) -> str | None:
    if args.all and args.node_name is not None:
        return 'Node name cannot be used with --all'
    if not args.all and args.node_name is None:
        return 'Either a node name or --all must be specified'
    return None
