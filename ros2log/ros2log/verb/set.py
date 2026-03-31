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
from typing import Optional

from rcl_interfaces.msg import LoggerLevel

from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy

from ros2log.api import call_set_logger_levels
from ros2log.api import get_logger_name_for_node
from ros2log.api import get_target_node_names
from ros2log.verb import VerbExtension
from ros2log.verb.levels import LEVEL_NAME_TO_VALUE
from ros2node.api import NodeNameCompleter


class SetVerb(VerbExtension):
    """Set a node's log level."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        add_arguments(parser)
        parser.add_argument(
            '--all', '-a', action='store_true',
            help='Set log levels for all nodes with logger services enabled')
        arg = parser.add_argument(
            'node_name', nargs='?',
            help='Name of the ROS node')
        arg.completer = NodeNameCompleter()
        parser.add_argument(
            'level',
            type=str.upper,
            choices=list(LEVEL_NAME_TO_VALUE.keys()),
            help='The log level to set')

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

        requested_level = LEVEL_NAME_TO_VALUE[args.level]
        levels_by_node = {}
        for node_name in nodes_to_query:
            logger_level = LoggerLevel()
            logger_level.name = get_logger_name_for_node(node_name)
            logger_level.level = requested_level
            levels_by_node[node_name] = [logger_level]

        with DirectNode(args) as node:
            results_by_node = call_set_logger_levels(
                node=node,
                levels_by_node=levels_by_node,
            )

        had_error = False
        for node_name in nodes_to_query:
            results = results_by_node[node_name]
            if isinstance(results, Exception):
                message = f"Exception while calling service of node '{node_name}': {results}"
                if args.all:
                    print(message, file=sys.stderr)
                    had_error = True
                    continue
                return message

            if len(results) != 1:
                message = f"Unexpected response while setting logger level for node '{node_name}'"
                if args.all:
                    print(message, file=sys.stderr)
                    had_error = True
                    continue
                return message

            result = results[0]
            message = 'Set logger level successful'
            if not result.successful:
                message = 'Setting logger level failed'
            if result.reason:
                message += ': ' + result.reason

            prefix = f'{node_name}: ' if args.all else ''
            if result.successful:
                print(prefix + message)
            else:
                print(prefix + message, file=sys.stderr)
                had_error = True

        return 1 if had_error else 0


def _validate_arguments(args) -> Optional[str]:
    if args.all and args.node_name is not None:
        return 'Node name cannot be used with --all'
    if not args.all and args.node_name is None:
        return 'Either a node name or --all must be specified'
    return None
