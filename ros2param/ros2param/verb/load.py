# Copyright 2021 Open Source Robotics Foundation, Inc.
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

from rclpy.parameter import parameter_dict_from_yaml_file
from ros2cli.helpers import wait_for
from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2node.api import get_absolute_node_name
from ros2node.api import get_node_names
from ros2node.api import NodeNameCompleter
from ros2node.api import wait_for_node
from ros2param.api import load_parameter_file
from ros2param.verb import VerbExtension


def _parameter_file_matches_node(parameter_file, node_name, use_wildcard):
    try:
        parameter_dict_from_yaml_file(
            parameter_file, use_wildcard, target_nodes=[node_name])
    except RuntimeError:
        # The complete file is validated before this helper is used. For a
        # validated file, rclpy raises RuntimeError here when the selected node
        # contributes no parameters.
        return False
    return True


def _get_matching_node_names(node, parameter_file, use_wildcard, include_hidden_nodes):
    discovered_nodes = get_node_names(
        node=node, include_hidden_nodes=include_hidden_nodes)
    local_node_name = None
    if node.daemon_node is None:
        local_node_name = node.direct_node.get_fully_qualified_name()

    return [
        node_name
        for node_name in sorted({n.full_name for n in discovered_nodes})
        if node_name != local_node_name
        and _parameter_file_matches_node(parameter_file, node_name, use_wildcard)
    ]


class LoadVerb(VerbExtension):
    """Load a parameter file for one node or all matching nodes."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        add_arguments(parser)
        arg = parser.add_argument(
            'node_name', nargs='?', help='Name of the ROS node (omit to load all matching nodes)')
        arg.completer = NodeNameCompleter(
            include_hidden_nodes_key='include_hidden_nodes')
        parser.add_argument(
            '--include-hidden-nodes', action='store_true',
            help='Consider hidden nodes as well')
        parser.add_argument(
            'parameter_file', help='Parameter file')
        parser.add_argument(
            '--no-use-wildcard', action='store_true',
            help="Do not load parameters in the '/**' namespace into the node")
        parser.add_argument(
            '--timeout', metavar='N', type=int, default=1,
            help='Wait for N seconds until node becomes available (default %(default)s sec)')
        parser.add_argument(
            '--service-timeout', metavar='N', type=float,
            help='Maximum time to wait for service response in seconds '
                 '(default: waits indefinitely)')

    def main(self, *, args):  # noqa: D102
        use_wildcard = not args.no_use_wildcard

        if args.node_name is not None:
            node_name = get_absolute_node_name(args.node_name)
            with NodeStrategy(args) as node:
                if not wait_for_node(node, node_name, args.include_hidden_nodes, args.timeout):
                    return 'Node not found'

            with DirectNode(args) as node:
                load_parameter_file(
                    node=node, node_name=node_name, parameter_file=args.parameter_file,
                    use_wildcard=use_wildcard, timeout=args.service_timeout)
            return

        # Validate the complete file before per-node matching. This keeps malformed
        # file errors distinct from an otherwise-valid file not selecting a node.
        parameter_dict_from_yaml_file(args.parameter_file, use_wildcard)

        with NodeStrategy(args) as node:
            matching_node_names = []

            def matching_node_available():
                nonlocal matching_node_names
                matching_node_names = _get_matching_node_names(
                    node,
                    args.parameter_file,
                    use_wildcard,
                    args.include_hidden_nodes,
                )
                return bool(matching_node_names)

            if not wait_for(matching_node_available, args.timeout):
                return 'No matching nodes found'

        with DirectNode(args) as node:
            for node_name in matching_node_names:
                load_parameter_file(
                    node=node, node_name=node_name, parameter_file=args.parameter_file,
                    use_wildcard=use_wildcard, timeout=args.service_timeout)
