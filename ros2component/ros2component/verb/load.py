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

import rclpy

from ros2cli.node import NODE_NAME_PREFIX
from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy

from ros2component.api import ComponentTypeNameCompleter
from ros2component.api import container_node_name_completer
from ros2component.api import find_container_node_names
from ros2component.api import load_component_into_container
from ros2component.api import package_with_components_name_completer
from ros2component.verb import VerbExtension

from ros2node.api import get_node_names


class LoadVerb(VerbExtension):
    """Load a component into a container node."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        argument = parser.add_argument(
            'container_node_name', help='Container node name to load component into'
        )
        argument.completer = container_node_name_completer
        argument = parser.add_argument(
            'package_name', help='Name of the package where the component is to be found'
        )
        argument.completer = package_with_components_name_completer
        argument = parser.add_argument(
            'plugin_name', help='Type name of the component to be loaded'
        )
        argument.completer = ComponentTypeNameCompleter(package_name_key='package_name')
        parser.add_argument('-n', '--node-name', default=None, help='Component node name')
        parser.add_argument('--node-namespace', default=None, help='Component node namespace')
        parser.add_argument('--log-level', default=None, help='Component node log level')
        parser.add_argument(
            '-r', '--remap-rule', action='append', default=None, dest='remap_rules',
            help="Component node remapping rules, in the 'from:=to' form"
        )
        parser.add_argument(
            '-p', '--parameter', action='append', default=None, dest='parameters',
            help="Component node parameters, in the 'name:=value' form"
        )
        parser.add_argument(
            '-e', '--extra-argument', action='append', default=None, dest='extra_arguments',
            help="Extra arguments for the container, in the 'name:=value' form"
        )

    def main(self, *, args):
        with NodeStrategy(args) as node:
            node_names = get_node_names(node=node)
        with DirectNode(args) as node:
            container_node_names = find_container_node_names(node=node, node_names=node_names)
        rclpy.init()
        node = rclpy.create_node(NODE_NAME_PREFIX + '_component_load_requester')
        try:
            if args.container_node_name in [n.full_name for n in container_node_names]:
                return load_component_into_container(
                    node=node, remote_container_node_name=args.container_node_name,
                    package_name=args.package_name, plugin_name=args.plugin_name,
                    node_name=args.node_name, node_namespace=args.node_namespace,
                    log_level=args.log_level, remap_rules=args.remap_rules,
                    parameters=args.parameters, extra_arguments=args.extra_arguments
                )
            else:
                return "Unable to find container node '" + args.container_node_name + "'"
        finally:
            node.destroy_node()
            rclpy.shutdown()
