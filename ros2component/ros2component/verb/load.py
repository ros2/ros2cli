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

from ros2component.api import add_component_arguments
from ros2component.api import container_node_name_completer
from ros2component.api import find_container_node_names
from ros2component.api import load_component_into_container
from ros2component.verb import VerbExtension

from ros2node.api import get_node_names


class LoadVerb(VerbExtension):
    """Load a component into a container node."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        argument = parser.add_argument(
            'container_node_name', help='Container node name to unload component from'
        )
        argument.completer = container_node_name_completer
        add_component_arguments(parser)

    def main(self, *, args):
        with NodeStrategy(args) as node:
            node_names = get_node_names(node=node)
        with DirectNode(args) as node:
            container_node_names = find_container_node_names(node=node, node_names=node_names)
            if args.container_node_name not in [n.full_name for n in container_node_names]:
                return "Unable to find container node '" + args.container_node_name + "'"
            component_uid, component_name = load_component_into_container(
                node=node, remote_container_node_name=args.container_node_name,
                package_name=args.package_name, plugin_name=args.plugin_name,
                node_name=args.node_name, node_namespace=args.node_namespace,
                log_level=args.log_level, remap_rules=args.remap_rules,
                parameters=args.parameters, extra_arguments=args.extra_arguments
            )
            print('{}  {}'.format(component_uid, component_name))
