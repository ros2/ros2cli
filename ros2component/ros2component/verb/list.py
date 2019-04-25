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

from ros2component.api import container_node_name_completer
from ros2component.api import find_container_node_names
from ros2component.api import get_container_components_info
from ros2component.verb import VerbExtension

from ros2node.api import get_node_names


class ListVerb(VerbExtension):
    """Output a list of running containers and components."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        argument = parser.add_argument(
            'container_node_name', nargs='?', default=None,
            help='Name of the container node to list components from')
        argument.completer = container_node_name_completer
        parser.add_argument(
            '--containers-only', action='store_true',
            help='List found containers nodes only')

    def main(self, *, args):
        with NodeStrategy(args) as node:
            node_names = get_node_names(node=node)
        with DirectNode(args) as node:
            container_node_names = find_container_node_names(
                node=node, node_names=node_names
            )
            if args.container_node_name is not None:
                if args.container_node_name not in [n.full_name for n in container_node_names]:
                    return "Unable to find container node '" + args.container_node_name + "'"
                if not args.containers_only:
                    components = get_container_components_info(
                        node=node, remote_container_node_name=args.container_node_name
                    )
                    if any(components):
                        print(*['{}    {}'.format(c.uid, c.name) for c in components], sep='\n')
            else:
                for n in container_node_names:
                    print(n.full_name)
                    if not args.containers_only:
                        components = get_container_components_info(
                            node=node, remote_container_node_name=n.full_name
                        )
                        if any(components):
                            print(*[
                                4 * ' ' + '{}    {}'.format(c.uid, c.name) for c in components
                            ], sep='\n')
