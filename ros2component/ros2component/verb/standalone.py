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

import uuid

from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import add_arguments

from ros2component.api import add_component_arguments
from ros2component.api import load_component_into_container
from ros2component.api import run_standalone_container
from ros2component.verb import VerbExtension


class StandaloneVerb(VerbExtension):
    """Run a component into its own standalone container node."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        add_component_arguments(parser)
        parser.add_argument(
            '-c', '--container-node-name',
            default='standalone_container_' + uuid.uuid4().hex[:12],
            help='Name of the standalone container node to be run'
        )

    def main(self, *, args):
        container = run_standalone_container(container_node_name=args.container_node_name)

        with DirectNode(args) as node:
            load_component_into_container(
                node=node, remote_container_node_name=args.container_node_name,
                package_name=args.package_name, plugin_name=args.plugin_name,
                node_name=args.node_name, node_namespace=args.node_namespace,
                log_level=args.log_level, remap_rules=args.remap_rules,
                parameters=args.parameters, extra_arguments=args.extra_arguments
            )

        while container.returncode is None:
            try:
                container.communicate()
            except KeyboardInterrupt:
                # the subprocess will also receive the signal and should shut down
                # therefore we continue here until the process has finished
                pass
        return container.returncode
