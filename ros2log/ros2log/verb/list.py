# Copyright 2025 Tomoya Fujita, Fumiya Ohnishi
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

from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2log.api import iter_logger_service_nodes
from ros2log.verb import VerbExtension


class ListVerb(VerbExtension):
    """Output a list of nodes with logger services enabled."""

    def add_arguments(self, parser, cli_name):
        """Add CLI arguments for the list verb."""
        add_arguments(parser)

    def main(self, *, args):
        """Execute the list verb."""
        with NodeStrategy(args) as node:
            for node_name in iter_logger_service_nodes(node=node):
                print(node_name.full_name)
        return 0
