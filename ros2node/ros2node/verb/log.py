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

from ros2node.verb import VerbExtension
from ros2node.api import call_log_level_set, LEVEL_STR_TO_ENUM
from ros2cli.node.direct import DirectNode


class LogVerb(VerbExtension):
    """Set the log level of a node"""
    def add_arguments(self, parser, cli_name):
        add_arguments(parser)

        parser.add_argument('node_name', help='The name of the node')
        parser.add_argument('-l', '--logger-name', help='The logger name, if it is different from the node name')
        parser.add_argument('level', choices=LEVEL_STR_TO_ENUM.keys(), help='Log level')

    def main(self, *, args):
        with DirectNode(args) as node:
            if args.logger_name is None:
                args.logger_name = args.node_name.lstrip('/')
            # Call the service to set the log level
            call_log_level_set(node, args.logger_name, args.level)
