# Copyright 2017 Open Source Robotics Foundation, Inc.
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

from ros2cli.node.strategy import add_arguments as add_strategy_node_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2service.api import get_service_names_and_types
from ros2service.verb import VerbExtension


class ListVerb(VerbExtension):
    """Output a list of available services."""

    def add_arguments(self, parser, cli_name):
        add_strategy_node_arguments(parser)
        parser.add_argument(
            '-t', '--show-types', action='store_true',
            help='Additionally show the service type')
        parser.add_argument(
            '-c', '--count-services', action='store_true',
            help='Only display the number of services discovered')

    def main(self, *, args):
        with NodeStrategy(args) as node:
            service_names_and_types = get_service_names_and_types(
                node=node,
                include_hidden_services=args.include_hidden_services)

        if args.count_services:
            print(len(service_names_and_types))
        elif service_names_and_types:
            for (service_name, service_types) in service_names_and_types:
                msg = f'{service_name}'
                service_types_formatted = ', '.join(service_types)
                if args.show_types:
                    msg += f' [{service_types_formatted}]'
                print(msg)
