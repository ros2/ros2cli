# Copyright 2022 PickNik Inc.
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

from ros2cli.node.strategy import NodeStrategy
from ros2service.api import get_service_clients_and_servers
from ros2service.api import ServiceNameCompleter
from ros2service.verb import VerbExtension


class InfoVerb(VerbExtension):
    """Print information about a service."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'service_name',
            help="Name of the ROS service to print info about (e.g. '/talker/list_parameters')")
        arg.completer = ServiceNameCompleter(
            include_hidden_services_key='include_hidden_services')
        parser.add_argument(
            '-t', '--show-types', action='store_true',
            help='Additionally show the service type')
        parser.add_argument(
            '-c', '--count', action='store_true',
            help='Only display the number of service clients and service servers')

    def main(self, *, args):
        with NodeStrategy(args) as node:

            service_clients, service_servers = get_service_clients_and_servers(
                node=node,
                service_name=args.service_name
            )

            print('Service: {}'.format(args.service_name))
            print('Service clients: {}'.format(len(service_clients)))
            if not args.count:
                for client_name, client_types in service_clients:
                    if args.show_types:
                        types_formatted = ', '.join(client_types)
                        print(f'    {client_name} [{types_formatted}]')
                    else:
                        print(f'    {client_name}')
            print('Service servers: {}'.format(len(service_servers)))
            if not args.count:
                for server_name, server_types in service_servers:
                    if args.show_types:
                        types_formatted = ', '.join(server_types)
                        print(f'    {server_name} [{types_formatted}]')
                    else:
                        print(f'    {server_name}')
