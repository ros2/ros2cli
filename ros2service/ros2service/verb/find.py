# Copyright 2019 Canonical Ltd.
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
from ros2service.api import get_service_names_and_types
from ros2service.verb import VerbExtension


class FindVerb(VerbExtension):
    """Output a list of available services of a given type."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'service_type',
            help="Name of the ROS service type to filter for (e.g. 'rcl_interfaces/srv/ListParameters')")
        parser.add_argument(
            '-c', '--count-services', action='store_true',
            help='Only display the number of services discovered')
        # duplicate the following argument from the command for visibility
        parser.add_argument(
            '--include-hidden-services', action='store_true',
            help='Consider hidden services as well')

    def main(self, *, args):
        with NodeStrategy(args) as node:
            service_names_and_types = get_service_names_and_types(
                node=node,
                include_hidden_services=args.include_hidden_services)

        filtered_services = []
        for (service_name, service_type) in service_names_and_types:
            if args.service_type in service_type:
                filtered_services.append(service_name)

        if args.count_services:
            print(len(filtered_services))
        else:
            for filtered_service in filtered_services:
                print(filtered_service)
