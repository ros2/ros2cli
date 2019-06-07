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
from ros2service.api import ServiceNameCompleter
from ros2service.verb import VerbExtension


class TypeVerb(VerbExtension):
    """Output a service's type."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'service_name',
            help="Name of the ROS service to get type (e.g. '/talker/list_parameters')")
        arg.completer = ServiceNameCompleter(
            include_hidden_services_key='include_hidden_services')

    def main(self, *, args):
        with NodeStrategy(args) as node:
            service_names_and_types = get_service_names_and_types(
                node=node,
                include_hidden_services=args.include_hidden_services)

        for (service_name, service_types) in service_names_and_types:
            if args.service_name == service_name:
                print(service_types[0])
                return 0

        return 1
