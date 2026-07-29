# Copyright 2026 Thomas Ung
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

from ament_index_python.resources import get_resources

from ros2index.api import resource_type_completer
from ros2index.verb import VerbExtension


class ListVerb(VerbExtension):
    """Output the resources of a given type in the ament resource index."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument('resource_type', metavar='TYPE', help='The type of the resource')
        arg.completer = resource_type_completer

    def main(self, *, args):
        resources = get_resources(args.resource_type)
        for resource_name in sorted(resources.keys()):
            print(f'{resource_name}\t{resources[resource_name]}')
