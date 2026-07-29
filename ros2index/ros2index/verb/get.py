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

from ament_index_python.resources import get_resource

from ros2index.api import resource_name_completer, resource_type_completer
from ros2index.verb import VerbExtension


class GetVerb(VerbExtension):
    """Output the content of a specific resource in the ament resource index."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument('resource_type', metavar='TYPE', help='The type of the resource')
        arg.completer = resource_type_completer
        arg = parser.add_argument('resource_name', metavar='NAME', help='The name of the resource')
        arg.completer = resource_name_completer

    def main(self, *, args):
        try:
            content, path = get_resource(args.resource_type, args.resource_name)
        except (LookupError, OSError) as e:
            return str(e)
        print(path)
        if content:
            print(content)
