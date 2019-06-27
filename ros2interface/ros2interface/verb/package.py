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

from ros2interface.api import get_interface
from ros2interface.api import package_name_completer
from ros2interface.verb import VerbExtension


class PackageVerb(VerbExtension):
    """Output a list of available interface types within one package."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'package_name',
            help="Name of the ROS package (e.g. 'std_msgs, std_srvs, etc.')")
        arg.completer = package_name_completer

    def main(self, *, args):
        try:
            names = get_interface(args.package_name)
        except LookupError as e:
            return str(e)
        for name in names:
            print('{args.package_name}/{name}'.format_map(locals()))
