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

from ros2interface.api import package_name_completer
from ros2interface.verb import VerbExtension
from rosidl_runtime_py import get_interfaces


class PackageVerb(VerbExtension):
    """Output a list of available interface types within one package."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'package_name',
            help="Name of the ROS package (e.g. 'example_interfaces')")
        arg.completer = package_name_completer

    def main(self, *, args):
        try:
            interfaces = get_interfaces([args.package_name])
        except LookupError as e:
            return str(e)
        for package_name in sorted(interfaces):
            for interface_name in interfaces[package_name]:
                print(f'{package_name}/{interface_name}')
