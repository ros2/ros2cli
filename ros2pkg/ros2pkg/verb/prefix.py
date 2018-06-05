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

from ament_index_python import get_package_share_directory
from ament_index_python import PackageNotFoundError
from ros2pkg.api import get_prefix_path
from ros2pkg.api import package_name_completer
from ros2pkg.verb import VerbExtension

PACKAGE_NOT_FOUND = 'Package not found'


class PrefixVerb(VerbExtension):
    """Output the prefix path of a package."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'package_name',
            help='The package name')
        parser.add_argument(
            '--share',
            action='store_true',
            help='Show share directory for the package')
        arg.completer = package_name_completer

    def main(self, *, args):
        if not args.share:
            prefix_path = get_prefix_path(args.package_name)
            if prefix_path is None:
                return PACKAGE_NOT_FOUND
            print(prefix_path)
        else:
            try:
                print(get_package_share_directory(args.package_name))
            except PackageNotFoundError:
                return PACKAGE_NOT_FOUND
