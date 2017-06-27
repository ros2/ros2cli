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

from ros2pkg.api import get_executable_paths
from ros2pkg.api import get_package_names
from ros2pkg.api import package_name_completer
from ros2pkg.api import PackageNotFound
from ros2pkg.verb import VerbExtension


class ExecutablesVerb(VerbExtension):
    """Output a list of package specific executables."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'package_name', nargs='?',
            help='The package name')
        arg.completer = package_name_completer

    def main(self, *, args):
        if args.package_name is None:
            package_names = get_package_names()
        else:
            package_names = [args.package_name]

        for package_name in sorted(package_names):
            try:
                paths = get_executable_paths(package_name=package_name)
            except PackageNotFound:
                if args.package_name is None:
                    assert False, 'This should never happen'
                raise RuntimeError(
                    "Package '{args.package_name}' not found"
                    .format_map(locals()))
            for path in sorted(paths):
                print(path)
