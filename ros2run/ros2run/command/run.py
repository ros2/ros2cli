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

from ros2cli.command import CommandExtension
from ros2pkg.api import package_name_completer
from ros2pkg.api import PackageNotFound
from ros2run.api import ExecutableNameCompleter
from ros2run.api import get_executable_path
from ros2run.api import MultipleExecutables
from ros2run.api import run_executable


class RunCommand(CommandExtension):
    """Run a package specific executable."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'package_name',
            help="Name of the ROS package")
        arg.completer = package_name_completer
        arg = parser.add_argument(
            'executable_name',
            help="Name of the executable")
        arg.completer = ExecutableNameCompleter(
            package_name_key='package_name')
        parser.add_argument(
            'argv', nargs='*',
            help="Pass arbitrary arguments to the executable (use '--' before "
                 "these arguments to ensure they are not handle by this "
                 "command)")

    def main(self, *, parser, args):
        try:
            path = get_executable_path(
                package_name=args.package_name,
                executable_name=args.executable_name)
        except PackageNotFound:
            raise RuntimeError(
                "Package '{args.package_name}' not found"
                .format_map(locals()))
        except MultipleExecutables as e:
            msg = 'Multiple executables found:'
            for p in e.paths:
                msg += '\n- {p}'.format_map(locals())
            raise RuntimeError(msg)
        if path is None:
            return 'No executable found'
        return run_executable(path=path, argv=args.argv)
