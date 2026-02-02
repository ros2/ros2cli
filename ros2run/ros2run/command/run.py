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

from argparse import REMAINDER
import os
import shlex

from ros2cli.command import CommandExtension
from ros2cli.helpers import interactive_select
from ros2pkg.api import get_executable_paths
from ros2pkg.api import get_package_names
from ros2pkg.api import package_name_completer
from ros2pkg.api import PackageNotFound
from ros2run.api import ExecutableNameCompleter
from ros2run.api import get_executable_path
from ros2run.api import MultipleExecutables
from ros2run.api import run_executable


class RunCommand(CommandExtension):
    """Run a package specific executable."""

    def add_arguments(self, parser, cli_name):
        try:
            from argcomplete.completers import SuppressCompleter
        except ImportError:
            SuppressCompleter = None

        arg = parser.add_argument(
            '--prefix',
            help='Prefix command, which should go before the executable. '
                 'Command must be wrapped in quotes if it contains spaces '
                 "(e.g. --prefix 'gdb -ex run --args').")
        if SuppressCompleter:
            arg.completer = SuppressCompleter()
        arg = parser.add_argument(
            'package_name', nargs='?',
            help='Name of the ROS package '
                 '(optional, interactive selection if not provided)')
        arg.completer = package_name_completer
        arg = parser.add_argument(
            'executable_name', nargs='?',
            help='Name of the executable '
                 '(optional, interactive selection if not provided)')
        arg.completer = ExecutableNameCompleter(
            package_name_key='package_name')
        arg = parser.add_argument(
            'argv', nargs=REMAINDER,
            help='Pass arbitrary arguments to the executable')
        if SuppressCompleter:
            arg.completer = SuppressCompleter()

    def main(self, *, parser, args):
        # If package not provided, use interactive selection
        if args.package_name is None:
            package_names = sorted(get_package_names())
            if not package_names:
                return 'No packages found'
            selected_package = interactive_select(
                package_names,
                prompt='Select package:')
            if selected_package is None:
                return 'No package selected'
            args.package_name = selected_package
        # If executable not provided, use interactive selection
        if args.executable_name is None:
            try:
                executable_paths = get_executable_paths(
                    package_name=args.package_name)
            except PackageNotFound:
                raise RuntimeError(
                    f"Package '{args.package_name}' not found")
            if not executable_paths:
                return (f'No executables found in package '
                        f"'{args.package_name}'")
            # Extract just the executable names from full paths
            executable_names = sorted(
                [os.path.basename(p) for p in executable_paths])
            selected_executable = interactive_select(
                executable_names,
                prompt='Select executable:')
            if selected_executable is None:
                return 'No executable selected'
            args.executable_name = selected_executable
        try:
            path = get_executable_path(
                package_name=args.package_name,
                executable_name=args.executable_name)
        except PackageNotFound:
            raise RuntimeError(f"Package '{args.package_name}' not found")
        except MultipleExecutables as e:
            msg = 'Multiple executables found:'
            for p in e.paths:
                msg += f'\n- {p}'
            raise RuntimeError(msg)
        if path is None:
            return 'No executable found'
        prefix = shlex.split(args.prefix) if args.prefix is not None else None
        return run_executable(path=path, argv=args.argv, prefix=prefix)
