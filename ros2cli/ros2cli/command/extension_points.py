# Copyright 2016-2017 Dirk Thomas
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
from ros2cli.entry_points import EXTENSION_POINT_GROUP_NAME
from ros2cli.entry_points import get_entry_points
from ros2cli.entry_points import get_first_line_doc


class ExtensionPointsCommand(CommandExtension):
    """List extension points."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '--all', '-a',
            action='store_true',
            default=False,
            help='Also show extension points which failed to be imported. '
                 '(prefixed with `- `)')
        parser.add_argument(
            '--verbose', '-v',
            action='store_true',
            default=False,
            help='Show more information for each extension point')

    def main(self, *, parser, args):
        extension_points = get_entry_points(EXTENSION_POINT_GROUP_NAME)
        for name in sorted(extension_points.keys()):
            self.print_extension_point(
                args, name, extension_points[name])

    def print_extension_point(self, args, name, entry_point):
        exception = None
        try:
            extension_point = entry_point.load()
        except Exception as e:
            if not args.all:
                # skip entry points which failed to load
                return
            exception = e
            extension_point = None

        prefix = '' if exception is None else '- '
        print(prefix + name + ':', get_first_line_doc(extension_point))

        if args.verbose:
            print(prefix, ' ', 'module_name:', entry_point.module_name)
            if entry_point.attrs:
                print(prefix, ' ', 'attributes:', '.'.join(entry_point.attrs))
            if hasattr(extension_point, 'EXTENSION_POINT_VERSION'):
                print(
                    prefix, ' ', 'version:',
                    extension_point.EXTENSION_POINT_VERSION)

        if exception:
            print(prefix, ' ', 'reason:', str(exception))
