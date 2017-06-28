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

import argparse
import signal

from ros2cli.command import add_subparsers
from ros2cli.command import get_command_extensions


def main(*, script_name='ros2', argv=None, description=None, extension=None):
    if description is None:
        description = '{script_name} is an extensible command-line tool for ' \
            'ROS 2.'.format_map(locals())

    # top level parser
    parser = argparse.ArgumentParser(
        description=description,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    # add arguments for command extension(s)
    if extension:
        extension.add_arguments(parser, script_name)
    else:
        # get command extensions
        extensions = get_command_extensions('ros2cli.command')
        selected_extension_key = '_command'
        add_subparsers(
            parser, script_name, selected_extension_key, extensions,
            # hide the special commands in the help
            hide_extensions=['extension_points', 'extensions'])

    # register argcomplete hook if available
    try:
        from argcomplete import autocomplete
    except ImportError:
        pass
    else:
        autocomplete(parser, exclude=['-h', '--help'])

    # parse the command line arguments
    args = parser.parse_args(args=argv)

    if extension is None:
        # the attribute should always exist
        # otherwise argparse should have exited
        extension = getattr(args, selected_extension_key)

    # call the main method of the extension
    try:
        rc = extension.main(parser=parser, args=args)
    except KeyboardInterrupt:
        rc = signal.SIGINT
    except RuntimeError as e:
        rc = str(e)
    return rc
