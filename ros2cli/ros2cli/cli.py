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
import builtins
import functools
import signal

from ros2cli.command import add_subparsers_on_demand


def main(*, script_name='ros2', argv=None, description=None, extension=None):
    # Make the output always line buffered, even when piping the output to another process.
    # If you explicitly passed `flush=false`, that's not going to be the case.
    # This only modifies the behavior of print(), if you write to stdout in another way line
    # buffering is not guaranteed.
    builtins.print = functools.partial(print, flush=True)

    if description is None:
        description = f'{script_name} is an extensible command-line tool ' \
            'for ROS 2.'

    # top level parser
    parser = argparse.ArgumentParser(
        description=description,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    # add arguments for command extension(s)
    if extension:
        extension.add_arguments(parser, script_name)
    else:
        # get command entry points as needed
        selected_extension_key = '_command'
        add_subparsers_on_demand(
            parser, script_name, selected_extension_key, 'ros2cli.command',
            # hide the special commands in the help
            hide_extensions=['extension_points', 'extensions'],
            required=False, argv=argv)

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
        # get extension identified by the passed command (if available)
        extension = getattr(args, selected_extension_key, None)

    # handle the case that no command was passed
    if extension is None:
        parser.print_help()
        return 0

    # call the main method of the extension
    try:
        rc = extension.main(parser=parser, args=args)
    except KeyboardInterrupt:
        rc = signal.SIGINT
    except RuntimeError as e:
        rc = str(e)
    return rc
