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

from ros2cli.entry_points import get_first_line_doc
from ros2cli.plugin_system import instantiate_extensions
from ros2cli.plugin_system import PLUGIN_SYSTEM_VERSION
from ros2cli.plugin_system import satisfies_version


class CommandExtension:
    """
    The extension point for 'command' extensions.

    The following properties must be defined:
    * `NAME` (will be set to the entry point name)

    The following methods must be defined:
    * `main`

    The following methods can be defined:
    * `add_arguments`
    """

    NAME = None
    EXTENSION_POINT_VERSION = '0.1'

    def __init__(self):
        super(CommandExtension, self).__init__()
        satisfies_version(PLUGIN_SYSTEM_VERSION, '^0.1')

    def add_arguments(self, parser, cli_name):
        pass

    def main(self, *, parser, args):
        raise NotImplementedError()


def get_command_extensions(group_name):
    extensions = instantiate_extensions(group_name)
    for name, extension in extensions.items():
        extension.NAME = name
    return extensions


def add_subparsers(
    parser, cli_name, dest, command_extensions, hide_extensions=None
):
    """
    Create argparse subparser for each extension.

    The ``cli_name`` is used for the title and description of the
    ``add_subparsers`` function call.

    For each extension a subparser is created.
    If the extension has an ``add_arguments`` method it is being called.

    :param parser: the parent argument parser
    :type parser: :py:class:`argparse.ArgumentParser`
    :param str cli_name: name of the command line command to which the
      subparsers are being added
    :param str dest: name of the attribute under which the selected extension
      will be stored
    :param dict command_extensions: dict of command extensions by their name
      where each contributes a command with specific arguments
    """
    # add subparser with description of available subparsers
    description = ''
    if command_extensions:
        max_length = max([
            len(name) for name in command_extensions.keys()
            if hide_extensions is None or name not in hide_extensions])
        for name in sorted(command_extensions.keys()):
            if hide_extensions is not None and name in hide_extensions:
                continue
            extension = command_extensions[name]
            description += '%s  %s\n' % (
                name.ljust(max_length), get_first_line_doc(extension))
    metavar = 'Call `{cli_name} <command> -h` for more detailed ' \
        'usage.'.format_map(locals())
    subparser = parser.add_subparsers(
        title='Commands', description=description, metavar=metavar)
    # use a name which doesn't collide with any argument
    # but is readable when shown as part of the the usage information
    subparser.dest = ' ' + dest.lstrip('_')
    subparser.required = True

    # add extension specific sub-parser with its arguments
    for name in sorted(command_extensions.keys()):
        extension = command_extensions[name]
        command_parser = subparser.add_parser(
            extension.NAME,
            description=get_first_line_doc(extension),
            formatter_class=argparse.RawDescriptionHelpFormatter)
        command_parser.set_defaults(**{dest: extension})
        if hasattr(extension, 'add_arguments'):
            extension.add_arguments(
                command_parser, '{cli_name} {name}'.format_map(locals()))
