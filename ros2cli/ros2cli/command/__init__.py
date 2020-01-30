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
import inspect
import os
import shlex
import types

from ros2cli.entry_points import get_entry_points
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

    def add_arguments(self, parser, cli_name, *, argv=None):
        pass

    def main(self, *, parser, args):
        raise NotImplementedError()


def get_command_extensions(group_name, *, exclude_names=None):
    extensions = instantiate_extensions(
        group_name, exclude_names=exclude_names)
    for name, extension in extensions.items():
        extension.NAME = name
    return extensions


def add_subparsers(
    parser, cli_name, dest, command_extensions, hide_extensions=None,
    required=True
):
    """
    Create argparse subparser for each extension.

    The ``cli_name`` is used for the title and description of the
    ``add_subparsers`` function call.

    For each extension a subparser is created.
    If the extension has an ``add_arguments`` method it is being called.

    This method is deprecated.
    Use the function ``add_subparsers_on_demand`` instead.
    Their signatures are almost identical.
    Instead of passing the extensions the new function expects the group name
    of these extensions.

    :param parser: the parent argument parser
    :type parser: :py:class:`argparse.ArgumentParser`
    :param str cli_name: name of the command line command to which the
      subparsers are being added
    :param str dest: name of the attribute under which the selected extension
      will be stored
    :param dict command_extensions: dict of command extensions by their name
      where each contributes a command with specific arguments
    """
    import warnings
    warnings.warn(
        "'ros2cli.command.add_subparsers' is deprecated, use "
        "'ros2cli.command.add_subparsers_on_demand' instead", stacklevel=2)
    # add subparser with description of available subparsers
    description = ''
    if command_extensions:
        max_length = max(
            len(name) for name in command_extensions.keys()
            if hide_extensions is None or name not in hide_extensions)
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
    subparser.required = required

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

    return subparser


class MutableString:
    """Behave like str with the ability to change the value of an instance."""

    def __init__(self):
        self.value = ''

    def __getattr__(self, name):
        return getattr(self.value, name)

    def __iter__(self):
        return self.value.__iter__()


def add_subparsers_on_demand(
    parser, cli_name, dest, group_name, hide_extensions=None,
    required=True, argv=None
):
    """
    Create argparse subparser for each extension on demand.

    The ``cli_name`` is used for the title and description of the
    ``add_subparsers`` function call.

    For each extension a subparser is created is necessary.
    If no extension has been selected by command line arguments all first level
    extension must be loaded and instantiated.
    If a specific extension has been selected by command line arguments the
    sibling extension can be skipped and only that one extension (as well as
    potentially its recursive extensions) are loaded and instantiated.
    If the extension has an ``add_arguments`` method it is being called.

    :param parser: the parent argument parser
    :type parser: :py:class:`argparse.ArgumentParser`
    :param str cli_name: name of the command line command to which the
      subparsers are being added
    :param str dest: name of the attribute under which the selected extension
      will be stored
    :param str group_name: the name of the ``entry_point`` group identifying
      the extensions to be added
    :param list hide_extensions: an optional list of extension names which
      should be skipped
    :param bool required: a flag if the command is a required argument
    :param list argv: the list of command line arguments (default:
      ``sys.argv``)
    """
    # add subparser without a description for now
    mutable_description = MutableString()
    metavar = 'Call `{cli_name} <command> -h` for more detailed ' \
        'usage.'.format_map(locals())
    subparser = parser.add_subparsers(
        title='Commands', description=mutable_description, metavar=metavar)
    # use a name which doesn't collide with any argument
    # but is readable when shown as part of the the usage information
    subparser.dest = ' ' + dest.lstrip('_')
    subparser.required = required

    # add entry point specific sub-parsers but without a description and
    # arguments for now
    entry_points = get_entry_points(group_name)
    command_parsers = {}
    for name in sorted(entry_points.keys()):
        entry_point = entry_points[name]
        command_parser = subparser.add_parser(
            name,
            formatter_class=argparse.RawDescriptionHelpFormatter)
        command_parsers[name] = command_parser

    # temporarily attach root parser to each command parser
    # in order to parse known args
    root_parser = getattr(parser, '_root_parser', parser)
    with SuppressUsageOutput({parser} | set(command_parsers.values())):
        args = argv
        # for completion use the arguments provided by the arcomplete env var
        if _is_completion_requested():
            args = shlex.split(os.environ['COMP_LINE'])[1:]
        try:
            known_args, _ = root_parser.parse_known_args(args=args)
        except SystemExit:
            if not _is_completion_requested():
                raise
            # if the partial arguments can't be parsed use no known args
            known_args = argparse.Namespace(**{subparser.dest: None})

    # check if a specific subparser is selected
    name = getattr(known_args, subparser.dest)
    if name is None:
        # add description for all command extensions to the root parser
        command_extensions = get_command_extensions(group_name)
        if command_extensions:
            description = ''
            max_length = max(
                len(name) for name in command_extensions.keys()
                if hide_extensions is None or name not in hide_extensions)
            for name in sorted(command_extensions.keys()):
                if hide_extensions is not None and name in hide_extensions:
                    continue
                extension = command_extensions[name]
                description += '%s  %s\n' % (
                    name.ljust(max_length), get_first_line_doc(extension))
                command_parser = command_parsers[name]
                command_parser.set_defaults(**{dest: extension})
            mutable_description.value = description
    else:
        # add description for the selected command extension to the subparser
        command_extensions = get_command_extensions(
            group_name, exclude_names=set(entry_points.keys() - {name}))
        extension = command_extensions[name]
        command_parser = command_parsers[name]
        command_parser.set_defaults(**{dest: extension})
        command_parser.description = get_first_line_doc(extension)

        # add the arguments for the requested extension
        if hasattr(extension, 'add_arguments'):
            command_parser = command_parsers[name]
            command_parser._root_parser = root_parser
            signature = inspect.signature(extension.add_arguments)
            kwargs = {}
            if 'argv' in signature.parameters:
                kwargs['argv'] = argv
            extension.add_arguments(
                command_parser, '{cli_name} {name}'.format_map(locals()),
                **kwargs)
            del command_parser._root_parser

    return subparser


class SuppressUsageOutput:
    """Context manager to suppress help action during `parse_known_args`."""

    def __init__(self, parsers):
        """
        Construct a SuppressUsageOutput.

        :param parsers: The parsers
        """
        self._parsers = parsers
        self._callbacks = {}

    def __enter__(self):  # noqa: D105
        for p in self._parsers:
            self._callbacks[p] = p.print_help, p.exit
            # temporary prevent printing usage early if help is requested
            p.print_help = lambda: None
            # temporary prevent help action to exit early,
            # but keep exiting on invalid arguments
            p.exit = types.MethodType(_ignore_zero_exit(p.exit), p)

        return self

    def __exit__(self, *args):  # noqa: D105
        for p, callbacks in self._callbacks.items():
            p.print_help, p.exit = callbacks


def _ignore_zero_exit(original_exit_handler):
    def exit_(self, status=0, message=None):
        nonlocal original_exit_handler
        if status == 0:
            return
        return original_exit_handler(status=status, message=message)

    return exit_


def _is_completion_requested():
    return os.environ.get('_ARGCOMPLETE') == '1'
