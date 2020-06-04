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

import argparse
import sys
import typing

from ros2interface.api import type_completer
from ros2interface.verb import VerbExtension
from rosidl_adapter.parser import \
    parse_message_string, MessageSpecification, \
    SERVICE_REQUEST_RESPONSE_SEPARATOR, ACTION_REQUEST_RESPONSE_SEPARATOR, \
    Field
from rosidl_runtime_py import get_interface_path


class InterfaceTextLine:
    """
    This class uses the class MessageSpecification to parse a single line
    from an interface file and makes the type of the message accessible so
    that it can be used to get the file path to nested messages.
    """

    def __init__(
            self,
            pkg_name: str,
            msg_name: str,
            line_text: str,
    ):
        if line_text in (SERVICE_REQUEST_RESPONSE_SEPARATOR, ACTION_REQUEST_RESPONSE_SEPARATOR):
            msg_spec = None
        else:
            msg_spec = parse_message_string(
                pkg_name=pkg_name,
                msg_name=msg_name,
                message_string=line_text,
            )
            if len(msg_spec.fields) > 1:
                raise ValueError("'message_string' must be only one line")
        self._msg_spec: typing.Optional[MessageSpecification] = msg_spec
        self._raw_message = line_text

    def __str__(self) -> str:
        return self._raw_message

    @property
    def nested_type(self) -> typing.Optional[str]:
        if self._field and self._is_nested():
            interface_type: str = str(self._field.type)
            if self._field.type.is_array:
                interface_type = interface_type[:interface_type.find("[")]
            return interface_type.replace("/", "/msg/")

    @property
    def _field(self) -> typing.Optional[Field]:
        if self._msg_spec and self._msg_spec.fields:
            return self._msg_spec.fields[0]

    def _is_nested(self) -> bool:
        if self._msg_spec and self._msg_spec.fields:
            return "/" in str(self._field.type)
        else:
            return False


def _get_interface_lines(interface_identifier: str) -> typing.Iterable[InterfaceTextLine]:

    parts: typing.List[str] = interface_identifier.split("/")
    if len(parts) != 3:
        raise ValueError(f"Invalid name '{interface_identifier}'. Expected three parts separated by '/'")
    pkg_name, _, msg_name = parts

    file_path = get_interface_path(interface_identifier)
    with open(file_path) as file_handler:
        while True:
            line = file_handler.readline()
            if not line:
                break
            yield InterfaceTextLine(
                pkg_name=pkg_name,
                msg_name=msg_name,
                line_text=line.rstrip(),
            )


def _show_interface(interface_identifier: str, indent_level: int = 0):
    for line in _get_interface_lines(interface_identifier):

        if line:
            indent_string = indent_level*"\t"
            print(f"{indent_string}{line}")
        else:
            print("")

        if line.nested_type:
            _show_interface(
                line.nested_type,
                indent_level=indent_level+1,
            )


class ReadStdinPipe(argparse.Action):
    """Get argument from stdin pipe."""

    def __call__(self, parser, namespace, values, option_string=None):
        if values == '-':
            if sys.stdin.isatty():
                parser.error('expected stdin pipe')
            values = sys.stdin.readline().strip()
        if not values:
            parser.error('the passed value is empty')
        setattr(namespace, self.dest, values)


class ShowVerb(VerbExtension):
    """Output the interface definition."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'type',
            action=ReadStdinPipe,
            help="Show an interface definition (e.g. 'example_interfaces/msg/String'). "
                 "Passing '-' reads the argument from stdin (e.g. "
                 "'ros2 topic type /chatter | ros2 interface show -').")
        arg.completer = type_completer

    def main(self, *, args):
        try:
            _show_interface(args.type)
        except (ValueError, LookupError) as e:
            return str(e)
