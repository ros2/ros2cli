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


class TextLine:

    def __init__(
            self,
            pkg_name: str,
            msg_name: str,
            message_string: str,
            indent_level: int = 0
    ):
        if message_string in (SERVICE_REQUEST_RESPONSE_SEPARATOR, ACTION_REQUEST_RESPONSE_SEPARATOR):
            msg_spec = None
        else:
            msg_spec = parse_message_string(
                pkg_name=pkg_name,
                msg_name=msg_name,
                message_string=message_string,
            )
            if len(msg_spec.fields) > 1:
                raise ValueError("'message_string' must be only one line")
        self._msg_spec: typing.Optional[MessageSpecification] = msg_spec
        self._raw_message = message_string
        self._indent_level: int = indent_level

    def __str__(self) -> str:
        if self._raw_message:
            indent_str = '\t' * self._indent_level
            return f"{indent_str}{self._raw_message}"
        else:
            return ""

    @property
    def indent_level(self) -> int:
        return self._indent_level

    @property
    def _field(self) -> typing.Optional[Field]:
        if self._msg_spec and self._msg_spec.fields:
            return self._msg_spec.fields[0]
        else:
            return None

    @property
    def nested_type(self) -> typing.Optional[str]:
        if self._field and self._is_nested():
            interface_type: str = str(self._field.type)
            return interface_type.replace("/", "/msg/")
        else:
            return None

    def _is_nested(self) -> bool:
        if self._msg_spec and self._msg_spec.fields:
            return "/" in str(self._field.type)
        else:
            return False


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
            self._expand_message(interface_identifier=args.type)
        except (ValueError, LookupError) as e:
            return str(e)

    def _expand_message(self, interface_identifier: str):
        lines = self._get_interface_content(interface_identifier, indent_level=0)
        line_idx = 0
        while line_idx < len(lines):
            line = lines[line_idx]
            if line.nested_type:
                new_lines = self._get_interface_content(
                    line.nested_type,
                    indent_level=line.indent_level+1,
                )
                lines = lines[:line_idx+1] + new_lines + lines[line_idx+1:]
            print(line)
            line_idx += 1

    @staticmethod
    def _get_interface_content(
            interface_identifier: str,
            indent_level: int = 0,
    ) -> typing.List[TextLine]:

        parts: typing.List[str] = interface_identifier.split("/")
        if len(parts) != 3:
            raise ValueError(f"Invalid name '{interface_identifier}'. Expected at least two parts separated by '/'")
        pkg_name, _, msg_name = parts

        file_path = get_interface_path(interface_identifier)
        with open(file_path, 'r', encoding='utf-8') as h:
            content = h.read().rstrip()
        return [
            TextLine(
                pkg_name=pkg_name,
                msg_name=msg_name,
                message_string=message_string,
                indent_level=indent_level,
            )
            for message_string in content.splitlines()
        ]
