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
import re
import sys
import typing
from typing import Optional

from ros2interface.api import type_completer
from ros2interface.verb import VerbExtension
from rosidl_runtime_py import get_interface_path
from rosidl_parser.definition import BASIC_TYPES


PRIMITIVE_FIELD_TYPES = [
    'bool',
    'byte',
    'char',
    'float32',
    'float64',
    'int8',
    'uint8',
    'int16',
    'uint16',
    'int32',
    'uint32',
    'int64',
    'uint64',
    'string',
]


class InterfaceTextLine:

    def __init__(
            self,
            text: str,
            indent_level: int = 0,
            default_package: str = None,
            default_type: str = None,
            indent_str: str = "\t",
    ):

        self._text = text
        self._indent_level = indent_level
        self._default_package = default_package
        self._default_type = default_type
        self._indent_str = indent_str

    def __str__(self):
        return self._text

    def __repr__(self):
        return self.__str__()

    def get_text(
            self,
            raw: bool = True,
            comment_string: str = "#",
            parenthatic_chars: str = "'\"`",
            escape_char: str = "\\",
    ):

        if not raw:
            if self._is_comment_or_whitespace() and not self._is_separator():
                return ""
            text = remove_comment_from_string(
                self._text,
                comment_string=comment_string,
                parenthatic_chars=parenthatic_chars,
                escape_char=escape_char,
            )
        else:
            text = self._text

        return "{indent}{text}".format(
            indent=self._indent_level*self._indent_str,
            text=text
        )

    def get_interface_string(self) -> str:
        interface_package, interface_type, interface_name = self.get_interface_parts()
        return "{package}/{type}/{name}".format(
            package=interface_package,
            type=interface_type,
            name=interface_name,
        )

    def get_interface_package_name(self) -> str:
        interface_package, _, _ = self.get_interface_parts()
        return interface_package

    def get_interface_type_name(self) -> str:
        _, interface_type, _ = self.get_interface_parts()
        return interface_type

    def is_expandable(self) -> bool:
        if self._is_comment_or_whitespace() or self._is_basic_idl_type():
            return False
        else:
            try:
                self.get_interface_string()
                return True
            except ValueError:
                return False

    @property
    def indent_level(self):
        return self._indent_level

    def _is_comment_or_whitespace(self) -> bool:
        return self._get_interface_string_from_text() is None

    def _is_separator(self) -> bool:
        return re.match(r"^[-]{3}\s*$", self._text) is not None

    def _is_basic_idl_type(self) -> bool:
        return self._get_interface_string_from_text() in PRIMITIVE_FIELD_TYPES

    def get_interface_parts(self) -> typing.Tuple[str, str, str]:
        interface_package, interface_type, interface_name = self._get_interface_parts_from_text()
        if interface_name is None:
            raise ValueError("No package name found")
        if interface_package is None:
            if self._default_package is None:
                raise ValueError("Unable to determine the package")
            interface_package = self._default_package
        if interface_type is None:
            if self._default_type is None:
                raise ValueError("Unable to determine the interface's type")
            interface_type = self._default_type
        return interface_package, interface_type, interface_name

    def _get_interface_parts_from_text(self) \
            -> typing.Tuple[Optional[str], Optional[str], Optional[str]]:
        interface_string = self._get_interface_string_from_text()
        if interface_string is None:
            return None, None, None
        match = re.match(
            r"^(?:(\w+)\/)?(?:(\w+)\/)?(\w+)$",
            interface_string
        )
        if match:
            package, interface_type, name = match.groups()
            return package, interface_type, name
        else:
            return None, None, None

    def _get_interface_string_from_text(self) -> typing.Optional[str]:
        match = re.match(r"^([\w\/]+).*$", self._text)
        if match:
            return match.group(1)
        else:
            return None


def remove_comment_from_string(
        text: str,
        comment_string: str = "#",
        parenthatic_chars: str = "'\"`",
        escape_char: str = "\\",
):
    parenthatic_stack = []
    comment_len = len(comment_string)
    for idx in range(len(text)):
        current_char = text[idx]
        if (
                current_char in parenthatic_chars
                and
                not is_preceded_by_escape_char(
                    text, idx, escape_char=escape_char,
                )
        ):
            if len(parenthatic_stack) == 0:
                parenthatic_stack.append(current_char)
            elif parenthatic_stack[-1] == current_char:
                parenthatic_stack.pop()
            else:
                parenthatic_stack.append(current_char)
        if parenthatic_stack == [] and len(text[idx:]) >= comment_len:
            if text[idx:idx + comment_len] == comment_string:
                return text[:idx].rstrip()
    if len(parenthatic_stack) > 0:
        raise ValueError("Not all parenthatic characters not were closed")
    return text


def is_preceded_by_escape_char(
        text: str,
        idx: int,
        escape_char: str = "\\",
):
    if idx > 2:
        if text[idx-1] == escape_char:
            if text[idx-2] == escape_char:
                return False
            else:
                return True
        else:
            return False
    elif idx == 1:
        if text[idx-1] == escape_char:
            return True
        else:
            return False
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
        parser.add_argument("-r", "--raw",
                          dest="raw", default=False, action="store_true",
                          help="show raw message text, including comments")
        arg.completer = type_completer

    def main(self, *, args):

        InterfaceExpander(args.type).print(raw=args.raw)


class InterfaceExpander:

    def __init__(self, interface_string):

        self._text_lines: typing.List[InterfaceTextLine] = []

        package_name = InterfaceTextLine(interface_string).get_interface_package_name()
        self._text_lines = self._expand_interface(
            interface_string,
            indent_level=0,
            default_package=package_name,
            default_type="msg"
        )

        line_idx: int = 0
        while line_idx < len(self._text_lines):
            line = self._text_lines[line_idx]
            if line.is_expandable():
                new_lines: typing.List[InterfaceTextLine] = self._expand_interface(
                    line.get_interface_string(),
                    indent_level=line.indent_level+1,
                    default_package=line.get_interface_package_name(),
                    default_type="msg",
                )
                self._insert_new_lines(new_lines, line_idx)
            line_idx += 1

    def print(self, raw: bool = True):
        for l in self._text_lines:
            text = l.get_text(raw=raw)
            if text != "" or raw:
                print(text)

    def _expand_interface(
            self,
            interface_string: str,
            indent_level: int = 0,
            default_package: str = None,
            default_type: str = None,
    ) -> typing.List[InterfaceTextLine]:
        file_path = self._get_path_from_ambiguous_interface_string(
            interface_string=interface_string,
            default_package=default_package,
            default_type=default_type,
        )
        with open(file_path, 'r', encoding='utf-8') as h:
            return [
                InterfaceTextLine(
                    text,
                    indent_level=indent_level,
                    default_package=default_package,
                    default_type="msg",
                )
                for text in h.read().rstrip().splitlines()
            ]

    def _insert_new_lines(self, new_lines: typing.List[InterfaceTextLine], insert_idx: int):
        self._text_lines = self._text_lines[:insert_idx + 1] + new_lines + self._text_lines[insert_idx + 1:]

    @staticmethod
    def _get_path_from_ambiguous_interface_string(
            interface_string: str,
            default_package: str = None,
            default_type: str = None,
    ) -> str:
        try:
            file_path = get_interface_path(interface_string)
        except (LookupError, ValueError) as e1:
            if default_package is None or default_type is None:
                raise e1
            else:
                try:
                    file_path = get_interface_path(
                        f"{default_package}/{default_type}/{interface_string}"
                    )
                except (LookupError, ValueError) as e2:
                    raise e2
        return file_path


if __name__ == '__main__':

    import collections

    is_raw = False

    Args = collections.namedtuple('Args', ['type', 'raw'])
    #args = Args('geometry_msgs/msg/PolygonStamped', is_raw)
    #args = Args('geometry_msgs/msg/Twist', is_raw)
    args = Args('std_srvs/srv/Trigger', is_raw)
    sv = ShowVerb()
    sv.main(args=args)
