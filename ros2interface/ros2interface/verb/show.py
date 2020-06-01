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


import re
import typing
from typing import Optional

from ros2interface.api import type_completer
from ros2interface.verb import VerbExtension
from rosidl_runtime_py import get_interface_path
from rosidl_parser.definition import BASIC_TYPES


class ShowVerb(VerbExtension):
    """Output the interface definition."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
                'type',
                help="Show an interface definition (e.g. 'example_interfaces/msg/String')")
        arg.completer = type_completer

    def main(self, *, args):
        try:
            file_path = get_interface_path(args.type)
        except LookupError as e:
            return str(e)
        with open(file_path, 'r', encoding='utf-8') as h:
            print(h.read().rstrip())


class InterfaceExpander:

    def __init__(self, interface_string):
        self._lines: typing.List[TextLine] = []


class TextLine:

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

    def get_text(
            self,
            raw: bool = True,
            comment_string: str = "#",
            parenthatic_chars: str = "'\"`",
            escape_char: str = "\\",
    ):

        if not raw:
            if self._is_comment_or_whitespace():
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
        interface_package, interface_type, interface_name = self._get_interface_parts()
        return "{package}/{type}/{name}".format(
            package=interface_package,
            type=interface_type,
            name=interface_name,
        )

    def get_interface_package_name(self) -> str:
        interface_package, _, _ = self._get_interface_parts()
        return interface_package

    def get_interface_type_name(self) -> str:
        _, interface_type, _ = self._get_interface_parts()
        return interface_type

    def is_expandable(self) -> bool:
        # TODO add or with basic types
        if self._is_comment_or_whitespace():
            return False
        else:
            try:
                self.get_interface_string()
                return True
            except ValueError:
                return False

    def _is_comment_or_whitespace(self) -> bool:
        return self._get_interface_string_from_text() is None

    def _is_basic_idl_type(self) -> bool:
        return self._get_interface_string_from_text() in BASIC_TYPES

    def _get_interface_parts(self) -> typing.Tuple[str, str, str]:
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
