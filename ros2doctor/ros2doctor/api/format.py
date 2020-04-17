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

from typing import List
from typing import Tuple
import warnings


def format_print(report):
    """
    Format print report content.

    :param report: Report object with name and items list
    """
    print('\n  ', report.name)
    if report.items:
        padding_num = compute_padding(report.items)
    for item_name, item_content in report.items:
        print('{:{padding}}: {}'.format(item_name, item_content, padding=padding_num))


def compute_padding(report_items: List[Tuple[str, str]]) -> int:
    """
    Compute padding based on report content.

    :param report_items: list of item name and item content tuple
    :return: padding number
    """
    padding = 8
    check_items = list(zip(*report_items))[0]  # get first elements of tuples
    max_len = len(max(check_items, key=len))  # find the longest string length
    if max_len >= padding:
        padding = max_len + 4  # padding number is longest string length + 4 spaces
    return padding


def custom_warning_format(msg, cat, filename, linenum, file=None, line=None):
    return '%s: %s: %s: %s\n' % (filename, linenum, cat.__name__, msg)


class CustomWarningFormat:
    """Support custom warning format without modifying default format."""

    def __enter__(self):
        self._default_format = warnings.formatwarning
        warnings.formatwarning = custom_warning_format

    def __exit__(self, t, v, trb):
        """
        Define exit action for context manager.

        :param t: type
        :param v: value
        :param trb: traceback
        """
        warnings.formatwarning = self._default_format


def doctor_warn(msg: str) -> None:
    """
    Print customized warning message with package and line info.

    :param msg: warning message to be printed
    """
    with CustomWarningFormat():
        warnings.warn(msg, stacklevel=2)


def doctor_error(msg: str) -> None:
    """
    Print customized error message with package and line info.

    :param msg: error message to be printed
    """
    with CustomWarningFormat():
        warnings.warn(f'ERROR: {msg}', stacklevel=2)
