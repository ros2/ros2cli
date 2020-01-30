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

from typing import Callable
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


def doctor_warn() -> Callable:
    """
    Print customized warning message with package and line info.

    :param msg: warning message to be printed
    """
    def custom_warning_format(message, category, filename, lineno, file=None, line=None):
        return f'{filename}:{lineno}: {category.__name__}: {message}\n'
    warnings.formatwarning = custom_warning_format
    return warnings.warn
