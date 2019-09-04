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

import sys


def format_print(report):
    """Format print report content."""
    # temp fix for missing ifcfg
    if report is None:
        sys.stderr.write('No report found. Skip print...')
        return

    print('\n', report.name)
    padding_num = compute_padding(report.items)
    for item_name, item_content in report.items:
        print('{:{padding}}: {}'.format(item_name, item_content, padding=padding_num))


def compute_padding(report_items):
    """Compute padding based on report content."""
    padding = 8
    check_items = list(zip(*report_items))[0]  # get first elements of tuples
    max_len = len(max(check_items, key=len))  # find the longest string length
    if max_len >= padding:
        padding = max_len + 4  # padding number is longest string length + 4
    return padding


def warning_format(msg, cat, filename, linenum, file=None, line=None):
    return '%s: %s: %s: %s\n' % (filename, linenum, cat.__name__, msg)
