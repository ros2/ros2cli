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


def format_print(names, report):
    """Format print report according to module names."""
    padding_num = compute_padding(names, report)
    for n in names:
        if n in report:
            module_report = report.get(n)
            for k, v in module_report:
                if k == 'NAME':
                    print(v)
                else:
                    print('{:{padding}}: {}'.format(k, v, padding = padding_num))
        else:
            # print warning if no report available for a particular check
            sys.stderr.write('WARNING: No report available for this check.')


def compute_padding(names, report):
    """Compute padding based on report content."""
    padding = 8
    for n in names:
        if n in report:
            module_report = report.get(n)
            item_names = list(zip(*module_report))[0]
            max_len = len(max(item_names, key=len))
            if max_len >= padding:
                padding = max_len + 4
    return padding