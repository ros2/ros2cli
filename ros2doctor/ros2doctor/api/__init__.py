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

from pkg_resources import iter_entry_points


def run_checks():
    """Run all checks when `ros2 doctor/wtf` is called."""
    all_results = []
    failed_names = []
    for check in iter_entry_points('ros2doctor.checks'):
        result = check.load()()  # load() returns method
        all_results.append(result)
        if result is False:
            failed_names.append(check.name)
    return all_results, failed_names


def generate_report():
    """Print report to terminal when `-r/--report` is attached."""
    for report in iter_entry_points('ros2doctor.report'):
        report.load()()  # load() returns method
