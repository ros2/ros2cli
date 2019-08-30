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


class DoctorCheck:

    def target(self):
        raise NotImplementedError

    def check(self):
        raise NotImplementedError


class DoctorReport:

    def target(self):
        raise NotImplementedError

    def report(self):
        raise NotImplementedError


class Report:

    def __init__(self):
        self.name = None
        self.items = []

    def add_to_report(item_name, item_info):
        self.items.append((item_name, item_info))


def run_checks():
    """Run all checks when `ros2 doctor/wtf` is called."""
    all_results = []
    failed_checks = []
    failed_modules = []
    for check in iter_entry_points('ros2doctor.checks'):
        result = check.load()()  # load() returns method
        all_results.append(result)
        if result is False:
            failed_checks.append(check.name)
            failed_modules.append(check.module_name)
    return all_results, failed_checks, failed_modules


def generate_report():
    """Print report to terminal when `-r/--report` is attached."""
    report = {}
    for r in iter_entry_points('ros2doctor.report'):
        if r.module_name in report:
            report[r.module_name].extend(r.load()())
        else:
            report[r.module_name] = r.load()()  # load() returns method
    return report

