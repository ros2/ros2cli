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
    """Abstract base class of ros2doctor check."""

    def category(self):
        """Return: String linking checks and reports."""
        raise NotImplementedError

    def check(self):
        """Return: Boolean indicating result of checks."""
        raise NotImplementedError


class DoctorReport:
    """Abstract base class of ros2doctor report."""

    def category(self):
        """Return: String linking checks and reports."""
        raise NotImplementedError

    def report(self):
        """Return: Report object containing report content."""
        raise NotImplementedError


class Report:
    """Stores report name and content."""

    __slots__ = ['name', 'items']

    def __init__(self, name):
        self.name = name
        self.items = []

    def add_to_report(self, item_name, item_info):
        self.items.append((item_name, item_info))


def run_checks():
    """
    Run all checks and return check results.

    Return: list of String, Int, Int
    """
    failed_cats = set()  # remove repeating elements
    fail = 0
    total = 0
    for check_entry_pt in iter_entry_points('ros2doctor.checks'):
        check_class = check_entry_pt.load()()
        cat = check_class.category()
        result = check_class.check()
        if result is False:
            fail += 1
            failed_cats.add(cat)
        total += 1
    return failed_cats, fail, total


def generate_reports(*, cats=None):
    """
    Print all reports or reports of failed checks to terminal.

    Return: list of Report objects
    """
    reports = []
    for report_entry_pt in iter_entry_points('ros2doctor.report'):
        report_class = report_entry_pt.load()()
        if cats:
            if report_class.category() in cats:
                reports.append(report_class.report())
        else:
            reports.append(report_class.report())
    return reports
