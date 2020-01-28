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
from typing import Set
from typing import Tuple

from pkg_resources import iter_entry_points
from pkg_resources import UnknownExtra

from ros2doctor.api.format import doctor_warn


class DoctorCheck:
    """Abstract base class of ros2doctor check."""

    def category(self) -> str:
        """:return: string linking checks and reports."""
        raise NotImplementedError

    def check(self) -> bool:
        """:return: boolean indicating result of checks."""
        raise NotImplementedError


class DoctorReport:
    """Abstract base class of ros2doctor report."""

    def category(self) -> str:
        """:return: string linking checks and reports."""
        raise NotImplementedError

    def report(self) -> 'Report':  # using str as wrapper for custom class Report
        """:return: Report object storing report content."""
        raise NotImplementedError


class Report:
    """Stores report name and content."""

    __slots__ = ['name', 'items']

    def __init__(self, name: str):
        """Initialize with report name."""
        self.name = name
        self.items = []

    def add_to_report(self, item_name: str, item_info: str) -> None:
        """Add report content to items list (list of string tuples)."""
        self.items.append((item_name, item_info))


class Result:
    """Stores check result."""

    __slots__ = ['error', 'warning']

    def __init__(self):
        """Initialize with no error or warning."""
        self.error = 0
        self.warning = 0

    def add_error(self, msg) -> None:
        doctor_warn(msg)
        self.error += 1

    def add_warning(self, msg) -> None:
        doctor_warn(msg)
        self.warning += 1


def run_checks(*, include_warnings=False) -> Tuple[Set[str], int, int]:
    """
    Run all checks and return check results.

    :return: 3-tuple (categories of failed checks, number of failed checks,
             total number of checks)
    """
    failed_cats = set()  # remove repeating elements
    fail = 0
    total = 0
    for check_entry_pt in iter_entry_points('ros2doctor.checks'):
        try:
            check_class = check_entry_pt.load()
        except (ImportError, UnknownExtra):
            doctor_warn(f'Check entry point {check_entry_pt.name} fails to load.')
        try:
            check_instance = check_class()
        except Exception:
            doctor_warn(f'Unable to instantiate check object from {check_entry_pt.name}.')
        try:
            check_category = check_instance.category()
            result = check_instance.check()
            if result.error or (include_warnings and result.warning):
                fail += 1
                failed_cats.add(check_category)
            total += 1
        except Exception:
            doctor_warn(f'Fail to call {check_entry_pt.name} class functions.')
    return failed_cats, fail, total


def generate_reports(*, categories=None) -> List[Report]:
    """
    Print all reports or reports of failed checks to terminal.

    :return: list of Report objects
    """
    reports = []
    for report_entry_pt in iter_entry_points('ros2doctor.report'):
        try:
            report_class = report_entry_pt.load()
        except (ImportError, UnknownExtra):
            doctor_warn(f'Report entry point {report_entry_pt.name} fails to load.')
        try:
            report_instance = report_class()
        except Exception:
            doctor_warn(f'Unable to instantiate report object from {report_entry_pt.name}.')
        try:
            report_category = report_instance.category()
            report = report_instance.report()
            if categories:
                if report_category in categories:
                    reports.append(report)
            else:
                reports.append(report)
        except Exception:
            doctor_warn(f'Fail to call {report_entry_pt.name} class functions.')
    return reports
