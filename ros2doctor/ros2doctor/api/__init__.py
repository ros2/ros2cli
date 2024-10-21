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

try:
    import importlib.metadata as importlib_metadata
except ModuleNotFoundError:
    import importlib_metadata

from ros2cli.node.strategy import NodeStrategy
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

    def add_error(self):
        self.error += 1

    def add_warning(self):
        self.warning += 1


def run_checks(*, include_warnings=False, exclude_packages=False) -> Tuple[Set[str], int, int]:
    """
    Run all checks and return check results.

    :return: 3-tuple (categories of failed checks, number of failed checks,
             total number of checks)
    """
    fail_categories = set()  # remove repeating elements
    fail = 0
    total = 0
    entry_points = importlib_metadata.entry_points()
    if hasattr(entry_points, 'select'):
        groups = entry_points.select(group='ros2doctor.checks')
    else:
        groups = entry_points.get('ros2doctor.checks', [])

    if exclude_packages:
        groups = [ep for ep in groups if ep.name != 'PackageCheck']

    for check_entry_pt in groups:
        try:
            check_class = check_entry_pt.load()
        except ImportError as e:
            doctor_warn(f'Check entry point {check_entry_pt.name} fails to load: {e}')
            continue
        try:
            check_instance = check_class()
        except Exception:
            doctor_warn(f'Unable to instantiate check object from {check_entry_pt.name}.')
            continue
        try:
            check_category = check_instance.category()
            result = check_instance.check()
            if result.error or (include_warnings and result.warning):
                fail += 1
                fail_categories.add(check_category)
            total += 1
        except Exception:
            doctor_warn(f'Fail to call {check_entry_pt.name} class functions.')
    return fail_categories, fail, total


def generate_reports(*, categories=None, exclude_packages=False) -> List[Report]:
    """
    Print all reports or reports of failed checks to terminal.

    :return: list of Report objects
    """
    reports = []
    entry_points = importlib_metadata.entry_points()
    if hasattr(entry_points, 'select'):
        groups = entry_points.select(group='ros2doctor.report')
    else:
        groups = entry_points.get('ros2doctor.report', [])

    if exclude_packages:
        groups = [ep for ep in groups if ep.name != 'PackageReport']

    for report_entry_pt in groups:
        try:
            report_class = report_entry_pt.load()
        except ImportError as e:
            doctor_warn(f'Report entry point {report_entry_pt.name} fails to load: {e}')
            continue
        try:
            report_instance = report_class()
        except Exception:
            doctor_warn(f'Unable to instantiate report object from {report_entry_pt.name}.')
            continue
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


def get_topic_names(skip_topics: List = ()) -> List:
    """Get all topic names using rclpy API."""
    topics = []
    with NodeStrategy(None) as node:
        topic_names_types = node.get_topic_names_and_types()
        for t_name, _ in topic_names_types:
            if t_name not in skip_topics:
                topics.append(t_name)
    return topics
