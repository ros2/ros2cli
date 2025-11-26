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

from importlib import metadata
import sys
from typing import Final
from typing import List
from typing import Set
from typing import Tuple
from typing import Union

from ros2cli.node.strategy import NodeStrategy
from ros2doctor.api.format import doctor_warn


# List of Environment Variables compiled from github.com/ros2/ros2cli/issues/1046
# TODO(@fujitatomoya): In the future maybe get from centralized storage via rcl/rmw interfaces.
# NOTE: In alphabetical order for ease of searching.
ROS_ENVIRONMENT_VARIABLES: Final = [
    'RCL_LOGGING_SPDLOG_EXPERIMENTAL_OLD_FLUSHING_BEHAVIOR',
    'ROS_AUTOMATIC_DISCOVERY_RANGE',
    'ROS_DISABLE_LOANED_MESSAGES',
    'ROS_DOMAIN_ID',
    'ROS_DISTRO',
    'ROS_HOME',
    'ROS_LOG_DIR',
    'ROS_SECURITY_ENABLE',
    'ROS_SECURITY_ENCLAVE_OVERRIDE',
    'ROS_SECURITY_KEYSTORE',
    'ROS_SECURITY_STRATEGY',
    'ROS_STATIC_PEERS',
    'ROS_TRACE_DIR'
    'RMW_IMPLEMENTATION',
    'TRACETOOLS_RUNTIME_DISABLE'
]


RCUTILS_ENVIRONMENT_VARIABLES: Final = [
    'RCUTILS_COLORIZED_OUTPUT',
    'RCUTILS_CONSOLE_OUTPUT_FORMAT',
    'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED',
    'RCUTILS_LOGGING_BUFFERED_STREAM',
    'RCUTILS_LOGGING_USE_STDOUT'
]


RMW_FASTRTPS_ENVIRONMENT_VARIABLES: Final = [
    'FASTDDS_BUILTIN_TRANSPORTS',
    'FASTRTPS_DEFAULT_PROFILES_FILE',
    'RMW_FASTRTPS_PUBLICATION_MODE',
    'RMW_FASTRTPS_USE_QOS_FROM_XML'
]


RMW_ZENOH_CPP_ENVIRONMENT_VARIABLES: Final = [
    'RUST_LOG',
    'ZENOH_CONFIG_OVERRIDE',
    'ZENOH_ROUTER_CHECK_ATTEMPTS',
    'ZENOH_ROUTER_CONFIG_URI',
    'ZENOH_SESSION_CONFIG_URI'
]


RMW_CONNEXTDDS_ENVIRONMENT_VARIABLES: Final = [
    'RMW_CONNEXT_CYCLONE_COMPATIBILITY_MODE',
    'RMW_CONNEXT_DISABLE_RELIABILITY_OPTIMIZATIONS',
    'RMW_CONNEXT_DISABLE_FAST_ENDPOINT_DISCOVERY',
    'RMW_CONNEXT_DISABLE_LARGE_DATA_OPTIMIZATIONS',
    'RMW_CONNEXT_ENDPOINT_QOS_OVERRIDE_POLICY',
    'RMW_CONNEXT_ENV_UDP_INTERFACE',
    'RMW_CONNEXT_INITIAL_PEERS',
    'RMW_CONNEXT_OLD_RMW_COMPATIBILITY_MODE',
    'RMW_CONNEXT_PARTICIPANT_QOS_OVERRIDE_POLICY',
    'RMW_CONNEXT_REQUEST_REPLY_MAPPING',
    'RMW_CONNEXT_SECURITY_LOG_FILE',
    'RMW_CONNEXT_SECURITY_LOG_PUBLISH',
    'RMW_CONNEXT_SECURITY_LOG_VERBOSITY',
    'RMW_CONNEXT_USE_DEFAULT_PUBLISH_MODE',
]


RMW_CYCLONEDDS_ENVIRONMENT_VARIABLES: Final = [
    'CYCLONEDDS_URI'
]


ALL_ENVIRONMENT_VARIABLES: Final = [
    *ROS_ENVIRONMENT_VARIABLES,
    *RCUTILS_ENVIRONMENT_VARIABLES,
    *RMW_FASTRTPS_ENVIRONMENT_VARIABLES,
    *RMW_ZENOH_CPP_ENVIRONMENT_VARIABLES,
    *RMW_CONNEXTDDS_ENVIRONMENT_VARIABLES,
    *RMW_CYCLONEDDS_ENVIRONMENT_VARIABLES
]

RMW_ENVIRONMENT_VARIABLES: Final = {
    'rmw_connextdds': RMW_CONNEXTDDS_ENVIRONMENT_VARIABLES,
    'rmw_cyclonedds_cpp': RMW_CYCLONEDDS_ENVIRONMENT_VARIABLES,
    'rmw_fastrtps_cpp': RMW_FASTRTPS_ENVIRONMENT_VARIABLES,
    'rmw_fastrtps_dynamic_cpp': RMW_FASTRTPS_ENVIRONMENT_VARIABLES,
    'rmw_zenoh_cpp': RMW_ZENOH_CPP_ENVIRONMENT_VARIABLES,
}


class DoctorCheck:
    """Abstract base class of ros2doctor check."""

    def category(self) -> str:
        """:return: string linking checks and reports."""
        raise NotImplementedError

    def check(self) -> 'Result':
        """:return: Result indicating result of checks."""
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
        self.items: List[Tuple[str, Union[str, int]]] = []

    def add_to_report(self, item_name: str, item_info: Union[int, str]) -> None:
        """Add report content to items list (list of string tuples)."""
        self.items.append((item_name, item_info))

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Report):
            return False
        return self.name == other.name and self.items == other.items

    def __str__(self) -> str:
        return f'{self.name} Report, Items: {self.items}'


class Result:
    """Stores check result."""

    __slots__ = ['error', 'warning']

    def __init__(self) -> None:
        """Initialize with no error or warning."""
        self.error = 0
        self.warning = 0

    def add_error(self) -> None:
        self.error += 1

    def add_warning(self) -> None:
        self.warning += 1


def run_checks(*, include_warnings: bool = False,
               exclude_packages: bool = False) -> Tuple[Set[str], int, int]:
    """
    Run all checks and return check results.

    :return: 3-tuple (categories of failed checks, number of failed checks,
             total number of checks)
    """
    fail_categories: Set[str] = set()  # remove repeating elements
    fail = 0
    total = 0
    entry_points = metadata.entry_points()
    if sys.version_info >= (3, 12):
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


def generate_reports(*, categories=None, exclude_packages: bool = False) -> List[Report]:
    """
    Print all reports or reports of failed checks to terminal.

    :return: list of Report objects
    """
    reports = []
    entry_points = metadata.entry_points()
    if sys.version_info >= (3, 12):
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


def print_warning_notice() -> None:
    print('\n' + '='*80)
    print('!!! WARNING !!!'.center(80))
    print('='*80)
    warning_message = [
        'The report includes all ROS 2 endpoint information and system platform information.',
        'Please review the report before sharing, as it may contain sensitive or private data.'
    ]
    for line in warning_message:
        print(line.center(80))
    print('='*80 + '\n')


def get_topic_names(skip_topics: List[str] = []) -> List[str]:
    """Get all topic names using rclpy API."""
    topics: List[str] = []
    with NodeStrategy(None) as node:
        topic_names_types = node.get_topic_names_and_types()
        for t_name, _ in topic_names_types:
            if t_name not in skip_topics:
                topics.append(t_name)
    return topics


def get_service_names(skip_services: List[str] = []) -> List[str]:
    """Get all service names using rclpy API."""
    services: List[str] = []
    with NodeStrategy(None) as node:
        service_names_types = node.get_service_names_and_types()
        for t_name, _ in service_names_types:
            if t_name not in skip_services:
                services.append(t_name)
    return services
