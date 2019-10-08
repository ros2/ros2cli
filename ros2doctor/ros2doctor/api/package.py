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

import os

from ros2doctor.api import DoctorCheck
from ros2doctor.api import DoctorReport
from ros2doctor.api import Report
from ros2doctor.api import Result
from ros2doctor.api.format import doctor_warn

import rosdistro
from vcstool.commands.command import add_common_arguments
from vcstool.commands.export import get_parser
from vcstool.commands.export import output_export_data
from vcstool.crawler import find_repositories
from vcstool.executor import generate_jobs
from vcstool.executor import execute_jobs


def _get_ros_package_info() -> dict:
    """
    Return all current distro's packages info using rosdistro API.

    :return: a dictionary contains package name, release, source and status
    """
    distro_name = os.environ.get('ROS_DISTRO')
    if not distro_name:
        doctor_warn('ROS_DISTRO is not set.')
        return
    else:
        distro_name = distro_name.lower()
    u = rosdistro.get_index_url()
    if not u:
        doctor_warn('Unable to access ROSDISTRO_INDEX_URL or DEFAULT_INDEX_URL.')
        return
    i = rosdistro.get_index(u)
    distro_data = rosdistro.get_distribution(i, distro_name).get_data()
    return distro_data.get('repositories')


def _get_local_package_info() -> dict:
    """
    Return all locally installed packages info using vcstool API.

    :return: a dictionary contains package name, type, url, version
    """
    pass


class PackageCheck(DoctorCheck):
    """Check local package versions against release versions on rosdistro."""

    def category(self):
        return 'package'

    def check(self):
        result = Result()
        pass


class PackageReport(DoctorReport):
    """Report local package versions and release versions on rosdistro."""

    def category(self):
        return 'package'

    def report(self):
        report = Report('PACKAGE VERSIONS')
        pass
