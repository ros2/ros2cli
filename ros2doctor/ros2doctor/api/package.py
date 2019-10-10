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
import xml.etree.ElementTree as ET

from ament_index_python import get_package_prefix
from ament_index_python.packages import PackageNotFoundError
from ros2doctor.api import DoctorCheck
from ros2doctor.api import DoctorReport
from ros2doctor.api import Report
from ros2doctor.api import Result
from ros2doctor.api.format import doctor_warn

import rosdistro


def _get_ros_packages_info() -> dict:
    """
    Return all current distro's packages info using rosdistro API.

    :return: a dictionary contains package name, release, source and status
    """
    distro_name = os.environ.get('ROS_DISTRO')
    if not distro_name:
        doctor_warn('ROS_DISTRO is not set.')
        return
    distro_name = distro_name.lower()
    url = rosdistro.get_index_url()
    if not url:
        doctor_warn('Unable to access ROSDISTRO_INDEX_URL or DEFAULT_INDEX_URL.')
        return
    i = rosdistro.get_index(url)
    distro_data = rosdistro.get_distribution(i, distro_name).get_data()
    return distro_data.get('repositories')


def _get_local_package_version(package_name) -> str:
    """
    Return all locally installed packages info.

    :return: a string contains local package version
    """
    try:
        prefix = get_package_prefix(package_name)
    except PackageNotFoundError:
        doctor_warn('Unable to find prefix for `%s` using index system.' % package_name)
        return
    xml_file = os.path.join(prefix, 'share', package_name, 'package.xml')
    try:
        root = ET.parse(xml_file).getroot()
        for child in root:
            if child.tag == 'version':
                return child.text
    except FileNotFoundError:
        doctor_warn('Unable to find package.xml of `%s`.' % package_name)


class PackageCheck(DoctorCheck):
    """Check local package versions against release versions on rosdistro."""

    def category(self):
        return 'package'

    def check(self):
        """Check packages within the directory where command is called."""
        result = Result()
        distro_packages_info = _get_ros_packages_info()
        if not distro_packages_info:
            result.add_error('ERROR: Unable to obtain current distro package information \
                from rosdistro.')
            return result
        for package_name, info in distro_packages_info.items():
            local_ver = _get_local_package_version(package_name)
            try:
                distro_release = info.get('release')
                distro_ver = distro_release.get('version')
            except AttributeError:
                result.add_warning('Unable to obtain the rosdistro version \
                    of `%s`.' % package_name)
            if local_ver and distro_ver:
                if distro_ver[:3] != local_ver[:3]:  # 0.8.0 vs. 0.8.0-1
                    result.add_warning('`%s` has different local version %s from \
                        required version %s' % (package_name, distro_ver, local_ver))
        return result


class PackageReport(DoctorReport):
    """Report local package versions and release versions on rosdistro."""

    def category(self):
        return 'package'

    def report(self):
        """Report packages within the directory where command is called."""
        report = Report('PACKAGE VERSIONS')
        return report
        pass
