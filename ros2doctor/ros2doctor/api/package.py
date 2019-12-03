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
from typing import List
from typing import Tuple
from urllib.error import URLError

from ament_index_python import get_packages_with_prefixes
from catkin_pkg.package import InvalidPackage
from catkin_pkg.package import parse_package
from packaging import version

from ros2doctor.api import DoctorCheck
from ros2doctor.api import DoctorReport
from ros2doctor.api import Report
from ros2doctor.api import Result

import rosdistro


def get_distro_package_versions() -> dict:
    """
    Return repos info using rosdistro API.

    :return: a dictionary contains package name, release, source and status
    """
    distro_name = os.environ.get('ROS_DISTRO')
    distro_name = distro_name.lower()
    url = rosdistro.get_index_url()
    i = rosdistro.get_index(url)
    distro_data = rosdistro.get_distribution(i, distro_name).get_data()
    repos_info = distro_data.get('repositories')
    distro_package_vers = {}
    if repos_info:
        for _, info in repos_info.items():
            release = info.get('release')
            if release:
                version = release.get('version')
                packages = release.get('packages')
                if packages:
                    for p in packages:
                        distro_package_vers[p] = version
    return distro_package_vers


def compare_versions(local_prefixes: dict,
                     distro_packages: dict,
                     isCheck=True) -> Tuple[List, dict]:
    warning_msgs = []
    report_dict = {}
    missing_req = ''
    missing_local = ''
    for name, prefix in local_prefixes.items():
        try:
            required_ver_str = distro_packages.get(name)
            required_ver = version.parse(required_ver_str).base_version
        except TypeError:
            missing_req += ' ' + name
            required_ver = version.parse('').base_version
        file_path = os.path.join(prefix, 'share', name)
        try:
            package_obj = parse_package(file_path)
            local_ver_str = package_obj.version
            local_ver = version.parse(local_ver_str).base_version
        except (AttributeError, IOError, InvalidPackage):
            missing_local += ' ' + name
            local_ver = version.parse('').base_version
        if isCheck:
            if local_ver < required_ver:
                warning_msgs.append(f'{name} is deprecated.'
                                    f' local: {local_ver} <'
                                    f' required: {required_ver}')
        else:
            report_dict[name] = f'required={required_ver}, local={local_ver}'
    if missing_req:
        warning_msgs.append('Cannot find required versions of packages:' + missing_req)
    if missing_local:
        warning_msgs.append('Cannot find local versions of packages:' + missing_local)
    return warning_msgs, report_dict


class PackageCheck(DoctorCheck):
    """Check local package versions against release versions on rosdistro."""

    def category(self):
        return 'package'

    def check(self):
        """Check packages within the directory where command is called."""
        result = Result()
        local_prefixes = get_packages_with_prefixes()
        if not local_prefixes:
            result.add_error('ERROR: local package info is not found.')
        try:
            distro_package_vers = get_distro_package_versions()
            if not distro_package_vers:
                result.add_error('ERROR: distro packages info is not found.')
        except (AttributeError, RuntimeError, URLError):
            result.add_error('ERROR: Unable to fetch package information from rosdistro.')
        if result.error != 0:
            return result

        warning_msgs, _ = compare_versions(local_prefixes,
                                           distro_package_vers,
                                           isCheck=True)
        for msg in warning_msgs:
            result.add_warning(msg)
        return result


class PackageReport(DoctorReport):
    """Report local package versions and release versions on rosdistro."""

    def category(self):
        return 'package'

    def report(self):
        """Report packages within the directory where command is called."""
        report = Report('PACKAGE VERSIONS')
        local_prefixes = get_packages_with_prefixes()
        try:
            distro_package_vers = get_distro_package_versions()
        except (AttributeError, RuntimeError, URLError):
            return report

        if local_prefixes and distro_package_vers:
            _, report_dict = compare_versions(local_prefixes,
                                              distro_package_vers,
                                              isCheck=False)
            for name, item in report_dict.items():
                report.add_to_report(name, item)
        return report
