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
from urllib.error import URLError

from ament_index_python import get_packages_with_prefixes
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

    :return: dictionary of rosdistro package name and version
    """
    distro_name = os.environ.get('ROS_DISTRO')
    distro_name = distro_name.lower()
    url = rosdistro.get_index_url()
    i = rosdistro.get_index(url)
    distro_data = rosdistro.get_distribution(i, distro_name).get_data()
    repos_info = distro_data.get('repositories')
    distro_package_vers = {}
    for package_name, info in repos_info.items():
        try:
            release = info['release']
            ver = release.get('version')
            if 'packages' in release:
                # Metapackage
                for package in release['packages']:
                    distro_package_vers[package] = ver
            else:
                distro_package_vers[package_name] = ver
        except KeyError:
            pass
    return distro_package_vers


def get_local_package_versions() -> dict:
    """
    Return local package name and versions.

    :return: dictionary of local package name and version
    """
    local_packages = {}
    package_name_prefixes = get_packages_with_prefixes()
    if package_name_prefixes:
        for name, prefix in package_name_prefixes.items():
            file_path = os.path.join(prefix, 'share', name)
            package_obj = parse_package(file_path)
            local_packages[name] = package_obj.version if package_obj.version else ''
    return local_packages


def compare_versions(local_packages: dict, distro_packages: dict) -> List:
    """
    Return warning messages for PackageCheck, and info for PackageReport.

    :param: dictionary of local package name and version
    :param: dictionary of rosdistro package name and version
    :param: boolean value determines which output to populate, msgs or report
    :return: list of warning messages
    """
    warning_msgs = []
    missing_req = ''
    missing_local = ''
    for name, local_ver_str in local_packages.items():
        if not local_ver_str:
            missing_local += ' ' + name
            local_ver_str = ''
        required_ver_str = distro_packages.get(name, '')
        if not required_ver_str:
            missing_req += ' ' + name
        local_ver = version.parse(local_ver_str).base_version
        required_ver = version.parse(required_ver_str).base_version
        if local_ver < required_ver:
            warning_msgs.append(f'{name} has been updated to a new version.'
                                f' local: {local_ver} <'
                                f' required: {required_ver}')
    if missing_req:
        warning_msgs.append('Cannot find required versions of packages:' + missing_req)
    if missing_local:
        warning_msgs.append('Cannot find local versions of packages:' + missing_local)
    return warning_msgs


class PackageCheck(DoctorCheck):
    """Check local package versions against release versions on rosdistro."""

    def category(self):
        return 'package'

    def check(self):
        """Check packages within the directory where command is called."""
        result = Result()
        try:
            distro_package_vers = get_distro_package_versions()
            if not distro_package_vers:
                result.add_error('ERROR: distro packages info is not found.')
        except (AttributeError, RuntimeError, URLError):
            result.add_error('ERROR: Unable to fetch package information from rosdistro.')
        local_package_vers = get_local_package_versions()
        if not local_package_vers:
            result.add_error('ERROR: local package info is not found.')
        if result.error != 0:
            return result

        warning_msgs = compare_versions(local_package_vers, distro_package_vers)
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
        try:
            distro_package_vers = get_distro_package_versions()
        except (AttributeError, RuntimeError, URLError):
            return report
        local_package_vers = get_local_package_versions()
        if not local_package_vers or not distro_package_vers:
            return report
        for name, local_ver_str in local_package_vers.items():
            required_ver_str = distro_package_vers.get(name, '')
            local_ver = version.parse(local_ver_str).base_version
            required_ver = version.parse(required_ver_str).base_version
            report.add_to_report(name, f'required={required_ver}, local={local_ver}')
        return report
