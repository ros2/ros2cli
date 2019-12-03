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


def get_ros_repos_info() -> dict:
    """
    Return repos info using rosdistro API.

    :return: a dictionary contains package name, release, source and status
    """
    distro_name = os.environ.get('ROS_DISTRO')
    distro_name = distro_name.lower()
    url = rosdistro.get_index_url()
    i = rosdistro.get_index(url)
    distro_data = rosdistro.get_distribution(i, distro_name).get_data()
    return distro_data.get('repositories')


def get_packages_versions(distro_repos_info: dict) -> dict:
    """
    Return all package names and corresponding versions in rosdistro.

    :param: a dictionary of repo info
    :return: a dictionary contains package names and versions
    """
    packages_versions = {}
    for _, info in distro_repos_info.items():
        release = info.get('release')
        if release:
            version = release.get('version')
            packages = release.get('packages')
            if packages:
                for p in packages:
                    packages_versions[p] = version
    return packages_versions


class PackageCheck(DoctorCheck):
    """Check local package versions against release versions on rosdistro."""

    def category(self):
        return 'package'

    def check(self):
        """Check packages within the directory where command is called."""
        result = Result()
        local_packages_prefixes = get_packages_with_prefixes()
        if not local_packages_prefixes:
            result.add_error('ERROR: local package info is not found.')
        try:
            distro_repos_info = get_ros_repos_info()
            packages_versions = get_packages_versions(distro_repos_info)
            if not packages_versions:
                result.add_error('ERROR: distro packages info is not found.')
        except (AttributeError, RuntimeError, URLError):
            result.add_error('ERROR: Unable to fetch package information from rosdistro.')
        if result.error != 0:
            return result
        for package_name, package_prefix in local_packages_prefixes.items():
            file_path = os.path.join(package_prefix, 'share', package_name)
            try:
                required_ver_str = packages_versions.get(package_name)
                required_ver = version.parse(required_ver_str).base_version
            except TypeError:
                result.add_warning(f'{package_name}: No required version found.')
            try:
                package_obj = parse_package(file_path)
                local_ver_str = package_obj.version
            except (AttributeError, IOError, InvalidPackage):
                result.add_warning(f'Unable to parse {package_name} package.xml file.')
                local_ver_str = ''
            try:
                local_ver = version.parse(local_ver_str).base_version
            except TypeError:
                result.add_warning(f'{package_name}: no local version found.')
            if local_ver < required_ver:
                result.add_warning('{package_name} is deprecated.'
                                   f'local: {local_ver} < required: {required_ver}')
        return result


class PackageReport(DoctorReport):
    """Report local package versions and release versions on rosdistro."""

    def category(self):
        return 'package'

    def report(self):
        """Report packages within the directory where command is called."""
        report = Report('PACKAGE VERSIONS')
        local_package_prefixes = get_packages_with_prefixes()
        try:
            distro_repos_info = get_ros_repos_info()
            packages_versions = get_packages_versions(distro_repos_info)
        except:
            return report
        if local_package_prefixes and packages_versions:
            for name, prefix in local_package_prefixes.items():
                required_ver = packages_versions.get(name)
                file_path = os.path.join(prefix, 'share', name)
                try:
                    package_obj = parse_package(file_path)
                    local_ver = package_obj.version
                except:
                    local_ver = ''
                report.add_to_report(name, 'required=' +
                                    (required_ver if required_ver else '') +
                                    ', local=' + local_ver)
        return report
