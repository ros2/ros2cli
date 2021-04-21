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
from pathlib import Path
import sys
import textwrap

from ament_index_python import get_packages_with_prefixes
from catkin_pkg.package import parse_package
from packaging import version

from ros2doctor.api import DoctorCheck
from ros2doctor.api import DoctorReport
from ros2doctor.api import Report
from ros2doctor.api import Result
from ros2doctor.api.format import doctor_error
from ros2doctor.api.format import doctor_warn

try:
    import rosdistro
except ImportError:
    doctor_warn(
        'Unable to import rosdistro. '
        f'Use `{Path(sys.executable).name} -m pip install rosdistro` to install needed package.')


def get_distro_package_versions() -> dict:
    """
    Return repos info using rosdistro API.

    :return: dictionary of rosdistro package name and version
    """
    distro_name = os.environ.get('ROS_DISTRO')
    if not distro_name:
        doctor_error('ROS_DISTRO is not set.')
        return
    distro_name = distro_name.lower()
    url = rosdistro.get_index_url()
    if not url:
        doctor_error(
            'Unable to access ROSDISTRO_INDEX_URL or DEFAULT_INDEX_URL. '
            'Check network setting to make sure machine is connected to internet.')
        return
    i = rosdistro.get_index(url)
    distro_info = rosdistro.get_distribution(i, distro_name)
    if not distro_info:
        doctor_warn(f'Distribution name {distro_name} is not found')
        return
    try:
        repos_info = distro_info.get_data().get('repositories')
    except AttributeError:
        doctor_warn('No repository information found.')
        return
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


def compare_versions(result: Result, local_packages: dict, distro_packages: dict):
    """
    Return warning messages for PackageCheck, and info for PackageReport.

    :param: dictionary of local package name and version
    :param: dictionary of rosdistro package name and version
    :param: boolean value determines which output to populate, msgs or report
    :return: list of warning messages
    """
    missing_req = ''
    missing_local = ''
    for name, local_ver_str in local_packages.items():
        if not local_ver_str:
            missing_local += ' ' + name
            local_ver_str = ''
        latest_ver_str = distro_packages.get(name, '')
        if not latest_ver_str:
            missing_req += ' ' + name
        local_ver = version.parse(local_ver_str).base_version
        latest_ver = version.parse(latest_ver_str).base_version
        if local_ver < latest_ver:
            doctor_warn(
                f'{name} has been updated to a new version.'
                f' local: {local_ver} <'
                f' latest: {latest_ver}')
            result.add_warning()
    if missing_req:
        if len(missing_req) > 100:
            doctor_warn(
                'Cannot find the latest versions of packages: ' +
                textwrap.shorten(missing_req, width=100) +
                ' Use `ros2 doctor --report` to see full list.')
        else:
            doctor_warn(
                'Cannot find the latest versions of packages: ' +
                missing_req)
    if missing_local:
        if len(missing_local) > 100:
            doctor_warn(
                'Cannot find local versions of packages: ' +
                textwrap.shorten(missing_local, width=100) +
                ' Use `ros2 doctor --report` to see full list.')
        else:
            doctor_warn(
                'Cannot find local versions of packages: ' +
                missing_local)


class PackageCheck(DoctorCheck):
    """Check local package versions against release versions on rosdistro."""

    def category(self):
        return 'package'

    def check(self):
        """Check packages within the directory where command is called."""
        result = Result()
        try:
            rosdistro
        except NameError:
            doctor_error('`rosdistro` module is not imported. Unable to run package check.')
            result.add_error()
            return result
        distro_package_vers = get_distro_package_versions()
        if not distro_package_vers:
            doctor_error('distro packages info is not found.')
            result.add_error()
        local_package_vers = get_local_package_versions()
        if not local_package_vers:
            doctor_error('local package info is not found.')
            result.add_error()
        if result.error != 0:
            return result

        compare_versions(result, local_package_vers, distro_package_vers)
        return result


class PackageReport(DoctorReport):
    """Report local package versions and release versions on rosdistro."""

    def category(self):
        return 'package'

    def report(self):
        """Report packages within the directory where command is called."""
        try:
            rosdistro
        except NameError:
            doctor_error('`rosdistro` module is not imported. Unable to generate package report.')
            return Report('')
        report = Report('PACKAGE VERSIONS')
        local_package_vers = get_local_package_versions()
        distro_package_vers = get_distro_package_versions()
        if not local_package_vers or not distro_package_vers:
            return report
        for name, local_ver_str in local_package_vers.items():
            latest_ver_str = distro_package_vers.get(name, '')
            local_ver = version.parse(local_ver_str).base_version
            latest_ver = version.parse(latest_ver_str).base_version
            report.add_to_report(name, f'latest={latest_ver}, local={local_ver}')
        return report
