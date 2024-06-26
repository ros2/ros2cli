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
import platform
from typing import Tuple

from ros2doctor.api import DoctorCheck
from ros2doctor.api import DoctorReport
from ros2doctor.api import Report
from ros2doctor.api import Result
from ros2doctor.api.format import doctor_error
from ros2doctor.api.format import doctor_warn

import rosdistro


def _check_platform_helper() -> Tuple[str, dict, dict]:
    """
    Check ROS_DISTRO environment variables and distribution installed.

    :return: string of distro name, dict of distribution info, dict of release platforms info
    """
    distro_name = os.environ.get('ROS_DISTRO')
    if not distro_name:
        doctor_error('ROS_DISTRO is not set.')
        return
    distro_name = distro_name.lower()
    u = rosdistro.get_index_url()
    if not u:
        doctor_error(
            'Unable to access ROSDISTRO_INDEX_URL or DEFAULT_INDEX_URL. '
            'Check network setting to make sure machine is connected to internet.')
        return
    i = rosdistro.get_index(u)
    distro_info = i.distributions.get(distro_name)
    if not distro_info:
        doctor_warn(f'Distribution name {distro_name} is not found')
        return
    try:
        distro_data = rosdistro.get_distribution(i, distro_name).get_data()
    except AttributeError:
        distro_data = ''
    return distro_name, distro_info, distro_data


class PlatformCheck(DoctorCheck):
    """Check system platform against ROSDistro."""

    def category(self):
        return 'platform'

    def check(self):
        """Check system platform against ROS 2 Distro."""
        result = Result()
        distros = _check_platform_helper()
        if not distros:
            doctor_error('Missing rosdistro info. Unable to check platform.')
            result.add_error()
            return result
        distro_name, distro_info, _ = distros

        # check distro status
        if distro_info.get('distribution_status') == 'prerelease':
            doctor_warn(
                f'Distribution {distro_name} is not fully supported or tested. '
                'To get more consistent features, download a stable version at '
                'https://docs.ros.org')
            result.add_warning()
        elif distro_info.get('distribution_status') == 'end-of-life':
            doctor_warn(
                f'Distribution {distro_name} is no longer supported or deprecated. '
                'To get the latest features, download the new versions at '
                'https://docs.ros.org')
            result.add_warning()
        return result


class PlatformReport(DoctorReport):
    """Output platform report."""

    def category(self):
        return 'platform'

    def report(self):
        platform_name = platform.system()

        # platform info
        platform_report = Report('PLATFORM INFORMATION')
        platform_report.add_to_report('system', platform_name)
        platform_report.add_to_report('platform info', platform.platform())
        if platform_name == 'Darwin':
            platform_report.add_to_report('mac OS version', platform.mac_ver())
        platform_report.add_to_report('release', platform.release())
        platform_report.add_to_report('processor', platform.processor())
        return platform_report


class RosdistroReport(DoctorReport):
    """Output ROSDistro report."""

    def category(self):
        return 'platform'

    def report(self):
        ros_report = Report('ROS 2 INFORMATION')
        distros = _check_platform_helper()
        if not distros:
            return ros_report
        distro_name, distro_info, distro_data = distros
        ros_report.add_to_report('distribution name', distro_name)
        ros_report.add_to_report('distribution type', distro_info.get('distribution_type'))
        ros_report.add_to_report('distribution status', distro_info.get('distribution_status'))
        ros_report.add_to_report('release platforms', distro_data.get('release_platforms'))
        return ros_report
