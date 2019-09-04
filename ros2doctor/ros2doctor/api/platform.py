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
import sys
import warnings

from ros2doctor.api import DoctorCheck
from ros2doctor.api import DoctorReport
from ros2doctor.api import Report
from ros2doctor.api.format import warning_format

warnings.formatwarning = warning_format

import rosdistro


def _check_platform_helper():
    """Check ROS_DISTRO related environment variables and distribution name."""
    distro_name = os.environ.get('ROS_DISTRO')
    if not distro_name:
        warnings.warn('ROS_DISTRO is not set.')
        return
    else:
        distro_name = distro_name.lower()
    u = rosdistro.get_index_url()
    if not u:
        warnings.warn('Unable to access ROSDISTRO_INDEX_URL or DEFAULT_INDEX_URL.')
        return
    i = rosdistro.get_index(u)
    distro_info = i.distributions.get(distro_name)
    if not distro_info:
        warnings.warn("Distribution name '%s' is not found" % distro_name)
        return
    distro_data = rosdistro.get_distribution(i, distro_name).get_data()
    return distro_name, distro_info, distro_data


class PlatformCheck(DoctorCheck):
    """Check system platform against ROSDistro."""

    def category(self):
        return 'platform'

    def check(self):
        """Check system platform against ROS 2 Distro."""
        result = False
        distros = _check_platform_helper()
        if not distros:
            return result
        distro_name, distro_info, _ = distros

        # check distro status
        if distro_info.get('distribution_status') == 'prerelease':
            warnings.warn('Distribution %s is not fully supported or tested. '
                             'To get more consistent features, download a stable version at '
                             'https://index.ros.org/doc/ros2/Installation/' % distro_name)
        elif distro_info.get('distribution_status') == 'end-of-life':
            warnings.warn('Distribution %s is no longer supported or deprecated. '
                             'To get the latest features, download the new versions at '
                             'https://index.ros.org/doc/ros2/Installation/' % distro_name)
        else:
            result = True
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
        distros = _check_platform_helper()
        if not distros:
            return
        distro_name, distro_info, distro_data = distros
        ros_report = Report('ROS 2 INFORMATION')
        ros_report.add_to_report('distribution name', distro_name)
        ros_report.add_to_report('distribution type', distro_info.get('distribution_type'))
        ros_report.add_to_report('distribution status', distro_info.get('distribution_status'))
        ros_report.add_to_report('release platforms', distro_data.get('release_platforms'))
        return ros_report
