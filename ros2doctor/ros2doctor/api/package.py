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

from ros2doctor.api import DoctorCheck
from ros2doctor.api import DoctorReport
from ros2doctor.api import Report
from ros2doctor.api import Result

import rosdistro


def _get_ros_package_info() -> dict:
    """
    Return all of current distro's package information.
    
    :return: a dictionary contains package names, release, source and status info
    """
    pass


def _get_local_package_info() -> dict:
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
