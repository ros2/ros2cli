# Copyright 2025 Open Source Robotics Foundation, Inc.
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
from typing import Literal

from rclpy.utilities import get_rmw_implementation_identifier
from ros2doctor.api import DoctorReport
from ros2doctor.api import RCUTILS_ENVIRONMENT_VARIABLES
from ros2doctor.api import Report
from ros2doctor.api import RMW_ENVIRONMENT_VARIABLES
from ros2doctor.api import ROS_ENVIRONMENT_VARIABLES


class EnvironmentReport(DoctorReport):
    """Report current ROS and RMW environment variable information."""

    def category(self) -> Literal['environment']:
        return 'environment'

    def report(self) -> Report:
        environment_report = Report('ROS ENVIRONMENT')

        rmw_name = get_rmw_implementation_identifier()

        rmw_environment_variables: list[str] = RMW_ENVIRONMENT_VARIABLES.get(rmw_name, [])

        ros_variable_list: list[str] = []
        rmw_variable_list: list[str] = []
        rcutils_variable_list: list[str] = []

        for key, value in sorted(os.environ.items()):
            if key in ROS_ENVIRONMENT_VARIABLES:
                ros_variable_list.append(f'{key}={value}')
            if key in RCUTILS_ENVIRONMENT_VARIABLES:
                rcutils_variable_list.append(f'{key}={value}')
            if key in rmw_environment_variables:
                rmw_variable_list.append(f'{key}={value}')

        environment_report.add_to_report('ROS environment variables',
                                         ', '.join(ros_variable_list))
        environment_report.add_to_report('rcutils environment variables',
                                         ', '.join(rcutils_variable_list))

        if rmw_environment_variables:
            environment_report.add_to_report(
                'rmw environment variables',
                ', '.join(rmw_variable_list))
        return environment_report
