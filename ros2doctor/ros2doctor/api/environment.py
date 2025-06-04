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
from typing import List

from ros2doctor.api import DoctorReport
from ros2doctor.api import Report


class EnvironmentReport(DoctorReport):
    """Report current ROS and RMW environment variable information."""

    def category(self) -> Literal['environment']:
        return 'environment'

    def report(self) -> Report:
        environment_report = Report('ROS ENVIRONMENT')

        ros_variable_list: List[str] = []
        rmw_variable_list: List[str] = []
        rcutils_variable_list: List[str] = []

        for key, value in os.environ.items():
            if "ROS" in key:
                ros_variable_list.append(f'{key}={value}')
            if "RMW" in key:
                rmw_variable_list.append(f'{key}={value}')
            if "RCUTILS" in key:
                rcutils_variable_list.append(f'{key}={value}')

        environment_report.add_to_report('ros environment variables', ", ".join(ros_variable_list))
        environment_report.add_to_report('rcutils environment variables', ", ".join(rcutils_variable_list))
        environment_report.add_to_report('rmw environment variables', ", ".join(rmw_variable_list))
        return environment_report
