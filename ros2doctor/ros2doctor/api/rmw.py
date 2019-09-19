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

from rclpy.utilities import get_rmw_implementation_identifier
from ros2doctor.api import DoctorReport
from ros2doctor.api import Report


class RMWReport(DoctorReport):
    """Report current RMW information."""

    def category(self):
        return 'middleware'

    def report(self):
        rmw_report = Report('RMW Middleware')
        rmw_report.add_to_report('middleware name', get_rmw_implementation_identifier())
        return rmw_report
