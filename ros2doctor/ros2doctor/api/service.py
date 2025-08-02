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

from typing import Literal

from ros2cli.node.direct import DirectNode
from ros2doctor.api import DoctorReport
from ros2doctor.api import get_service_names
from ros2doctor.api import Report


class ServiceReport(DoctorReport):
    """Report service related information."""

    def category(self) -> Literal['service']:
        return 'service'

    def report(self) -> Report:
        report = Report('SERVICE LIST')
        to_be_reported = get_service_names()
        if not to_be_reported:
            report.add_to_report('service', 'none')
            report.add_to_report('service count', 0)
            report.add_to_report('client count', 0)
        with DirectNode(None) as node:
            for service_name in to_be_reported:
                report.add_to_report('service', service_name)
                report.add_to_report('service count', node.node.count_services(service_name))
                report.add_to_report('client count', node.node.count_clients(service_name))
        return report
