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

from ros2action.api import get_action_clients_and_servers
from ros2action.api import get_action_names
from ros2cli.node.direct import DirectNode
from ros2doctor.api import DoctorReport
from ros2doctor.api import Report


class ActionReport(DoctorReport):
    """Report service related information."""

    def category(self) -> Literal['action']:
        return 'action'

    def report(self) -> Report:
        report = Report('ACTION LIST')
        with DirectNode(None) as node:
            to_be_reported = get_action_names(node=node)
            if not to_be_reported:
                report.add_to_report('action', 'none')
                report.add_to_report('action server count', 0)
                report.add_to_report('action client count', 0)
                return report

            for action_name in to_be_reported:
                clients, server = get_action_clients_and_servers(node=node,
                                                                 action_name=action_name)
                report.add_to_report('action', action_name)
                report.add_to_report('action server count', len(clients))
                report.add_to_report('action client count', len(server))

        return report
