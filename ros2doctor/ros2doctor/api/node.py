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


from collections import Counter

from ros2node.api import get_node_names
from ros2cli.node.strategy import NodeStrategy
from ros2doctor.api import DoctorCheck
from ros2doctor.api import DoctorReport
from ros2doctor.api import Report
from ros2doctor.api import Result
from ros2doctor.api.format import doctor_warn


def find_duplicates(values):
    """Return values that appear more than once."""
    counts = Counter(values)
    return [v for v, c in counts.items() if c > 1]


class NodeCheck(DoctorCheck):
    """Check for duplicate node names."""

    def category(self):
        return 'node'

    def check(self):
        result = Result()
        with NodeStrategy(None) as node:
            node_list = get_node_names(node=node, include_hidden_nodes=True)
            node_names = [n.full_name for n in node_list]
            duplicates = find_duplicates(node_names)
            for duplicate in duplicates:
                doctor_warn(f'Duplicate node name: {duplicate}')
                result.add_warning()
        return result


class NodeReport(DoctorReport):
    """Report node related information."""

    def category(self):
        return 'node'

    def report(self):
        report = Report('NODE LIST')
        with NodeStrategy(None) as node:
            node_list = get_node_names(node=node, include_hidden_nodes=True)
            node_names = [n.full_name for n in node_list]
            if not node_names:
                report.add_to_report('node count', 0)
                report.add_to_report('node', 'none')
            else:
                report.add_to_report('node count', len(node_names))
                for node_name in sorted(node_names):
                    report.add_to_report('node', node_name)
        return report
