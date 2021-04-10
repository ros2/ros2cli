# Copyright 2021 Open Source Robotics Foundation, Inc.
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

from rclpy.qos import qos_check_compatible
from rclpy.qos import QoSCompatibility
from ros2cli.node.strategy import NodeStrategy
from ros2doctor.api import DoctorCheck
from ros2doctor.api import DoctorReport
from ros2doctor.api import get_topic_names
from ros2doctor.api import Report
from ros2doctor.api import Result
from ros2doctor.api.format import doctor_error
from ros2doctor.api.format import doctor_warn


class QoSCompatibilityCheck(DoctorCheck):
    """Check for incompatible QoS profiles in each pub/sub pair."""

    def category(self):
        return 'middleware'

    def check(self):
        """Check publisher and subscriber counts."""
        result = Result()
        to_be_checked = get_topic_names()
        with NodeStrategy(None) as node:
            for topic in to_be_checked:
                for pub in node.get_publishers_info_by_topic(topic):
                    for sub in node.get_subscriptions_info_by_topic(topic):
                        compatibility, reason = qos_check_compatible(
                            pub.qos_profile, sub.qos_profile)
                        if compatibility == QoSCompatibility.WARNING:
                            doctor_warn(f"QOS compatibility warning found on topic '{topic}'. "
                                        'Use `ros2 doctor --report` for more details.')
                            result.add_warning()
                        elif compatibility == QoSCompatibility.ERROR:
                            doctor_error(f"QOS compatibility error found on topic '{topic}'. "
                                         'Use `ros2 doctor --report` for more details.')
                            result.add_error()
        return result


class QoSCompatibilityReport(DoctorReport):
    """Report QoS compatibility related information."""

    def category(self):
        return 'topic'

    def report(self):
        report = Report('QOS COMPATIBILITY LIST')
        to_be_reported = get_topic_names()
        with NodeStrategy(None) as node:
            for topic in to_be_reported:
                for pub in node.get_publishers_info_by_topic(topic):
                    for sub in node.get_subscriptions_info_by_topic(topic):
                        compatibility, reason = qos_check_compatible(
                            pub.qos_profile, sub.qos_profile)
                        report.add_to_report('topic [type]', f'{topic} [{pub.topic_type}]')
                        report.add_to_report('publisher node', pub.node_name)
                        report.add_to_report('subscriber node', sub.node_name)
                        if compatibility == QoSCompatibility.OK:
                            compatibility_msg = 'OK'
                        else:
                            compatibility_msg = reason
                        report.add_to_report('compatibility status', compatibility_msg)
        return report
