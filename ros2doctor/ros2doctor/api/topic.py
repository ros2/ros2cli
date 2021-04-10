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

from typing import List

from ros2cli.node.direct import DirectNode
from ros2doctor.api import DoctorCheck
from ros2doctor.api import DoctorReport
from ros2doctor.api import get_topic_names
from ros2doctor.api import Report
from ros2doctor.api import Result
from ros2doctor.api.format import doctor_warn


class TopicCheck(DoctorCheck):
    """Check for pub without sub or sub without pub."""

    def category(self):
        return 'topic'

    def check(self):
        """Check publisher and subscriber counts."""
        result = Result()
        to_be_checked = get_topic_names()
        with DirectNode(None) as node:
            for topic in to_be_checked:
                pub_count = node.count_publishers(topic)
                sub_count = node.count_subscribers(topic)
                if pub_count > 0 and sub_count == 0:
                    doctor_warn(f'Publisher without subscriber detected on {topic}.')
                    result.add_warning()
                elif sub_count > 0 and pub_count == 0:
                    doctor_warn(f'Subscriber without publisher detected on {topic}.')
                    result.add_warning()
        return result


class TopicReport(DoctorReport):
    """Report topic related information."""

    def category(self):
        return 'topic'

    def report(self):
        report = Report('TOPIC LIST')
        to_be_reported = get_topic_names()
        if not to_be_reported:
            report.add_to_report('topic', 'none')
            report.add_to_report('publisher count', 0)
            report.add_to_report('subscriber count', 0)
        with DirectNode(None) as node:
            for topic in to_be_reported:
                report.add_to_report('topic', topic)
                report.add_to_report('publisher count', node.count_publishers(topic))
                report.add_to_report('subscriber count', node.count_subscribers(topic))
        return report
