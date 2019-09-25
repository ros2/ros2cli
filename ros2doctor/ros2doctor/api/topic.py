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
from ros2cli.node.strategy import NodeStrategy
from ros2doctor.api import DoctorCheck
from ros2doctor.api import DoctorReport
from ros2doctor.api import Report
from ros2doctor.api import Result
from ros2topic.api import get_topic_names


def _get_topics() -> List:
    """Get all topic names using ros2topic API."""
    white_list = ['/parameter_events', '/rosout']
    topics = []
    with NodeStrategy('') as node:
        all_topic_names = get_topic_names(node=node)
        for t_name in all_topic_names:
            if t_name not in white_list:
                topics.append(t_name)
    return topics


class TopicCheck(DoctorCheck):
    """Check for pub without sub or sub without pub."""

    def category(self):
        return 'topic'

    def check(self):
        """Check publisher and subscriber counts."""
        result = Result()
        to_be_checked = _get_topics()
        for topic in to_be_checked:
            with DirectNode(topic) as node:
                pub_count = node.count_publishers(topic)
                sub_count = node.count_subscribers(topic)
                if pub_count > sub_count:
                    result.add_warning('Publisher without subscriber detected on %s.' % topic)
                elif pub_count < sub_count:
                    result.add_warning('Subscriber without publisher detected on %s.' % topic)
        return result


class TopicReport(DoctorReport):
    """Report `ros2 topic info` output."""

    def category(self):
        return 'topic'

    def report(self):
        report = Report('TOPIC LIST')
        to_be_reported = _get_topics()
        if not to_be_reported:
            report.add_to_report('topic', 'none')
        for topic in to_be_reported:
            with DirectNode(topic) as node:
                report.add_to_report('topic', topic)
                report.add_to_report('publisher count', node.count_publishers(topic))
                report.add_to_report('subscriber count', node.count_subscribers(topic))
        return report
