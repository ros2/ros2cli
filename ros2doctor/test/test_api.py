# Copyright 2020 Open Source Robotics Foundation, Inc.
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

import unittest

from common import generate_expected_service_report
from common import generate_expected_topic_report

from launch import LaunchDescription
from launch.actions import ExecuteProcess

import launch_testing.actions
import launch_testing.markers

import pytest

from ros2doctor.api.service import ServiceReport
from ros2doctor.api.topic import TopicCheck
from ros2doctor.api.topic import TopicReport


@pytest.mark.rostest
@launch_testing.markers.keep_alive
def generate_test_description() -> LaunchDescription:
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'daemon', 'stop'],
            name='daemon-stop',
            on_exit=[
                ExecuteProcess(
                    cmd=['ros2', 'daemon', 'start'],
                    name='daemon-start',
                    on_exit=[
                        launch_testing.actions.ReadyToTest()
                    ]
                )
            ]
        )
    ])


class TestROS2DoctorAPI(unittest.TestCase):

    def test_topic_check(self):
        """Assume no topics are publishing or subscribing other than whitelisted ones."""
        topic_check = TopicCheck()
        check_result = topic_check.check()
        self.assertEqual(check_result.error, 0)
        self.assertEqual(check_result.warning, 0)

    def test_no_topic_report(self):
        """Assume no topics are publishing or subscribing other than whitelisted ones."""
        report = TopicReport().report()
        expected_report = generate_expected_topic_report('none', 0, 0)
        self.assertEqual(report.name, expected_report.name)
        self.assertEqual(report.items, expected_report.items)
        self.assertEqual(report, expected_report)

    def test_no_service_report(self):
        """Assume no services are being used other than whitelisted ones."""
        report = ServiceReport().report()
        expected_report = generate_expected_service_report(['none'], [0], [0])
        self.assertEqual(report.name, expected_report.name)
        self.assertEqual(report.items, expected_report.items)
        self.assertEqual(report, expected_report)
