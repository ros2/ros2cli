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

import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch.actions import UnsetEnvironmentVariable

import launch_testing.actions

import pytest

from rclpy.utilities import get_available_rmw_implementations
from ros2doctor.api import Report
from ros2doctor.api.environment import EnvironmentReport


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation: str):

    return LaunchDescription([
        # Always restart daemon to isolate tests.
        ExecuteProcess(
            cmd=['ros2', 'daemon', 'stop'],
            name='daemon-stop',
            on_exit=[
                ExecuteProcess(
                    cmd=['ros2', 'daemon', 'start'],
                    name='daemon-start',
                    on_exit=[
                        UnsetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE'),
                        UnsetEnvironmentVariable('ROS_DISTRO'),
                        SetEnvironmentVariable('RMW_IMPLEMENTATION', rmw_implementation),
                        SetEnvironmentVariable('ROS_HOME', 'BAR'),
                        SetEnvironmentVariable('ROS_LOG_DIR', 'BAZ'),
                        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', 'FOO'),
                        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', 'FOO'),
                        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'FASTBOO'),
                        SetEnvironmentVariable('ZENOH_CONFIG_OVERRIDE', 'ZENOHBOO'),
                        SetEnvironmentVariable('RMW_CONNEXT_INITIAL_PEERS', 'CONNEXTBOO'),
                        SetEnvironmentVariable('CYCLONEDDS_URI', 'CYCLONEBOO'),
                        launch_testing.actions.ReadyToTest()
                    ],
                )
            ]
        ),
    ])


class TestROS2Environment(unittest.TestCase):

    def test_environment_check_rmw_cyclonedds(self) -> None:
        self._check_env('rmw_cyclonedds_cpp', 'CYCLONEDDS_URI=CYCLONEBOO')

    def test_environment_check_rmw_connext(self) -> None:
        self._check_env('rmw_connext_cpp', 'RMW_CONNEXT_INITIAL_PEERS=CONNEXTBOO')

    def test_environment_check_rmw_zenoh(self) -> None:
        self._check_env('rmw_zenoh_cpp', 'ZENOH_CONFIG_OVERRIDE=ZENOHBOO')

    def test_environment_check_rmw_fastrtps(self) -> None:
        self._check_env('rmw_fastrtps_cpp', 'FASTDDS_BUILTIN_TRANSPORTS=FASTBOO')

    def _check_env(self, rmw: str, expected_rmw_vars_line: str) -> None:
        environment_report = EnvironmentReport().report()
        expected_report = Report('ROS ENVIRONMENT')
        expected_report.add_to_report('ros environment variables',
                                      f'ROS_HOME=BAR, ROS_LOG_DIR=BAZ, RMW_IMPLEMETNATION={rmw}')
        expected_report.add_to_report('rcutils environment variables',
                                      'RCUTILS_COLORIZED_OUTPUT=FOO')
        expected_report.add_to_report(f'{rmw} environment variables', expected_rmw_vars_line)
        self.assertEqual(environment_report.name, expected_report.name)
        self.assertEqual(environment_report.items, expected_report.items)
        self.assertEqual(environment_report, expected_report)
