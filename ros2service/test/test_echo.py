# Copyright 2023 Open Source Robotics Foundation, Inc.
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

import contextlib
import functools
import os
import re
import sys
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
import launch_testing_ros.tools

import pytest

from rclpy.utilities import get_available_rmw_implementations


# Skip cli tests on Windows while they exhibit pathological behavior
# https://github.com/ros2/build_farmer/issues/248
if sys.platform.startswith('win'):
    pytest.skip(
            'CLI tests can block for a pathological amount of time on Windows.',
            allow_module_level=True)


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation):
    path_to_introspectable_script = os.path.join(
        os.path.dirname(__file__), 'fixtures', 'introspectable.py'
    )
    additional_env = {'RMW_IMPLEMENTATION': rmw_implementation}
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
                        # Add test fixture actions.
                        Node(
                            executable=sys.executable,
                            arguments=[path_to_introspectable_script],
                            name='introspectable_service',
                            additional_env=additional_env,
                        ),
                        launch_testing.actions.ReadyToTest()
                    ],
                    additional_env=additional_env
                )
            ]
        ),
    ])


class TestROS2ServiceEcho(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output,
        rmw_implementation
    ):
        @contextlib.contextmanager
        def launch_service_command(self, arguments):
            service_command_action = ExecuteProcess(
                cmd=['ros2', 'service', *arguments],
                additional_env={
                    'RMW_IMPLEMENTATION': rmw_implementation,
                    'PYTHONUNBUFFERED': '1'
                },
                name='ros2service-echo',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, service_command_action, proc_info, proc_output,
                output_filter=launch_testing_ros.tools.basic_output_filter(
                    filtered_prefixes=[
                        'waiting for service to become available...',
                        '/launch_ros'  # cope with launch_ros internal node.
                    ],
                    filtered_rmw_implementation=rmw_implementation
                )
            ) as service_command:
                yield service_command
        cls.launch_service_command = launch_service_command

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_echo_no_arr(self):
        echo_arguments = ['echo', '/test_introspectable', '--no-arr']
        expected_output = [
            'info:',
            '  event_type: 0',
            '  stamp:',
            re.compile(r'    sec: \d+'),
            re.compile(r'    nanosec: \d+'),
            "  client_gid: '<array type: uint8[16]>'",
            re.compile(r'  sequence_number: \d+'),
            "request: '<sequence type: test_msgs/srv/BasicTypes_Request[1], length: 1>'",
            "response: '<sequence type: test_msgs/srv/BasicTypes_Response[1], length: 0>'",
            '---',
            'info:',
            '  event_type: 1',
            '  stamp:',
            re.compile(r'    sec: \d+'),
            re.compile(r'    nanosec: \d+'),
            "  client_gid: '<array type: uint8[16]>'",
            re.compile(r'  sequence_number: \d+'),
            "request: '<sequence type: test_msgs/srv/BasicTypes_Request[1], length: 1>'",
            "response: '<sequence type: test_msgs/srv/BasicTypes_Response[1], length: 0>'",
            '---',
            'info:',
            '  event_type: 2',
            '  stamp:',
            re.compile(r'    sec: \d+'),
            re.compile(r'    nanosec: \d+'),
            "  client_gid: '<array type: uint8[16]>'",
            re.compile(r'  sequence_number: \d+'),
            "request: '<sequence type: test_msgs/srv/BasicTypes_Request[1], length: 0>'",
            "response: '<sequence type: test_msgs/srv/BasicTypes_Response[1], length: 1>'",
            '---',
            'info:',
            '  event_type: 3',
            '  stamp:',
            re.compile(r'    sec: \d+'),
            re.compile(r'    nanosec: \d+'),
            "  client_gid: '<array type: uint8[16]>'",
            re.compile(r'  sequence_number: \d+'),
            "request: '<sequence type: test_msgs/srv/BasicTypes_Request[1], length: 0>'",
            "response: '<sequence type: test_msgs/srv/BasicTypes_Response[1], length: 1>'",
        ],

        with self.launch_service_command(arguments=echo_arguments) as service_command:
            assert service_command.wait_for_output(
                functools.partial(
                    launch_testing.tools.expect_output,
                    expected_lines=expected_output,
                    strict=True
                ),
                timeout=10,
            )
        assert service_command.wait_for_shutdown(timeout=10)
