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

import contextlib
import os
import re
import sys
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
import launch_testing_ros.tools

import pytest

from rclpy.utilities import get_available_rmw_implementations

import yaml


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation):
    path_to_action_server_executable = os.path.join(
        os.path.dirname(__file__), 'fixtures', 'fibonacci_action_server.py'
    )
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
                        ExecuteProcess(
                            cmd=[sys.executable, path_to_action_server_executable],
                            additional_env={'RMW_IMPLEMENTATION': rmw_implementation}
                        ),
                        launch_testing.actions.ReadyToTest()
                    ],
                    additional_env={'RMW_IMPLEMENTATION': rmw_implementation}
                )
            ]
        ),
    ])


def get_fibonacci_send_goal_output(*, order=1, with_feedback=0):
    assert order > 0
    output = [
        'Waiting for an action server to become available...',
        'Sending goal:',
        '     order: {}'.format(order),
        '',
        re.compile('Goal accepted with ID: [a-f0-9]+'),
        '',
    ]
    sequence = [0, 1]
    for _ in range(order - 1):
        sequence.append(sequence[-1] + sequence[-2])
        if with_feedback > 0:
            output.append('Feedback:')
            output.extend(('    ' + yaml.dump({
                'sequence': sequence
            })).splitlines())
            output.append('')
            with_feedback -= 1
    output.append('Result:'),
    output.extend(('    ' + yaml.dump({
        'sequence': sequence
    })).splitlines())
    output.append('')
    output.append('Goal finished with status: SUCCEEDED')
    return output


class TestROS2ActionCLI(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output,
        rmw_implementation
    ):
        @contextlib.contextmanager
        def launch_action_command(self, arguments):
            action_command_action = ExecuteProcess(
                cmd=['ros2', 'action', *arguments],
                name='ros2action-cli', output='screen',
                additional_env={
                    'RMW_IMPLEMENTATION': rmw_implementation,
                    'PYTHONUNBUFFERED': '1'
                }
            )
            with launch_testing.tools.launch_process(
                launch_service, action_command_action, proc_info, proc_output,
                output_filter=launch_testing_ros.tools.basic_output_filter(
                    filtered_rmw_implementation=rmw_implementation
                )
            ) as action_command:
                yield action_command
        cls.launch_action_command = launch_action_command

    def test_info_on_nonexistent_action(self):
        with self.launch_action_command(arguments=['info', '/not_an_action']) as action_command:
            timeout = 20 if os.name == 'nt' else 10  # ros2 action info is slower on Windows
            assert action_command.wait_for_shutdown(timeout=timeout)
        assert action_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                'Action: /not_an_action',
                'Action clients: 0',
                'Action servers: 0',
            ],
            text=action_command.output,
            strict=False
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_fibonacci_info(self):
        with self.launch_action_command(arguments=['info', '/fibonacci']) as action_command:
            timeout = 20 if os.name == 'nt' else 10  # ros2 action info is slower on Windows
            assert action_command.wait_for_shutdown(timeout=timeout)
        assert action_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                'Action: /fibonacci',
                'Action clients: 0',
                'Action servers: 1',
                '  /fibonacci_action_server'
            ],
            text=action_command.output,
            strict=False
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_fibonacci_info_with_types(self):
        with self.launch_action_command(arguments=['info', '-t', '/fibonacci']) as action_command:
            timeout = 20 if os.name == 'nt' else 10  # ros2 action info is slower on Windows
            assert action_command.wait_for_shutdown(timeout=timeout)
        assert action_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                'Action: /fibonacci',
                'Action clients: 0',
                'Action servers: 1',
                '  /fibonacci_action_server [test_msgs/action/Fibonacci]'
            ],
            text=action_command.output,
            strict=False
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_fibonacci_info_count(self):
        with self.launch_action_command(arguments=['info', '-c', '/fibonacci']) as action_command:
            timeout = 20 if os.name == 'nt' else 10  # ros2 action info is slower on Windows
            assert action_command.wait_for_shutdown(timeout=timeout)
        assert action_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                'Action: /fibonacci',
                'Action clients: 0',
                'Action servers: 1',
            ],
            text=action_command.output,
            strict=False
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_list(self):
        with self.launch_action_command(arguments=['list']) as action_command:
            assert action_command.wait_for_shutdown(timeout=10)
        assert action_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['/fibonacci'],
            text=action_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_list_with_types(self):
        with self.launch_action_command(arguments=['list', '-t']) as action_command:
            assert action_command.wait_for_shutdown(timeout=10)
        assert action_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['/fibonacci [test_msgs/action/Fibonacci]'],
            text=action_command.output, strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_list_count(self):
        with self.launch_action_command(arguments=['list', '-c']) as action_command:
            assert action_command.wait_for_shutdown(timeout=10)
        assert action_command.exit_code == launch_testing.asserts.EXIT_OK
        command_output_lines = action_command.output.splitlines()
        assert len(command_output_lines) == 1
        assert int(command_output_lines[0]) == 1

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_send_fibonacci_goal(self):
        with self.launch_action_command(
            arguments=[
                'send_goal',
                '/fibonacci',
                'test_msgs/action/Fibonacci',
                '{order: 5}'
            ],
        ) as action_command:
            assert action_command.wait_for_shutdown(timeout=10)
        assert action_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=get_fibonacci_send_goal_output(order=5),
            text=action_command.output, strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_send_fibonacci_goal_with_feedback(self):
        with self.launch_action_command(
            arguments=[
                'send_goal',
                '-f',
                '/fibonacci',
                'test_msgs/action/Fibonacci',
                '{order: 5}'
            ],
        ) as action_command:
            assert action_command.wait_for_shutdown(timeout=10)
        assert action_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=get_fibonacci_send_goal_output(order=5, with_feedback=1),
            text=action_command.output, strict=False
        )
