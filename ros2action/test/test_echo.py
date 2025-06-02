# Copyright 2025 Sony Group Corporation.
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

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools

import launch_testing_ros.tools

from osrf_pycommon.terminal_color import remove_ansi_escape_sequences

import pytest

from rclpy.utilities import get_available_rmw_implementations
from ros2cli.helpers import get_rmw_additional_env

EXPECTED_OUTPUT = [
    [
        'interface: GOAL_SERVICE',
        'info:',
        '  event_type: REQUEST_SENT',
        '  stamp:',
        re.compile(r'    sec: \d+'),
        re.compile(r'    nanosec: \d+'),
        "  client_gid: '<array type: uint8[16]>'",
        re.compile(r'  sequence_number: \d+'),
        "request: '<sequence type: test_msgs/action/Fibonacci_SendGoal_Request[1], "
        "length: 1>'",
        "response: '<sequence type: test_msgs/action/Fibonacci_SendGoal_Response[1], "
        "length: 0>'",
        '---',
    ],
    [
        'interface: GOAL_SERVICE',
        'info:',
        '  event_type: REQUEST_RECEIVED',
        '  stamp:',
        re.compile(r'    sec: \d+'),
        re.compile(r'    nanosec: \d+'),
        "  client_gid: '<array type: uint8[16]>'",
        re.compile(r'  sequence_number: \d+'),
        "request: '<sequence type: test_msgs/action/Fibonacci_SendGoal_Request[1], "
        "length: 1>'",
        "response: '<sequence type: test_msgs/action/Fibonacci_SendGoal_Response[1], "
        "length: 0>'",
        '---',
    ],
    [
        'interface: GOAL_SERVICE',
        'info:',
        '  event_type: RESPONSE_SENT',
        '  stamp:',
        re.compile(r'    sec: \d+'),
        re.compile(r'    nanosec: \d+'),
        "  client_gid: '<array type: uint8[16]>'",
        re.compile(r'  sequence_number: \d+'),
        "request: '<sequence type: test_msgs/action/Fibonacci_SendGoal_Request[1], "
        "length: 0>'",
        "response: '<sequence type: test_msgs/action/Fibonacci_SendGoal_Response[1], "
        "length: 1>'",
        '---',
    ],
    [
        'interface: GOAL_SERVICE',
        'info:',
        '  event_type: RESPONSE_RECEIVED',
        '  stamp:',
        re.compile(r'    sec: \d+'),
        re.compile(r'    nanosec: \d+'),
        "  client_gid: '<array type: uint8[16]>'",
        re.compile(r'  sequence_number: \d+'),
        "request: '<sequence type: test_msgs/action/Fibonacci_SendGoal_Request[1], "
        "length: 0>'",
        "response: '<sequence type: test_msgs/action/Fibonacci_SendGoal_Response[1], "
        "length: 1>'",
        '---',
    ],
    [
        'interface: STATUS_TOPIC',
        "status_list: '<sequence type: action_msgs/msg/GoalStatus, length: 1>'",
        '---',
    ],
    [
        'interface: FEEDBACK_TOPIC',
        'goal_id:',
        "  uuid: '<array type: uint8[16]>'",
        'feedback:',
        "  sequence: '<sequence type: int32, length: 3>'",
        '---',
    ],
    [
        'interface: RESULT_SERVICE',
        'info:',
        '  event_type: REQUEST_RECEIVED',
        '  stamp:',
        re.compile(r'    sec: \d+'),
        re.compile(r'    nanosec: \d+'),
        "  client_gid: '<array type: uint8[16]>'",
        re.compile(r'  sequence_number: \d+'),
        "request: '<sequence type: test_msgs/action/Fibonacci_GetResult_Request[1], "
        "length: 1>'",
        "response: '<sequence type: test_msgs/action/Fibonacci_GetResult_Response[1], "
        "length: 0>'",
        '---',
    ],
    [
        'interface: RESULT_SERVICE',
        'info:',
        '  event_type: RESPONSE_SENT',
        '  stamp:',
        re.compile(r'    sec: \d+'),
        re.compile(r'    nanosec: \d+'),
        "  client_gid: '<array type: uint8[16]>'",
        re.compile(r'  sequence_number: \d+'),
        "request: '<sequence type: test_msgs/action/Fibonacci_GetResult_Request[1], "
        "length: 0>'",
        "response: '<sequence type: test_msgs/action/Fibonacci_GetResult_Response[1], "
        "length: 1>'",
        '---'
    ]
]


# Skip cli tests on Windows while they exhibit pathological behavior
# https://github.com/ros2/build_farmer/issues/248
if sys.platform.startswith('win'):
    pytest.skip(
            'CLI tests can block for a pathological amount of time on Windows.',
            allow_module_level=True)


# Check if all parts of the expected_output are present in the output.
def match_expected_output(output, expected_output) -> bool:
    output_lines_tmp = output.splitlines()
    output_lines = [remove_ansi_escape_sequences(line) for line in output_lines_tmp]

    for msg_block in expected_output:

        # Find the position where the first message of the message block appears.
        matched_begin_lines = []
        if isinstance(msg_block[0], str):
            for index, line in enumerate(output_lines):
                if msg_block[0] == line:
                    matched_begin_lines.append(index)

        if hasattr(msg_block[0], 'match'):
            for index, line in enumerate(output_lines):
                if msg_block[0].match(line):
                    matched_begin_lines.append(index)

        filtered_matched_begin_lines = matched_begin_lines
        if matched_begin_lines:
            for line in matched_begin_lines:
                if len(msg_block) > len(output_lines) - line:
                    filtered_matched_begin_lines.remove(line)

            if not filtered_matched_begin_lines:
                return False
        else:
            return False

        # filter_matched_begin_lines include all positions where the first message appears.

        msg_block_matched = False
        for line in filtered_matched_begin_lines:
            matched_index = -1
            for index, (expected_line, output_line) in enumerate(
                zip(msg_block, output_lines[line:]), start=1
            ):
                if isinstance(expected_line, str):
                    if expected_line != output_line:
                        break
                    else:
                        matched_index = index
                        continue
                if hasattr(expected_line, 'match'):
                    if not expected_line.match(output_line):
                        break
                    else:
                        matched_index = index
                        continue
            # A complete match has been found, skipping the rest of the detection.
            if (matched_index == len(msg_block)):
                msg_block_matched = True
                break

        # If no message block is matched throughout the entire output.
        if not msg_block_matched:
            return False

    return True


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation):
    path_to_action_server_executable = os.path.join(
        os.path.dirname(__file__), 'fixtures', 'fibonacci_action_introspection.py'
    )
    additional_env = get_rmw_additional_env(rmw_implementation)
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
                            additional_env=additional_env
                        ),
                        launch_testing.actions.ReadyToTest()
                    ],
                    additional_env=additional_env
                )
            ]
        ),
    ])


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
            additional_env = get_rmw_additional_env(rmw_implementation)
            additional_env['PYTHONUNBUFFERED'] = '1'
            action_command_action = ExecuteProcess(
                cmd=['ros2', 'action', *arguments],
                name='ros2action-cli', output='screen',
                additional_env=additional_env
            )
            with launch_testing.tools.launch_process(
                launch_service, action_command_action, proc_info, proc_output,
                output_filter=launch_testing_ros.tools.basic_output_filter(
                    filtered_rmw_implementation=rmw_implementation
                )
            ) as action_command:
                yield action_command
        cls.launch_action_command = launch_action_command

    # @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_echo(self):
        echo_arguments = [
            'echo', '/test/fibonacci', 'test_msgs/action/Fibonacci', '--no-arr']

        with self.launch_action_command(arguments=echo_arguments) as action_command:
            assert action_command.wait_for_output(
                functools.partial(
                    match_expected_output,
                    expected_output=EXPECTED_OUTPUT,
                ),
                timeout=10,
            )
        assert action_command.wait_for_shutdown(timeout=10)

    # @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_echo_with_qos(self):
        echo_arguments = [
            'echo', '/test/fibonacci', 'test_msgs/action/Fibonacci', '--no-arr',
            '--qos-profile', 'services_default']

        with self.launch_action_command(arguments=echo_arguments) as action_command:
            assert action_command.wait_for_output(
                functools.partial(
                    match_expected_output,
                    expected_output=EXPECTED_OUTPUT,
                ),
                timeout=10,
            )
        assert action_command.wait_for_shutdown(timeout=10)
