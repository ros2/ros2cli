# Copyright 2026 Open Source Robotics Foundation, Inc.
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
import tempfile
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.actions import ResetEnvironment
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnShutdown
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
from launch_testing_ros.actions import EnableRmwIsolation
import launch_testing_ros.tools
import pytest
from rclpy.utilities import get_available_rmw_implementations

from ros2cli.helpers import get_rmw_additional_env


if sys.platform.startswith('win'):
    pytest.skip(
        'CLI tests can block for a pathological amount of time on Windows.',
        allow_module_level=True,
    )


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation):
    action_server = os.path.join(
        os.path.dirname(__file__), 'fixtures', 'fibonacci_action_server.py'
    )
    additional_env = get_rmw_additional_env(rmw_implementation)
    set_env_actions = [SetEnvironmentVariable(k, v) for k, v in additional_env.items()]

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'daemon', 'stop'],
            name='daemon-stop',
            on_exit=[
                *set_env_actions,
                EnableRmwIsolation(),
                RegisterEventHandler(OnShutdown(on_shutdown=[
                    ExecuteProcess(
                        cmd=['ros2', 'daemon', 'stop'],
                        name='daemon-stop-isolated',
                        additional_env=dict(additional_env),
                    ),
                    ResetEnvironment(),
                ])),
                ExecuteProcess(
                    cmd=['ros2', 'daemon', 'start'],
                    name='daemon-start',
                    on_exit=[
                        ExecuteProcess(cmd=[sys.executable, action_server]),
                        launch_testing.actions.ReadyToTest(),
                    ],
                ),
            ],
        ),
    ])


class TestSendGoalFileCLI(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output,
        rmw_implementation,
    ):
        @contextlib.contextmanager
        def launch_action_command(self, arguments):
            action = ExecuteProcess(
                cmd=['ros2', 'action', *arguments],
                name='ros2action-goal-file-cli',
                output='screen',
                additional_env={'PYTHONUNBUFFERED': '1'},
            )
            with launch_testing.tools.launch_process(
                launch_service,
                action,
                proc_info,
                proc_output,
                output_filter=launch_testing_ros.tools.basic_output_filter(
                    filtered_rmw_implementation=rmw_implementation
                ),
            ) as process:
                yield process

        cls.launch_action_command = launch_action_command

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_goal_file_reaches_action_server(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            goal_file = os.path.join(tmpdir, 'goal.yaml')
            with open(goal_file, 'w', encoding='utf-8') as f:
                f.write('order: 5\n')

            with self.launch_action_command([
                'send_goal',
                '/test/fibonacci',
                'test_msgs/action/Fibonacci',
                '--goal-file',
                goal_file,
            ]) as process:
                assert process.wait_for_shutdown(timeout=10)

        assert process.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                'Sending goal:',
                '     order: 5',
                re.compile('Goal accepted with ID: [a-f0-9]+'),
                'Goal finished with status: SUCCEEDED',
            ],
            text=process.output,
            strict=False,
        )
