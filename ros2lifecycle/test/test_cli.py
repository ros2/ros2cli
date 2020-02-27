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


ALL_LIFECYCLE_NODE_TRANSITIONS = [
    '- configure [1]',
    '\tStart: unconfigured',
    '\tGoal: configuring',
    '- transition_success [10]',
    '\tStart: configuring',
    '\tGoal: inactive',
    '- transition_failure [11]',
    '\tStart: configuring',
    '\tGoal: unconfigured',
    '- transition_error [12]',
    '\tStart: configuring',
    '\tGoal: errorprocessing',
    '- cleanup [2]',
    '\tStart: inactive',
    '\tGoal: cleaningup',
    '- transition_success [20]',
    '\tStart: cleaningup',
    '\tGoal: unconfigured',
    '- transition_failure [21]',
    '\tStart: cleaningup',
    '\tGoal: inactive',
    '- transition_error [22]',
    '\tStart: cleaningup',
    '\tGoal: errorprocessing',
    '- activate [3]',
    '\tStart: inactive',
    '\tGoal: activating',
    '- transition_success [30]',
    '\tStart: activating',
    '\tGoal: active',
    '- transition_failure [31]',
    '\tStart: activating',
    '\tGoal: inactive',
    '- transition_error [32]',
    '\tStart: activating',
    '\tGoal: errorprocessing',
    '- deactivate [4]',
    '\tStart: active',
    '\tGoal: deactivating',
    '- transition_success [40]',
    '\tStart: deactivating',
    '\tGoal: inactive',
    '- transition_failure [41]',
    '\tStart: deactivating',
    '\tGoal: active',
    '- transition_error [42]',
    '\tStart: deactivating',
    '\tGoal: errorprocessing',
    '- shutdown [5]',
    '\tStart: unconfigured',
    '\tGoal: shuttingdown',
    '- shutdown [6]',
    '\tStart: inactive',
    '\tGoal: shuttingdown',
    '- shutdown [7]',
    '\tStart: active',
    '\tGoal: shuttingdown',
    '- transition_success [50]',
    '\tStart: shuttingdown',
    '\tGoal: finalized',
    '- transition_failure [51]',
    '\tStart: shuttingdown',
    '\tGoal: finalized',
    '- transition_error [52]',
    '\tStart: shuttingdown',
    '\tGoal: errorprocessing',
    '- transition_success [60]',
    '\tStart: errorprocessing',
    '\tGoal: unconfigured',
    '- transition_failure [61]',
    '\tStart: errorprocessing',
    '\tGoal: finalized',
    '- transition_error [62]',
    '\tStart: errorprocessing',
    '\tGoal: finalized'
]


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation):
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
                            package='ros2lifecycle_test_fixtures',
                            node_executable='simple_lifecycle_node',
                            node_name='test_lifecycle_node',
                            output='screen',
                            additional_env=additional_env
                        ),
                        Node(
                            package='ros2lifecycle_test_fixtures',
                            node_executable='simple_lifecycle_node',
                            node_name='_hidden_test_lifecycle_node',
                            output='screen',
                            additional_env=additional_env
                        ),
                        launch_testing.actions.ReadyToTest()
                    ],
                    additional_env=additional_env
                )
            ]
        ),
    ])


class TestROS2LifecycleCLI(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output,
        rmw_implementation
    ):
        @contextlib.contextmanager
        def launch_lifecycle_command(self, arguments):
            lifecycle_command_action = ExecuteProcess(
                cmd=['ros2', 'lifecycle', *arguments],
                additional_env={
                    'RMW_IMPLEMENTATION': rmw_implementation,
                    'PYTHONUNBUFFERED': '1'
                },
                name='ros2lifecycle-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, lifecycle_command_action, proc_info, proc_output,
                output_filter=launch_testing_ros.tools.basic_output_filter(
                    filtered_rmw_implementation=rmw_implementation
                )
            ) as lifecycle_command:
                yield lifecycle_command
        cls.launch_lifecycle_command = launch_lifecycle_command

    @launch_testing.markers.retry_on_failure(times=5)
    def test_list_lifecycle_node_transitions(self):
        with self.launch_lifecycle_command(
            arguments=['list', 'test_lifecycle_node']
        ) as lifecycle_command:
            assert lifecycle_command.wait_for_shutdown(timeout=20)
        assert lifecycle_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                '- configure [1]',
                '\tStart: unconfigured',
                '\tGoal: configuring',
                '- shutdown [5]',
                '\tStart: unconfigured',
                '\tGoal: shuttingdown'
            ],
            text=lifecycle_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_list_all_lifecycle_node_transitions(self):
        with self.launch_lifecycle_command(
            arguments=['list', 'test_lifecycle_node', '-a']
        ) as lifecycle_command:
            assert lifecycle_command.wait_for_shutdown(timeout=20)
        assert lifecycle_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=ALL_LIFECYCLE_NODE_TRANSITIONS,
            text=lifecycle_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_list_all_lifecycle_hidden_node_transitions_without_hidden_flag(self):
        with self.launch_lifecycle_command(
            arguments=['list', '_hidden_test_lifecycle_node', '-a']
        ) as lifecycle_command:
            assert lifecycle_command.wait_for_shutdown(timeout=20)
        assert lifecycle_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Node not found'],
            text=lifecycle_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_list_all_lifecycle_hidden_node_transitions_with_hidden_flag(self):
        with self.launch_lifecycle_command(
            arguments=['list', '--include-hidden-nodes', '_hidden_test_lifecycle_node', '-a']
        ) as lifecycle_command:
            assert lifecycle_command.wait_for_shutdown(timeout=20)
        assert lifecycle_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=ALL_LIFECYCLE_NODE_TRANSITIONS,
            text=lifecycle_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_list_lifecycle_nodes(self):
        with self.launch_lifecycle_command(arguments=['nodes']) as lifecycle_command:
            assert lifecycle_command.wait_for_shutdown(timeout=20)
        assert lifecycle_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['/test_lifecycle_node'],
            text=lifecycle_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_list_all_lifecycle_nodes(self):
        with self.launch_lifecycle_command(arguments=['nodes', '-a']) as lifecycle_command:
            assert lifecycle_command.wait_for_shutdown(timeout=20)
        assert lifecycle_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                '/_hidden_test_lifecycle_node',
                '/test_lifecycle_node'
            ],
            text=lifecycle_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_count_lifecycle_nodes(self):
        with self.launch_lifecycle_command(arguments=['nodes', '-c']) as lifecycle_command:
            assert lifecycle_command.wait_for_shutdown(timeout=20)
        assert lifecycle_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = lifecycle_command.output.splitlines()
        assert len(output_lines) == 1
        assert int(output_lines[0]) == 1

    @launch_testing.markers.retry_on_failure(times=5)
    def test_count_all_lifecycle_nodes(self):
        with self.launch_lifecycle_command(arguments=['nodes', '-a', '-c']) as lifecycle_command:
            assert lifecycle_command.wait_for_shutdown(timeout=20)
        assert lifecycle_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = lifecycle_command.output.splitlines()
        assert len(output_lines) == 1
        assert int(output_lines[0]) == 2

    @launch_testing.markers.retry_on_failure(times=5)
    def test_set_lifecycle_node_invalid_transition(self):
        with self.launch_lifecycle_command(
            arguments=['set', '/test_lifecycle_node', 'noop']
        ) as lifecycle_command:
            assert lifecycle_command.wait_for_shutdown(timeout=20)
        assert lifecycle_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=[
                'Unknown transition requested, available ones are:',
                '- configure [1]',
                '- shutdown [5]'
            ],
            text=lifecycle_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_set_nonexistent_lifecycle_node_state(self):
        with self.launch_lifecycle_command(
            arguments=['set', '/nonexistent_test_lifecycle_node', 'configure']
        ) as lifecycle_command:
            assert lifecycle_command.wait_for_shutdown(timeout=20)
        assert lifecycle_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Node not found'],
            text=lifecycle_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_set_hidden_lifecycle_node_transition(self):
        with self.launch_lifecycle_command(
            arguments=['set', '/_hidden_test_lifecycle_node', 'configure']
        ) as lifecycle_command:
            assert lifecycle_command.wait_for_shutdown(timeout=20)
        assert lifecycle_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Node not found'],
            text=lifecycle_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_get_nonexistent_lifecycle_node_state(self):
        with self.launch_lifecycle_command(
            arguments=['get', '/nonexistent_test_lifecycle_node']
        ) as lifecycle_command:
            assert lifecycle_command.wait_for_shutdown(timeout=20)
        assert lifecycle_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Node not found'],
            text=lifecycle_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_get_hidden_lifecycle_node_state(self):
        with self.launch_lifecycle_command(
            arguments=['get', '/_hidden_test_lifecycle_node']
        ) as lifecycle_command:
            assert lifecycle_command.wait_for_shutdown(timeout=20)
        assert lifecycle_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Node not found'],
            text=lifecycle_command.output,
            strict=True
        )

        with self.launch_lifecycle_command(
            arguments=['get', '--include-hidden-nodes', '/_hidden_test_lifecycle_node']
        ) as lifecycle_command:
            assert lifecycle_command.wait_for_shutdown(timeout=20)
        assert lifecycle_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['unconfigured [1]'],
            text=lifecycle_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_get_lifecycle_node_state(self):
        with self.launch_lifecycle_command(
            arguments=['get', '/test_lifecycle_node']
        ) as lifecycle_command:
            assert lifecycle_command.wait_for_shutdown(timeout=20)
        assert lifecycle_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['unconfigured [1]'],
            text=lifecycle_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_node_lifecycle(self):
        lifecycle = [
            ('unconfigured [1]', 'configure'),
            ('inactive [2]', 'activate'),
            ('active [3]', 'deactivate'),
            ('inactive [2]', 'cleanup')
        ]
        for current_state, next_action in lifecycle:
            with self.launch_lifecycle_command(
                arguments=['get', '--include-hidden-nodes', '/_hidden_test_lifecycle_node']
            ) as lifecycle_command:
                assert lifecycle_command.wait_for_shutdown(timeout=20)
            assert lifecycle_command.exit_code == launch_testing.asserts.EXIT_OK
            assert launch_testing.tools.expect_output(
                expected_lines=[current_state],
                text=lifecycle_command.output,
                strict=True
            )

            with self.launch_lifecycle_command(
                arguments=[
                    'set', '--include-hidden-nodes', '/_hidden_test_lifecycle_node', next_action
                ]
            ) as lifecycle_command:
                assert lifecycle_command.wait_for_shutdown(timeout=20)
            assert lifecycle_command.exit_code == launch_testing.asserts.EXIT_OK
            assert launch_testing.tools.expect_output(
                expected_lines=['Transitioning successful'],
                text=lifecycle_command.output,
                strict=True
            )

        with self.launch_lifecycle_command(
            arguments=['get', '--include-hidden-nodes', '/_hidden_test_lifecycle_node']
        ) as lifecycle_command:
            assert lifecycle_command.wait_for_shutdown(timeout=20)
        assert lifecycle_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[lifecycle[0][0]], text=lifecycle_command.output, strict=True
        )
