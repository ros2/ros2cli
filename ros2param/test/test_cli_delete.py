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
import sys
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction

from launch_ros.actions import Node

import launch_testing
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
import launch_testing_ros.tools

import pytest

from rmw_implementation import get_available_rmw_implementations


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation, ready_fn):
    path_to_param_node_script = os.path.join(
        os.path.dirname(__file__), 'param_node.py'
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
                            node_executable=sys.executable,
                            arguments=[path_to_param_node_script],
                            node_name='param_delete_node',
                            additional_env=additional_env
                        ),
                        Node(
                            node_executable=sys.executable,
                            arguments=[path_to_param_node_script],
                            node_name='_hidden_param_delete_node',
                            additional_env=additional_env
                        ),
                        OpaqueFunction(function=lambda context: ready_fn())
                    ],
                    additional_env=additional_env
                )
            ]
        ),
    ])


class TestROS2ParamDeleteCLI(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output,
        rmw_implementation
    ):
        @contextlib.contextmanager
        def launch_param_command(self, arguments):
            param_command_action = ExecuteProcess(
                cmd=['ros2', 'param', *arguments],
                additional_env={
                    'RMW_IMPLEMENTATION': rmw_implementation,
                    'PYTHONUNBUFFERED': '1'
                },
                name='ros2param-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, param_command_action, proc_info, proc_output,
                output_filter=launch_testing_ros.tools.basic_output_filter(
                    # ignore ros2cli daemon node
                    filtered_patterns=['.*ros2cli.*'],
                    filtered_rmw_implementation=rmw_implementation
                )
            ) as param_command:
                yield param_command
        cls.launch_param_command = launch_param_command

    @launch_testing.markers.retry_on_failure(times=3)
    def test_delete_int_param_existing_node(self):
        with self.launch_param_command(
                arguments=['delete', '/param_delete_node', 'int_param']) as param_command:
            assert param_command.wait_for_shutdown(timeout=5)
        assert param_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Deleted parameter successfully'],
            text=param_command.output,
            strict=True
        )
        with self.launch_param_command(
                arguments=['get', '/param_delete_node', 'int_param']) as param_command:
            assert param_command.wait_for_shutdown(timeout=5)
        assert param_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Parameter not set.'],
            text=param_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_delete_int_param_hidden_node(self):
        with self.launch_param_command(
                arguments=['delete', '/_hidden_param_delete_node', 'int_param']) as param_command:
            assert param_command.wait_for_shutdown(timeout=5)
        assert param_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Node not found'],
            text=param_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_delete_int_param_hidden_node_with_hidden_flag(self):
        with self.launch_param_command(
                arguments=[
                    'delete', '--include-hidden-nodes',
                    '/_hidden_param_delete_node', 'int_param']) as param_command:
            assert param_command.wait_for_shutdown(timeout=8)
        assert param_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Deleted parameter successfully'],
            text=param_command.output,
            strict=True
        )
        with self.launch_param_command(
                arguments=[
                    'get', '--include-hidden-nodes',
                    '/_hidden_param_delete_node', 'int_param']) as param_command:
            assert param_command.wait_for_shutdown(timeout=5)
        assert param_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Parameter not set.'],
            text=param_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_delete_any_param_nonexistent_node(self):
        with self.launch_param_command(
                arguments=['delete', '/foo/nonexistent_node', 'int_param']) as param_command:
            assert param_command.wait_for_shutdown(timeout=5)
        assert param_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Node not found'],
            text=param_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_delete_nonexistent_param_existent_node(self):
        with self.launch_param_command(
                arguments=['delete', '/param_delete_node', 'foo_param']) as param_command:
            assert param_command.wait_for_shutdown(timeout=5)
        assert param_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Deleting parameter failed'],
            text=param_command.output,
            strict=True
        )
