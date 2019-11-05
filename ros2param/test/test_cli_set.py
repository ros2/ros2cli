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

import param_node

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
                            node_name='param_node',
                            additional_env=additional_env
                        ),
                        Node(
                            node_executable=sys.executable,
                            arguments=[path_to_param_node_script],
                            node_name='_hidden_param_node',
                            additional_env=additional_env
                        ),
                        OpaqueFunction(function=lambda context: ready_fn())
                    ],
                    additional_env=additional_env
                )
            ]
        ),
    ])


class TestROS2ParamSetCLI(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output,
        rmw_implementation
    ):
        @contextlib.contextmanager
        def launch_node_command(self, arguments):
            node_command_action = ExecuteProcess(
                cmd=['ros2', 'param', *arguments],
                additional_env={
                    'RMW_IMPLEMENTATION': rmw_implementation,
                    'PYTHONUNBUFFERED': '1'
                },
                name='ros2param-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, node_command_action, proc_info, proc_output,
                output_filter=launch_testing_ros.tools.basic_output_filter(
                    # ignore ros2cli daemon node
                    filtered_patterns=['.*ros2cli.*'],
                    filtered_rmw_implementation=rmw_implementation
                )
            ) as node_command:
                yield node_command
        cls.launch_node_command = launch_node_command

    @launch_testing.markers.retry_on_failure(times=3)
    def test_set_nonexistent_param_existing_node(self):
        with self.launch_node_command(
                arguments=['set', '/param_node', 'nonexistent_parameter', '1']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                "Setting parameter failed: ('Invalid access to undeclared parameter(s)', [])"],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_set_valid_param_existing_node(self):
        new_parameters = param_node.ParamNode.get_modified_parameters()
        # Byte array is unsupported for the yaml parser in get_parameter_value
        new_parameters.pop('byte_array')
        for param_key, param_value in new_parameters.items():
            with self.launch_node_command(
                    arguments=['set', '/param_node', param_key, str(param_value)]) as node_command:
                assert node_command.wait_for_shutdown(timeout=5)
            assert node_command.exit_code == launch_testing.asserts.EXIT_OK
            assert launch_testing.tools.expect_output(
                expected_lines=['Set parameter successful'],
                text=node_command.output,
                strict=True
            )
        for param_key, param_value in new_parameters.items():
            with self.launch_node_command(
                    arguments=['get', '/param_node', param_key]) as node_command:
                assert node_command.wait_for_shutdown(timeout=5)
            assert node_command.exit_code == launch_testing.asserts.EXIT_OK
            if param_value is None:
                param_node.GET_VERB_PARAM_VALUES[param_key] = \
                    param_node.GET_VERB_PARAM_VALUES['str_param']
            assert launch_testing.tools.expect_output(
                expected_text=param_node.GET_VERB_PARAM_VALUES[param_key].format(str(param_value)),
                text=node_command.output,
                strict=True
            )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_set_nonexistent_param_hidden_node_no_hidden_argument(self):
        with self.launch_node_command(
                arguments=[
                    'set', '/_hidden_param_node', 'nonexistent_parameter', '1']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Node not found'],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_set_nonexistent_param_hidden_node_hidden_argument(self):
        with self.launch_node_command(
                arguments=[
                    'set', '/_hidden_param_node',
                    'nonexistent_parameter', '1', '--include-hidden-nodes']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                "Setting parameter failed: ('Invalid access to undeclared parameter(s)', [])"],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_set_nonexistent_node(self):
        with self.launch_node_command(
                arguments=['set', '/foo/nonexistent_node', 'test_param', '3.14']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Node not found'],
            text=node_command.output,
            strict=True
        )
