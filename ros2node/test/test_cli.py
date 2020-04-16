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
import itertools
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


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation):
    path_to_complex_node_script = os.path.join(
        os.path.dirname(__file__), 'fixtures', 'complex_node.py'
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
                            arguments=[path_to_complex_node_script],
                            node_name='complex_node',
                            additional_env=additional_env
                        ),
                        Node(
                            executable=sys.executable,
                            arguments=[path_to_complex_node_script],
                            node_name='_hidden_complex_node',
                            additional_env=additional_env
                        ),
                        launch_testing.actions.ReadyToTest()
                    ],
                    additional_env=additional_env
                )
            ]
        ),
    ])


class TestROS2NodeCLI(unittest.TestCase):

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
                cmd=['ros2', 'node', *arguments],
                additional_env={
                    'RMW_IMPLEMENTATION': rmw_implementation,
                    'PYTHONUNBUFFERED': '1'
                },
                name='ros2node-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, node_command_action, proc_info, proc_output,
                output_filter=launch_testing_ros.tools.basic_output_filter(
                    # ignore launch_ros and ros2cli daemon nodes
                    filtered_patterns=['.*launch_ros.*', '.*ros2cli.*'],
                    filtered_rmw_implementation=rmw_implementation
                )
            ) as node_command:
                yield node_command
        cls.launch_node_command = launch_node_command

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_list_nodes(self):
        with self.launch_node_command(arguments=['list']) as node_command:
            assert node_command.wait_for_shutdown(timeout=10)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['/complex_node'],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_list_all_nodes(self):
        with self.launch_node_command(arguments=['list', '-a']) as node_command:
            assert node_command.wait_for_shutdown(timeout=10)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                '/_hidden_complex_node',
                '/complex_node'
            ],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_list_node_count(self):
        with self.launch_node_command(arguments=['list', '-c']) as node_command:
            assert node_command.wait_for_shutdown(timeout=10)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = node_command.output.splitlines()
        assert len(output_lines) == 1
        # Fixture nodes that are not hidden.
        assert int(output_lines[0]) == 1

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_list_all_nodes_count(self):
        with self.launch_node_command(arguments=['list', '-c', '-a']) as node_command:
            assert node_command.wait_for_shutdown(timeout=10)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = node_command.output.splitlines()
        assert len(output_lines) == 1
        # All fixture nodes plus ros2cli daemon node.
        assert int(output_lines[0]) == 3

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_info_node(self):
        with self.launch_node_command(arguments=['info', '/complex_node']) as node_command:
            assert node_command.wait_for_shutdown(timeout=10)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        # TODO(hidmic): only optionally show hidden topics and services
        assert launch_testing.tools.expect_output(
            expected_lines=itertools.chain([
                '/complex_node',
                '  Subscribers:',
                '    /strings: test_msgs/msg/Strings',
                '  Publishers:',
                '    /arrays: test_msgs/msg/Arrays',
                '    /parameter_events: rcl_interfaces/msg/ParameterEvent',
                '    /rosout: rcl_interfaces/msg/Log',
                '  Service Servers:',
                '    /basic: test_msgs/srv/BasicTypes',
            ], itertools.repeat(re.compile(
                r'\s*/complex_node/.*parameter.*: rcl_interfaces/srv/.*Parameter.*'
            ), 6), [
                '  Service Clients:',
                '',
                '  Action Servers:',
                '    /fibonacci: test_msgs/action/Fibonacci',
                '  Action Clients:',
                ''
            ]),
            text=node_command.output, strict=False
        ), 'Output does not match:\n' + node_command.output

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_info_hidden_node_no_hidden_flag(self):
        with self.launch_node_command(arguments=['info', '/_hidden_complex_node']) as node_command:
            assert node_command.wait_for_shutdown(timeout=10)
        assert node_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=["Unable to find node '/_hidden_complex_node'"],
            text=node_command.output, strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_info_hidden_node_hidden_flag(self):
        with self.launch_node_command(
            arguments=['info', '/_hidden_complex_node', '--include-hidden']
        ) as node_command:
            assert node_command.wait_for_shutdown(timeout=10)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=itertools.chain([
                '/_hidden_complex_node',
                '  Subscribers:',
                '    /strings: test_msgs/msg/Strings',
                '  Publishers:',
                '    /arrays: test_msgs/msg/Arrays',
                '    /parameter_events: rcl_interfaces/msg/ParameterEvent',
                '    /rosout: rcl_interfaces/msg/Log',
                '  Service Servers:'
            ], itertools.repeat(re.compile(
                r'\s*/_hidden_complex_node/.*parameter.*: rcl_interfaces/srv/.*Parameter.*'
            ), 6), [
                '    /basic: test_msgs/srv/BasicTypes',
                '  Service Clients:',
                '  Action Servers:',
                '    /fibonacci: test_msgs/action/Fibonacci',
                '  Action Clients:',
                ''
            ]),
            text=node_command.output, strict=False
        ), 'Output does not match:\n' + node_command.output
