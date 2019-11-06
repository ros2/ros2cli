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
from launch.actions import OpaqueFunction

from launch_ros.actions import Node

import launch_testing
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
import launch_testing_ros.tools

import pytest

from rmw_implementation import get_available_rmw_implementations

DEMO_NODES_TYPES = """\
ros2component_fixtures
  ros2component_fixtures::Talker
  ros2component_fixtures::Listener
"""


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation, ready_fn):
    additional_env = {'RMW_IMPLEMENTATION': rmw_implementation}
    component_node = Node(
        package='rclcpp_components', node_executable='component_container', output='screen')
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
                        component_node,
                        OpaqueFunction(function=lambda context: ready_fn())
                    ],
                    additional_env=additional_env
                )
            ]
        ),
    ]), locals()


class TestROS2ComponentTypesCLI(unittest.TestCase):

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
                cmd=['ros2', 'component', *arguments],
                additional_env={
                    'RMW_IMPLEMENTATION': rmw_implementation,
                    'PYTHONUNBUFFERED': '1'
                },
                name='ros2component-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, node_command_action, proc_info, proc_output,
                output_filter=launch_testing_ros.tools.basic_output_filter(
                    # ignore ros2cli daemon nodes
                    filtered_patterns=['.*ros2cli.*'],
                    filtered_rmw_implementation=rmw_implementation
                )
            ) as node_command:
                yield node_command
        cls.launch_node_command = launch_node_command

    # Set verb tests
    @launch_testing.markers.retry_on_failure(times=3)
    def test_types_verb(self):
        with self.launch_node_command(
                arguments=['types']) as node_command:
            assert node_command.wait_for_shutdown(timeout=20)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_text=DEMO_NODES_TYPES,
            text=node_command.output,
            strict=True
        )
