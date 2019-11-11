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
from launch.actions import TimerAction

from launch_ros.actions import Node

import launch_testing
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
import launch_testing_ros.tools

import pytest

from rmw_implementation import get_available_rmw_implementations

NON_EXISTENT_ID = [
    "Failed to unload component 5 from '/ComponentManager' container node",
    '    No node found with unique_id: 5'
]


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation, ready_fn):
    additional_env = {'RMW_IMPLEMENTATION': rmw_implementation}
    component_node = Node(
        package='rclcpp_components', node_executable='component_container', output='screen',
        additional_env=additional_env)
    listener_command_action = ExecuteProcess(
        cmd=['ros2', 'component', 'load', '/ComponentManager',
             'ros2component_test_fixtures', 'ros2component_test_fixtures::Listener'],
        additional_env={
            'RMW_IMPLEMENTATION': rmw_implementation,
            'PYTHONUNBUFFERED': '1'
        },
        name='ros2component-cli',
        output='screen',
        on_exit=[
            ExecuteProcess(
                cmd=['ros2', 'component', 'load', '/ComponentManager',
                     'ros2component_test_fixtures', 'ros2component_test_fixtures::Talker'],
                additional_env={
                    'RMW_IMPLEMENTATION': rmw_implementation,
                    'PYTHONUNBUFFERED': '1'
                },
                name='ros2component-cli',
                output='screen',
                on_exit=[
                    ExecuteProcess(
                        cmd=['ros2', 'component', 'load', '/ComponentManager',
                             'ros2component_test_fixtures', 'ros2component_test_fixtures::Talker'],
                        additional_env={
                            'RMW_IMPLEMENTATION': rmw_implementation,
                            'PYTHONUNBUFFERED': '1'
                        },
                        name='ros2component-cli',
                        output='screen'
                    )
                ]
            )
        ]
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
                        component_node,
                        TimerAction(period=3.0, actions=[listener_command_action]),
                        OpaqueFunction(function=lambda context: ready_fn())
                    ],
                    additional_env=additional_env
                )
            ]
        ),
    ]), locals()


class TestROS2ComponentUnloadCLI(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output,
        rmw_implementation
    ):
        @contextlib.contextmanager
        def launch_component_command(self, arguments):
            component_command_action = ExecuteProcess(
                cmd=['ros2', 'component', *arguments],
                additional_env={
                    'RMW_IMPLEMENTATION': rmw_implementation,
                    'PYTHONUNBUFFERED': '1'
                },
                name='ros2component-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, component_command_action, proc_info, proc_output,
                output_filter=launch_testing_ros.tools.basic_output_filter(
                    # ignore ros2cli daemon nodes
                    filtered_patterns=['.*ros2cli.*'],
                    filtered_rmw_implementation=rmw_implementation
                )
            ) as component_command:
                yield component_command
        cls.launch_component_command = launch_component_command

    @launch_testing.markers.retry_on_failure(times=10)
    def test_unload_verb(self):
        with self.launch_component_command(
                arguments=[
                    'unload', '/ComponentManager', '1']) as unload_command:
            assert unload_command.wait_for_shutdown(timeout=10)
        assert unload_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=["Unloaded component 1 from '/ComponentManager' container"],
            text=unload_command.output,
            strict=False
        )

    @launch_testing.markers.retry_on_failure(times=4)
    def test_unload_verb_nonexistent_id(self):
        with self.launch_component_command(
                arguments=[
                    'unload', '/ComponentManager', '5']) as unload_command:
            assert unload_command.wait_for_shutdown(timeout=20)
        assert unload_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=NON_EXISTENT_ID,
            text=unload_command.output,
            strict=False
        )

    @launch_testing.markers.retry_on_failure(times=4)
    def test_unload_verb_nonexistent_container(self):
        with self.launch_component_command(
                arguments=[
                    'unload', '/NonExistentContainer', '5']) as unload_command:
            assert unload_command.wait_for_shutdown(timeout=20)
        assert unload_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=["Unable to find container node '/NonExistentContainer'"],
            text=unload_command.output,
            strict=False
        )
