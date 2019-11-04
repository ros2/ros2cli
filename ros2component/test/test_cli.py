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
demo_nodes_cpp
  demo_nodes_cpp::OneOffTimerNode
  demo_nodes_cpp::ReuseTimerNode
  demo_nodes_cpp::ServerNode
  demo_nodes_cpp::ClientNode
  demo_nodes_cpp::ListParameters
  demo_nodes_cpp::ParameterBlackboard
  demo_nodes_cpp::SetAndGetParameters
  demo_nodes_cpp::ParameterEventsAsyncNode
  demo_nodes_cpp::EvenParameterNode
  demo_nodes_cpp::Talker
  demo_nodes_cpp::LoanedMessageTalker
  demo_nodes_cpp::SerializedMessageTalker
  demo_nodes_cpp::Listener
  demo_nodes_cpp::SerializedMessageListener
  demo_nodes_cpp::ListenerBestEffort
"""

LIST_THREE_NODES = """\
/ComponentManager
  2  /listener
  3  /talker
  4  /talker
"""

LIST_FOUR_NODES = """\
/ComponentManager
  2  /listener
  3  /talker
  4  /talker
  5  /talker
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


# TODO(BMarchi): Open splice is returning a different code (2) when
# the launch of a component verb ends. It should return EXIT_OK unless
# the command returns a error string, which should be code 1.
class TestROS2ComponentCLI(unittest.TestCase):

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
    def test_types_component(self):
        with self.launch_node_command(
                arguments=['types']) as node_command:
            assert node_command.wait_for_shutdown(timeout=20)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_text=DEMO_NODES_TYPES,
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=2)
    def test_load_unload_verb(self):
        with self.launch_node_command(
                arguments=[
                    'load', '/ComponentManager',
                    'demo_nodes_cpp', 'demo_nodes_cpp::Talker']) as talker_node:
            assert talker_node.wait_for_shutdown(timeout=20)
        assert talker_node.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                "Loaded component 1 into '/ComponentManager' "
                "container node as '/talker'"],
            text=talker_node.output,
            strict=True
        )
        with self.launch_node_command(
                arguments=[
                    'load', '/ComponentManager',
                    'demo_nodes_cpp', 'demo_nodes_cpp::Listener']) as listener_node:
            assert listener_node.wait_for_shutdown(timeout=20)
        assert listener_node.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                "Loaded component 2 into '/ComponentManager' "
                "container node as '/listener'"],
            text=listener_node.output,
            strict=True
        )
        with self.launch_node_command(
                arguments=[
                    'unload', '/ComponentManager', '1']) as unload_command:
            assert unload_command.wait_for_shutdown(timeout=20)
        assert unload_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=["Unloaded component 1 from '/ComponentManager' container"],
            text=unload_command.output,
            strict=True
        )
        # Test the unique id for loaded nodes.
        with self.launch_node_command(
                arguments=[
                    'load', '/ComponentManager',
                    'demo_nodes_cpp', 'demo_nodes_cpp::Talker']) as talker_node:
            assert talker_node.wait_for_shutdown(timeout=20)
        assert talker_node.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                "Loaded component 3 into '/ComponentManager' "
                "container node as '/talker'"],
            text=talker_node.output,
            strict=True
        )
        # Test we can load the same node more than once.
        with self.launch_node_command(
                arguments=[
                    'load', '/ComponentManager',
                    'demo_nodes_cpp', 'demo_nodes_cpp::Talker']) as talker_node:
            assert talker_node.wait_for_shutdown(timeout=20)
        assert talker_node.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                "Loaded component 4 into '/ComponentManager' "
                "container node as '/talker'"],
            text=talker_node.output,
            strict=True
        )
        with self.launch_node_command(
                arguments=['list']) as list_command:
            assert list_command.wait_for_shutdown(timeout=20)
        assert list_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_text=LIST_THREE_NODES,
            text=list_command.output,
            strict=True
        )

        # Unexisting class in existing plugin
        with self.launch_node_command(
                arguments=[
                    'load', '/ComponentManager',
                    'demo_nodes_cpp', 'demo_nodes_cpp::NonExistingPlugin']) as command:
            assert command.wait_for_shutdown(timeout=20)
        assert command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=[
                'Failed to load component: '
                'Failed to find class with the requested plugin name.'],
            text=command.output,
            strict=True
        )
        # Test unexisting plugin
        with self.launch_node_command(
                arguments=[
                    'load', '/ComponentManager',
                    'non_existing_plugin', 'non_existing_plugin::Test']) as command:
            assert command.wait_for_shutdown(timeout=20)
        assert command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=[
                'Failed to load component: '
                'Could not find requested resource in ament index'],
            text=command.output,
            strict=True
        )
        with self.launch_node_command(
                arguments=[
                    'load', '/ComponentManager',
                    'demo_nodes_cpp', 'demo_nodes_cpp::Talker']) as talker_node:
            assert talker_node.wait_for_shutdown(timeout=20)
        assert talker_node.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                "Loaded component 5 into '/ComponentManager' "
                "container node as '/talker'"],
            text=talker_node.output,
            strict=True
        )
        # Verify that we added just one talker and not the non existent nodes.
        with self.launch_node_command(
                arguments=['list']) as list_command:
            assert list_command.wait_for_shutdown(timeout=20)
        assert list_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_text=LIST_FOUR_NODES,
            text=list_command.output,
            strict=True
        )
