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
from pathlib import Path
import sys
import time
import unittest
import xmlrpc

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

import rclpy
from rclpy.utilities import get_available_rmw_implementations

from ros2cli.node.strategy import NodeStrategy


TEST_NODE = 'parameter_delete_node'
TEST_NAMESPACE = '/test'

TEST_TIMEOUT = 20.0

# Skip cli tests on Windows while they exhibit pathological behavior
# https://github.com/ros2/build_farmer/issues/248
if sys.platform.startswith('win'):
    pytest.skip(
        'CLI tests can block for a pathological amount of time on Windows.',
        allow_module_level=True)


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation):
    path_to_fixtures = Path(__file__).parent / 'fixtures'
    additional_env = {'RMW_IMPLEMENTATION': rmw_implementation}

    path_to_parameter_node_script = path_to_fixtures / 'parameter_delete_node.py'
    parameter_node = Node(
        executable=sys.executable,
        name=TEST_NODE,
        namespace=TEST_NAMESPACE,
        arguments=[str(path_to_parameter_node_script)],
        additional_env=additional_env
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'daemon', 'stop'],
            name='daemon-stop',
            on_exit=[
                ExecuteProcess(
                    cmd=['ros2', 'daemon', 'start'],
                    name='daemon-start',
                    on_exit=[
                        parameter_node,
                        launch_testing.actions.ReadyToTest(),
                    ],
                    additional_env=additional_env
                )
            ]
        ),
    ])


class TestVerbDelete(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output,
        rmw_implementation
    ):
        rmw_implementation_filter = launch_testing_ros.tools.basic_output_filter(
            filtered_rmw_implementation=rmw_implementation
        )

        @contextlib.contextmanager
        def launch_param_delete_command(self, arguments):
            param_delete_command_action = ExecuteProcess(
                cmd=['ros2', 'param', 'delete', *arguments],
                additional_env={
                    'RMW_IMPLEMENTATION': rmw_implementation,
                },
                name='ros2param-delete-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, param_delete_command_action, proc_info, proc_output,
                output_filter=rmw_implementation_filter
            ) as param_delete_command:
                yield param_delete_command
        cls.launch_param_delete_command = launch_param_delete_command

        @contextlib.contextmanager
        def launch_param_set_command(self, arguments):
            param_set_command_action = ExecuteProcess(
                cmd=['ros2', 'param', 'set', *arguments],
                additional_env={
                    'RMW_IMPLEMENTATION': rmw_implementation,
                },
                name='ros2param-set-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, param_set_command_action, proc_info, proc_output,
                output_filter=rmw_implementation_filter
            ) as param_set_command:
                yield param_set_command
        cls.launch_param_set_command = launch_param_set_command

    def setUp(self):
        start_time = time.time()
        timed_out = True
        with NodeStrategy(None) as node:
            while (time.time() - start_time) < TEST_TIMEOUT:
                try:
                    services = node.get_service_names_and_types_by_node(
                        TEST_NODE, TEST_NAMESPACE)
                except rclpy.node.NodeNameNonExistentError:
                    continue
                except ConnectionRefusedError:
                    continue
                except xmlrpc.client.Fault as e:
                    if 'NodeNameNonExistentError' in e.faultString:
                        continue
                    raise

                service_names = [name_type_tuple[0] for name_type_tuple in services]
                if (
                    len(service_names) > 0
                    and f'{TEST_NAMESPACE}/{TEST_NODE}/set_parameters' in service_names
                ):
                    timed_out = False
                    break
        if timed_out:
            self.fail(f'CLI daemon failed to find test node after {TEST_TIMEOUT} seconds')

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_delete_param(self):
        """Test deleting an existing parameter."""
        # 1. Set a dynamic parameter first
        # (since statically declared parameters cannot be undeclared)
        with self.launch_param_set_command(
            arguments=[f'{TEST_NAMESPACE}/{TEST_NODE}', 'dynamic_param', '123']
        ) as param_set_command:
            assert param_set_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_set_command.exit_code == launch_testing.asserts.EXIT_OK

        # 2. Delete the dynamic parameter
        with self.launch_param_delete_command(
            arguments=[f'{TEST_NAMESPACE}/{TEST_NODE}', 'dynamic_param']
        ) as param_delete_command:
            assert param_delete_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_delete_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Deleted parameter successfully'],
            text=param_delete_command.output,
            strict=False
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_delete_node_not_found(self):
        """Test that 'Node not found' is reported for a nonexistent node."""
        with self.launch_param_delete_command(
            arguments=['/nonexistent_node', 'dynamic_param']
        ) as param_delete_command:
            assert param_delete_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_delete_command.exit_code != launch_testing.asserts.EXIT_OK
        assert 'Node not found' in param_delete_command.output
