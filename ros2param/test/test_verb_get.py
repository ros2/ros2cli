# Copyright 2026 Sony Corporation
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
from launch.actions import RegisterEventHandler
from launch.actions import ResetEnvironment
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
from launch_testing_ros.actions import EnableRmwIsolation
import launch_testing_ros.tools

import pytest

import rclpy
from rclpy.utilities import get_available_rmw_implementations

from ros2cli.helpers import get_rmw_additional_env
from ros2cli.node.strategy import NodeStrategy


TEST_NODE1 = 'test_node1'
TEST_NODE2 = 'test_node2'
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
    additional_env = get_rmw_additional_env(rmw_implementation)
    set_env_actions = [SetEnvironmentVariable(k, v) for k, v in additional_env.items()]

    # Parameter node test fixtures
    path_to_parameter_node_script = path_to_fixtures / 'parameter_node.py'
    parameter_node1 = Node(
        executable=sys.executable,
        name=TEST_NODE1,
        namespace=TEST_NAMESPACE,
        arguments=[str(path_to_parameter_node_script)],
    )
    parameter_node2 = Node(
        executable=sys.executable,
        name=TEST_NODE2,
        namespace=TEST_NAMESPACE,
        arguments=[str(path_to_parameter_node_script)],
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'daemon', 'stop'],
            name='daemon-stop',
            on_exit=[
                *set_env_actions,
                EnableRmwIsolation(),
                RegisterEventHandler(OnShutdown(on_shutdown=[
                    # Stop daemon in isolated environment with proper ROS_DOMAIN_ID
                    ExecuteProcess(
                        cmd=['ros2', 'daemon', 'stop'],
                        name='daemon-stop-isolated',
                        # Use the same isolated environment
                        additional_env=dict(additional_env),
                    ),
                    # This must be done after stopping the daemon in the isolated environment
                    ResetEnvironment(),
                ])),
                ExecuteProcess(
                    cmd=['ros2', 'daemon', 'start'],
                    name='daemon-start',
                    on_exit=[
                        parameter_node1,
                        parameter_node2,
                        launch_testing.actions.ReadyToTest(),
                    ],
                )
            ]
        ),
    ])


class TestVerbGet(unittest.TestCase):

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
        def launch_param_get_command(self, arguments):
            param_get_command_action = ExecuteProcess(
                cmd=['ros2', 'param', 'get', *arguments],
                name='ros2param-get-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, param_get_command_action, proc_info, proc_output,
                output_filter=rmw_implementation_filter
            ) as param_get_command:
                yield param_get_command
        cls.launch_param_get_command = launch_param_get_command

    def setUp(self):
        start_time = time.time()
        timed_out = True
        with NodeStrategy(None) as node:
            while (time.time() - start_time) < TEST_TIMEOUT:
                try:
                    services1 = node.get_service_names_and_types_by_node(
                        TEST_NODE1, TEST_NAMESPACE)
                    services2 = node.get_service_names_and_types_by_node(
                        TEST_NODE2, TEST_NAMESPACE)
                except rclpy.node.NodeNameNonExistentError:
                    continue
                except ConnectionRefusedError:
                    continue
                except xmlrpc.client.Fault as e:
                    if 'NodeNameNonExistentError' in e.faultString:
                        continue
                    raise

                service_names1 = [name_type_tuple[0] for name_type_tuple in services1]
                service_names2 = [name_type_tuple[0] for name_type_tuple in services2]
                if (
                    len(service_names1) > 0
                    and f'{TEST_NAMESPACE}/{TEST_NODE1}/get_parameters' in service_names1
                    and len(service_names2) > 0
                    and f'{TEST_NAMESPACE}/{TEST_NODE2}/get_parameters' in service_names2
                ):
                    timed_out = False
                    break
        if timed_out:
            self.fail(f'CLI daemon failed to find test nodes after {TEST_TIMEOUT} seconds')

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_get_single_node(self):
        """Test getting a parameter from a specific node (existing behavior)."""
        with self.launch_param_get_command(
            arguments=[f'{TEST_NAMESPACE}/{TEST_NODE1}', 'bool_param']
        ) as param_get_command:
            assert param_get_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_get_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Boolean value is: True'],
            text=param_get_command.output,
            strict=False
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_get_all_nodes(self):
        """Test getting a parameter from all nodes (new feature)."""
        with self.launch_param_get_command(
            arguments=['use_sim_time']
        ) as param_get_command:
            assert param_get_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_get_command.exit_code == launch_testing.asserts.EXIT_OK
        # Should contain output for both test nodes
        output = param_get_command.output
        assert f'{TEST_NAMESPACE}/{TEST_NODE1}' in output
        assert f'{TEST_NAMESPACE}/{TEST_NODE2}' in output
        assert 'Boolean value is:' in output

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_get_all_nodes_hide_type(self):
        """Test getting a parameter from all nodes with --hide-type."""
        with self.launch_param_get_command(
            arguments=['use_sim_time', '--hide-type']
        ) as param_get_command:
            assert param_get_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_get_command.exit_code == launch_testing.asserts.EXIT_OK
        # Should contain node names and values but not type info
        output = param_get_command.output
        assert f'{TEST_NAMESPACE}/{TEST_NODE1}' in output
        assert f'{TEST_NAMESPACE}/{TEST_NODE2}' in output
        assert 'Boolean value is:' not in output

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_get_all_nodes_nonexistent_param(self):
        """Test getting a nonexistent parameter from all nodes."""
        with self.launch_param_get_command(
            arguments=['nonexistent_param']
        ) as param_get_command:
            assert param_get_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_get_command.exit_code == launch_testing.asserts.EXIT_OK
        # Should report that parameter is not set on any node
        assert launch_testing.tools.expect_output(
            expected_lines=["Parameter 'nonexistent_param' not set on any node"],
            text=param_get_command.output,
            strict=False
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_get_multiple_params(self):
        """Test getting multiple parameters at once from a specific node."""
        with self.launch_param_get_command(
            arguments=[f'{TEST_NAMESPACE}/{TEST_NODE1}', 'bool_param', 'int_param']
        ) as param_get_command:
            assert param_get_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_get_command.exit_code == launch_testing.asserts.EXIT_OK
        output = param_get_command.output
        # Each parameter should be prefixed with its name
        assert 'bool_param:' in output
        assert 'Boolean value is: True' in output
        assert 'int_param:' in output
        assert 'Integer value is: 42' in output

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_get_multiple_params_hide_type(self):
        """Test getting multiple parameters at once with --hide-type."""
        with self.launch_param_get_command(
            arguments=[
                f'{TEST_NAMESPACE}/{TEST_NODE1}', 'bool_param', 'int_param', '--hide-type'
            ]
        ) as param_get_command:
            assert param_get_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_get_command.exit_code == launch_testing.asserts.EXIT_OK
        output = param_get_command.output
        # Should prefix each with param name and suppress type label
        assert 'bool_param: True' in output
        assert 'int_param: 42' in output
        assert 'Boolean value is:' not in output
        assert 'Integer value is:' not in output

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_get_multiple_params_three(self):
        """Test getting three parameters at once from a specific node."""
        with self.launch_param_get_command(
            arguments=[
                f'{TEST_NAMESPACE}/{TEST_NODE1}',
                'bool_param', 'int_param', 'str_param'
            ]
        ) as param_get_command:
            assert param_get_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_get_command.exit_code == launch_testing.asserts.EXIT_OK
        output = param_get_command.output
        assert 'bool_param:' in output
        assert 'Boolean value is: True' in output
        assert 'int_param:' in output
        assert 'Integer value is: 42' in output
        assert 'str_param:' in output
        assert 'String value is: Hello World' in output

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_get_multiple_params_with_nonexistent(self):
        """
        Test getting multiple params where one does not exist on the node.

        When any parameter in the batch is undeclared, the get_parameters service
        returns an empty values list for the whole request.  The command returns
        a diagnostic message listing all requested parameter names and exits with
        a non-zero exit code (ros2param treats a returned string from main() as an
        error).
        """
        with self.launch_param_get_command(
            arguments=[
                f'{TEST_NAMESPACE}/{TEST_NODE1}', 'bool_param', 'nonexistent_param'
            ]
        ) as param_get_command:
            assert param_get_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        # main() returns an error string, that exits code 1
        assert param_get_command.exit_code != launch_testing.asserts.EXIT_OK
        output = param_get_command.output
        # Both parameter names and the diagnostic text should appear
        assert 'bool_param' in output
        assert 'nonexistent_param' in output
        assert 'not available on node' in output
