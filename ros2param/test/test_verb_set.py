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


TEST_NODE = 'test_node1'
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

    path_to_parameter_node_script = path_to_fixtures / 'parameter_node.py'
    parameter_node = Node(
        executable=sys.executable,
        name=TEST_NODE,
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
                        parameter_node,
                        launch_testing.actions.ReadyToTest(),
                    ],
                )
            ]
        ),
    ])


class TestVerbSet(unittest.TestCase):

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
        def launch_param_set_command(self, arguments):
            param_set_command_action = ExecuteProcess(
                cmd=['ros2', 'param', 'set', *arguments],
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

    # -------------------------------------------------------------------------
    # Single-parameter tests
    # -------------------------------------------------------------------------

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_set_single_param_bool(self):
        """Test setting a single boolean parameter (existing behavior)."""
        with self.launch_param_set_command(
            arguments=[f'{TEST_NAMESPACE}/{TEST_NODE}', 'bool_param', 'false']
        ) as param_set_command:
            assert param_set_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_set_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Set parameter successful'],
            text=param_set_command.output,
            strict=False
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_set_single_param_int(self):
        """Test setting a single integer parameter (existing behavior)."""
        with self.launch_param_set_command(
            arguments=[f'{TEST_NAMESPACE}/{TEST_NODE}', 'int_param', '99']
        ) as param_set_command:
            assert param_set_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_set_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Set parameter successful'],
            text=param_set_command.output,
            strict=False
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_set_single_param_string(self):
        """Test setting a single string parameter (existing behavior)."""
        with self.launch_param_set_command(
            arguments=[f'{TEST_NAMESPACE}/{TEST_NODE}', 'str_param', 'new_value']
        ) as param_set_command:
            assert param_set_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_set_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Set parameter successful'],
            text=param_set_command.output,
            strict=False
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_set_node_not_found(self):
        """Test that 'Node not found' is reported for a nonexistent node."""
        with self.launch_param_set_command(
            arguments=['/nonexistent_node', 'bool_param', 'true']
        ) as param_set_command:
            assert param_set_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_set_command.exit_code != launch_testing.asserts.EXIT_OK
        assert 'Node not found' in param_set_command.output

    # -------------------------------------------------------------------------
    # Multiple-parameter tests (new feature)
    # -------------------------------------------------------------------------

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_set_multiple_params(self):
        """Test setting two parameters at once."""
        with self.launch_param_set_command(
            arguments=[
                f'{TEST_NAMESPACE}/{TEST_NODE}',
                'bool_param', 'false',
                'int_param', '7',
            ]
        ) as param_set_command:
            assert param_set_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_set_command.exit_code == launch_testing.asserts.EXIT_OK
        output = param_set_command.output
        # Multi-param output is prefixed with each parameter name
        assert 'bool_param: Set parameter successful' in output
        assert 'int_param: Set parameter successful' in output

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_set_multiple_params_three(self):
        """Test setting three parameters at once."""
        with self.launch_param_set_command(
            arguments=[
                f'{TEST_NAMESPACE}/{TEST_NODE}',
                'bool_param', 'true',
                'int_param', '42',
                'str_param', 'Hello World',
            ]
        ) as param_set_command:
            assert param_set_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_set_command.exit_code == launch_testing.asserts.EXIT_OK
        output = param_set_command.output
        assert 'bool_param: Set parameter successful' in output
        assert 'int_param: Set parameter successful' in output
        assert 'str_param: Set parameter successful' in output

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_set_multiple_params_mixed_types(self):
        """Test setting parameters of different types at once."""
        with self.launch_param_set_command(
            arguments=[
                f'{TEST_NAMESPACE}/{TEST_NODE}',
                'bool_param', 'false',
                'double_param', '9.99',
                'str_param', 'updated',
            ]
        ) as param_set_command:
            assert param_set_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_set_command.exit_code == launch_testing.asserts.EXIT_OK
        output = param_set_command.output
        assert 'bool_param: Set parameter successful' in output
        assert 'double_param: Set parameter successful' in output
        assert 'str_param: Set parameter successful' in output

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_set_odd_number_of_args(self):
        """Test that an odd number of positional arguments is rejected."""
        with self.launch_param_set_command(
            arguments=[
                f'{TEST_NAMESPACE}/{TEST_NODE}',
                'bool_param', 'false', 'int_param',  # missing value for int_param
            ]
        ) as param_set_command:
            assert param_set_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        # An odd number of name/value arguments must produce an error exit
        assert param_set_command.exit_code != launch_testing.asserts.EXIT_OK
        assert 'name/value pairs' in param_set_command.output

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_set_multiple_params_one_nonexistent(self):
        """
        Test setting multiple params where one is not declared on the node.

        set_parameters returns one result per parameter.  For an undeclared
        parameter the service sets successful=False with a reason string.
        The declared parameter should be set successfully, the undeclared one
        should report failure; the overall exit code is 0 because the service
        call itself completed without error.
        """
        with self.launch_param_set_command(
            arguments=[
                f'{TEST_NAMESPACE}/{TEST_NODE}',
                'bool_param', 'true',
                'nonexistent_param', 'some_value',
            ]
        ) as param_set_command:
            assert param_set_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_set_command.exit_code == launch_testing.asserts.EXIT_OK
        output = param_set_command.output
        # Declared parameter succeeds
        assert 'bool_param: Set parameter successful' in output
        # Undeclared parameter reports failure
        assert 'nonexistent_param: Setting parameter failed' in output

    # -------------------------------------------------------------------------
    # Atomic tests (--atomic flag)
    # -------------------------------------------------------------------------

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_set_atomic_single_param(self):
        """Test setting a single parameter atomically."""
        with self.launch_param_set_command(
            arguments=[
                f'{TEST_NAMESPACE}/{TEST_NODE}', 'bool_param', 'false', '--atomic'
            ]
        ) as param_set_command:
            assert param_set_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_set_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Set parameters atomically successful'],
            text=param_set_command.output,
            strict=False
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_set_atomic_multiple_params(self):
        """Test setting multiple parameters atomically (all declared)."""
        with self.launch_param_set_command(
            arguments=[
                f'{TEST_NAMESPACE}/{TEST_NODE}',
                'bool_param', 'true',
                'int_param', '42',
                'str_param', 'Hello World',
                '--atomic',
            ]
        ) as param_set_command:
            assert param_set_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_set_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Set parameters atomically successful'],
            text=param_set_command.output,
            strict=False
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_set_atomic_one_nonexistent(self):
        """
        Test atomic set where one parameter is undeclared.

        SetParametersAtomically is all-or-nothing: if any parameter fails the
        entire transaction is rejected and no parameters are changed.  The
        command prints a single failure message and exits cleanly (exit 0)
        because the service call itself completed.
        """
        with self.launch_param_set_command(
            arguments=[
                f'{TEST_NAMESPACE}/{TEST_NODE}',
                'bool_param', 'false',
                'nonexistent_param', 'some_value',
                '--atomic',
            ]
        ) as param_set_command:
            assert param_set_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_set_command.exit_code == launch_testing.asserts.EXIT_OK
        # A single failure message is printed (not per-parameter)
        assert 'Setting parameters atomically failed' in param_set_command.output
        # The successful param name must NOT appear as individually set
        assert 'bool_param: Set parameter' not in param_set_command.output
