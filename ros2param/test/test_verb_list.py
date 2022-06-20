# Copyright 2021 Open Source Robotics Foundation, Inc.
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

TEST_NODE = 'test_node'
TEST_NAMESPACE = '/foo'

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

    # Parameter node test fixture
    path_to_parameter_node_script = path_to_fixtures / 'parameter_node.py'
    parameter_node = Node(
        executable=sys.executable,
        name=TEST_NODE,
        namespace=TEST_NAMESPACE,
        arguments=[str(path_to_parameter_node_script)],
        additional_env=additional_env
    )

    return LaunchDescription([
        # TODO(jacobperron): Provide a common RestartCliDaemon launch action in ros2cli
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


# Flaky on Galactic: https://github.com/ros2/ros2cli/issues/630
@pytest.mark.xfail
class TestVerbList(unittest.TestCase):

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
        def launch_param_list_command(self, arguments):
            param_list_command_action = ExecuteProcess(
                cmd=['ros2', 'param', 'list', *arguments],
                additional_env={
                    'RMW_IMPLEMENTATION': rmw_implementation,
                },
                name='ros2param-list-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, param_list_command_action, proc_info, proc_output,
                output_filter=rmw_implementation_filter
            ) as param_list_command:
                yield param_list_command
        cls.launch_param_list_command = launch_param_list_command

    def setUp(self):
        # Ensure the daemon node is running and discovers the test node
        start_time = time.time()
        timed_out = True
        with NodeStrategy(None) as node:
            while (time.time() - start_time) < TEST_TIMEOUT:
                # TODO(jacobperron): Create a generic 'CliNodeError' so we can treat errors
                #                    from DirectNode and DaemonNode the same
                try:
                    services = node.get_service_names_and_types_by_node(TEST_NODE, TEST_NAMESPACE)
                except rclpy.node.NodeNameNonExistentError:
                    continue
                except xmlrpc.client.Fault as e:
                    if 'NodeNameNonExistentError' in e.faultString:
                        continue
                    raise

                service_names = [name_type_tuple[0] for name_type_tuple in services]
                if (
                    len(service_names) > 0
                    and f'{TEST_NAMESPACE}/{TEST_NODE}/list_parameters' in service_names
                ):
                    timed_out = False
                    break
        if timed_out:
            self.fail(f'CLI daemon failed to find test node after {TEST_TIMEOUT} seconds')

    def test_verb_list_invalid_node(self):
        with self.launch_param_list_command(arguments=['invalid_node']) as param_list_command:
            assert param_list_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_list_command.exit_code != launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Node not found'],
            text=param_list_command.output,
            strict=True
        )
        with self.launch_param_list_command(
            arguments=[f'invalid_ns/{TEST_NODE}']
        ) as param_list_command:
            assert param_list_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_list_command.exit_code != launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Node not found'],
            text=param_list_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_list(self):
        with self.launch_param_list_command(
            arguments=[f'{TEST_NAMESPACE}/{TEST_NODE}']
        ) as param_list_command:
            assert param_list_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_list_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                '  bool_array_param',
                '  bool_param',
                '  double_array_param',
                '  double_param',
                '  foo.bar.str_param',
                '  foo.str_param',
                '  int_array_param',
                '  int_param',
                '  str_array_param',
                '  str_param',
                '  use_sim_time'],
            text=param_list_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_verb_list_filter(self):
        with self.launch_param_list_command(
            arguments=[f'{TEST_NAMESPACE}/{TEST_NODE}', '--filter', 'bool.*']
        ) as param_list_command:
            assert param_list_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_list_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                '  bool_array_param',
                '  bool_param'],
            text=param_list_command.output,
            strict=True
        ), f'actual output: {param_list_command.output}'
