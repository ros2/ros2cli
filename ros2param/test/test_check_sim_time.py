# Copyright 2026 Tim Wendt
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
import xmlrpc.client

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

TEST_NODE_TRUE = 'test_node1'
TEST_NODE_FALSE = 'test_node2'
TEST_NODE_HANG = 'test_node3'
TEST_NODE_NOT = 'test_node4'
TEST_NAMESPACE = '/test'

TEST_TIMEOUT = 20.0

# Skip cli tests on Windows while they exhibit pathological behavior
# https://github.com/ros2/build_farmer/issues/248
if sys.platform.startswith('win'):
    pytest.skip(
        'CLI tests can block for a pathological amount of time on Windows.',
        allow_module_level=True,
    )


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
        name=TEST_NODE_TRUE,
        namespace=TEST_NAMESPACE,
        arguments=[str(path_to_parameter_node_script)],
        parameters=[{'use_sim_time': True}],
    )
    parameter_node2 = Node(
        executable=sys.executable,
        name=TEST_NODE_FALSE,
        namespace=TEST_NAMESPACE,
        arguments=[str(path_to_parameter_node_script)],
        parameters=[{'use_sim_time': False}],
    )

    path_to_hang_node_script = path_to_fixtures / 'param_list_hang_node.py'
    parameter_node3 = Node(
        executable=sys.executable,
        name=TEST_NODE_HANG,
        namespace=TEST_NAMESPACE,
        arguments=[str(path_to_hang_node_script)],
        parameters=[{'use_sim_time': True}],
    )

    path_to_no_sim_time_node_script = path_to_fixtures / 'no_sim_time_node.py'
    parameter_node4 = Node(
        executable=sys.executable,
        name=TEST_NODE_NOT,
        namespace=TEST_NAMESPACE,
        arguments=[str(path_to_no_sim_time_node_script)],
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
                        parameter_node3,
                        parameter_node4,
                        launch_testing.actions.ReadyToTest(),
                    ],
                )
            ]
        ),
    ])


class TestVerbCheckSimTime(unittest.TestCase):

    @classmethod
    def setUpClass(cls, launch_service, proc_info, proc_output, rmw_implementation):
        rmw_implementation_filter = launch_testing_ros.tools.basic_output_filter(
            filtered_rmw_implementation=rmw_implementation
        )

        @contextlib.contextmanager
        def launch_check_sim_time_command(self, arguments):
            action = ExecuteProcess(
                cmd=['ros2', 'param', 'check_sim_time', *arguments],
                name='ros2param-check-sim-time-cli',
                output='screen',
            )
            with launch_testing.tools.launch_process(
                launch_service,
                action,
                proc_info,
                proc_output,
                output_filter=rmw_implementation_filter,
            ) as proc:
                yield proc

        cls.launch_check_sim_time_command = launch_check_sim_time_command

    def setUp(self):
        start_time = time.time()
        timed_out = True

        with NodeStrategy(None) as node:
            while (time.time() - start_time) < TEST_TIMEOUT:
                try:
                    services_true = node.get_service_names_and_types_by_node(
                        TEST_NODE_TRUE, TEST_NAMESPACE
                    )
                    services_false = node.get_service_names_and_types_by_node(
                        TEST_NODE_FALSE, TEST_NAMESPACE
                    )
                    services_hang = node.get_service_names_and_types_by_node(
                        TEST_NODE_HANG, TEST_NAMESPACE
                    )
                    services_not = node.get_service_names_and_types_by_node(
                        TEST_NODE_NOT, TEST_NAMESPACE
                    )
                except rclpy.node.NodeNameNonExistentError:
                    continue
                except ConnectionRefusedError:
                    continue
                except xmlrpc.client.Fault as e:
                    if 'NodeNameNonExistentError' in e.faultString:
                        continue
                    raise

                service_names_true = [s[0] for s in services_true]
                service_names_false = [s[0] for s in services_false]
                service_names_hang = [s[0] for s in services_hang]
                service_names_not = [s[0] for s in services_not]

                if (
                    f'{TEST_NAMESPACE}/{TEST_NODE_TRUE}/get_parameters'
                    in service_names_true
                    and f'{TEST_NAMESPACE}/{TEST_NODE_FALSE}/get_parameters'
                    in service_names_false
                    and f'{TEST_NAMESPACE}/{TEST_NODE_HANG}/get_parameters'
                    in service_names_hang
                    and f'{TEST_NAMESPACE}/{TEST_NODE_NOT}/get_parameters'
                    in service_names_not
                ):
                    timed_out = False
                    break

        if timed_out:
            self.fail(
                f'CLI daemon failed to find test nodes after {TEST_TIMEOUT} seconds'
            )

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_check_sim_time_table(self):
        with self.launch_check_sim_time_command(arguments=[]) as proc:
            assert proc.wait_for_shutdown(timeout=TEST_TIMEOUT)

        assert proc.exit_code == launch_testing.asserts.EXIT_OK, proc.output

        assert launch_testing.tools.expect_output(
            expected_lines=[
                'use_sim_time:',
                f'{TEST_NAMESPACE}/{TEST_NODE_TRUE}: True',
                f'{TEST_NAMESPACE}/{TEST_NODE_FALSE}: False',
                (
                    f'{TEST_NAMESPACE}/{TEST_NODE_HANG}: '
                    '<error: Exception while calling service of node '
                    f'{TEST_NAMESPACE}/{TEST_NODE_HANG}: None>'
                ),
                f'{TEST_NAMESPACE}/{TEST_NODE_NOT}: <not declared>',
            ],
            text=proc.output,
            strict=True,
        ), f'actual output:\n{proc.output}'
