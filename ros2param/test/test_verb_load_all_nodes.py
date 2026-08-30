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
import os
import sys
import tempfile
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


TEST_NODE = 'test_node'
TEST_NAMESPACE = '/foo'
TEST_TIMEOUT = 20.0


if sys.platform.startswith('win'):
    pytest.skip(
        'CLI tests can block for a pathological amount of time on Windows.',
        allow_module_level=True,
    )


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation):
    path_to_fixtures = os.path.join(os.path.dirname(__file__), 'fixtures')
    additional_env = get_rmw_additional_env(rmw_implementation)
    set_env_actions = [SetEnvironmentVariable(k, v) for k, v in additional_env.items()]
    parameter_node = Node(
        executable=sys.executable,
        name=TEST_NODE,
        namespace=TEST_NAMESPACE,
        arguments=[os.path.join(path_to_fixtures, 'parameter_node.py')],
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
                    on_exit=[parameter_node, launch_testing.actions.ReadyToTest()],
                ),
            ],
        ),
    ])


class TestVerbLoadAllNodes(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output,
        rmw_implementation,
    ):
        output_filter = launch_testing_ros.tools.basic_output_filter(
            filtered_rmw_implementation=rmw_implementation
        )

        @contextlib.contextmanager
        def launch_cli(self, arguments):
            action = ExecuteProcess(
                cmd=['ros2', 'param', *arguments],
                name='ros2param-cli',
                output='screen',
            )
            with launch_testing.tools.launch_process(
                launch_service,
                action,
                proc_info,
                proc_output,
                output_filter=output_filter,
            ) as process:
                yield process

        cls.launch_cli = launch_cli

    def setUp(self):
        start_time = time.time()
        with NodeStrategy(None) as node:
            while (time.time() - start_time) < TEST_TIMEOUT:
                try:
                    services = node.get_service_names_and_types_by_node(
                        TEST_NODE, TEST_NAMESPACE
                    )
                except rclpy.node.NodeNameNonExistentError:
                    continue
                except ConnectionRefusedError:
                    continue
                except xmlrpc.client.Fault as e:
                    if 'NodeNameNonExistentError' in e.faultString:
                        continue
                    raise

                if f'{TEST_NAMESPACE}/{TEST_NODE}/set_parameters' in {
                    name for name, _ in services
                }:
                    return
        self.fail(f'CLI daemon failed to find test node after {TEST_TIMEOUT} seconds')

    def test_load_parameter_file_without_node_name(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = os.path.join(tmpdir, 'params.yaml')
            with open(filepath, 'w') as f:
                f.write(
                    f'{TEST_NAMESPACE}/{TEST_NODE}:\n'
                    '  ros__parameters:\n'
                    '    str_param: Loaded by all-node mode\n'
                )

            with self.launch_cli(['load', filepath, '--timeout', '3']) as process:
                assert process.wait_for_shutdown(timeout=TEST_TIMEOUT)
            assert process.exit_code == launch_testing.asserts.EXIT_OK

            with self.launch_cli([
                'get',
                f'{TEST_NAMESPACE}/{TEST_NODE}',
                'str_param',
                '--hide-type',
                '--timeout',
                '3',
            ]) as process:
                assert process.wait_for_shutdown(timeout=TEST_TIMEOUT)
            assert process.exit_code == launch_testing.asserts.EXIT_OK
            assert launch_testing.tools.expect_output(
                expected_lines=['Loaded by all-node mode'],
                text=process.output,
                strict=True,
            )
