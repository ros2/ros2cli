# Copyright 2019 Canonical Ltd
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

EXPECTED_PARAMETER_FILE = (
    f'{TEST_NODE}:\n'
    '  ros__parameters:\n'
    '    bool_array_param:\n'
    '    - false\n'
    '    - false\n'
    '    - true\n'
    '    bool_param: true\n'
    '    double_array_param:\n'
    '    - 3.125\n'
    '    - 6.25\n'
    '    - 12.5\n'
    '    double_param: 1.23\n'
    '    foo:\n'
    '      bar:\n'
    '        str_param: foobar\n'
    '      str_param: foo\n'
    '    int_array_param:\n'
    '    - 1\n'
    '    - 2\n'
    '    - 3\n'
    '    int_param: 42\n'
    '    str_array_param:\n'
    '    - foo\n'
    '    - bar\n'
    '    - baz\n'
    '    str_param: Hello World\n'
    '    use_sim_time: false\n'
)


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation):
    path_to_fixtures = os.path.join(os.path.dirname(__file__), 'fixtures')
    additional_env = {'RMW_IMPLEMENTATION': rmw_implementation}

    # Parameter node test fixture
    path_to_parameter_node_script = os.path.join(path_to_fixtures, 'parameter_node.py')
    parameter_node = Node(
        executable=sys.executable,
        name=TEST_NODE,
        namespace=TEST_NAMESPACE,
        arguments=[path_to_parameter_node_script],
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


class TestVerbDump(unittest.TestCase):

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
        def launch_param_dump_command(self, arguments):
            param_dump_command_action = ExecuteProcess(
                cmd=['ros2', 'param', 'dump', *arguments],
                additional_env={
                    'RMW_IMPLEMENTATION': rmw_implementation,
                },
                name='ros2param-dump-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, param_dump_command_action, proc_info, proc_output,
                output_filter=rmw_implementation_filter
            ) as param_dump_command:
                yield param_dump_command
        cls.launch_param_dump_command = launch_param_dump_command

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
                    and f'{TEST_NAMESPACE}/{TEST_NODE}/get_parameters' in service_names
                ):
                    timed_out = False
                    break
        if timed_out:
            self.fail(f'CLI daemon failed to find test node after {TEST_TIMEOUT} seconds')

    def _output_file(self):
        return f'{TEST_NAMESPACE}/{TEST_NODE}'.lstrip('/').replace('/', '__') + '.yaml'

    def test_verb_dump_invalid_node(self):
        with self.launch_param_dump_command(arguments=['invalid_node']) as param_dump_command:
            assert param_dump_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_dump_command.exit_code != launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Node not found'],
            text=param_dump_command.output,
            strict=True
        )
        with self.launch_param_dump_command(
            arguments=[f'invalid_ns/{TEST_NODE}']
        ) as param_dump_command:
            assert param_dump_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_dump_command.exit_code != launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Node not found'],
            text=param_dump_command.output,
            strict=True
        )

    def test_verb_dump_invalid_path(self):
        with self.launch_param_dump_command(
            arguments=[f'{TEST_NAMESPACE}/{TEST_NODE}', '--output-dir', 'invalid_path']
        ) as param_dump_command:
            assert param_dump_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_dump_command.exit_code != launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=["'invalid_path' is not a valid directory."],
            text=param_dump_command.output,
            strict=True
        )

    def test_verb_dump(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            with self.launch_param_dump_command(
                arguments=[f'{TEST_NAMESPACE}/{TEST_NODE}', '--output-dir', tmpdir]
            ) as param_dump_command:
                assert param_dump_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
            assert param_dump_command.exit_code == launch_testing.asserts.EXIT_OK
            assert launch_testing.tools.expect_output(
                expected_lines=[''],
                text=param_dump_command.output,
                strict=True
            )
            # Compare generated parameter file against expected
            generated_param_file = os.path.join(tmpdir, self._output_file())
            assert (open(generated_param_file, 'r').read() == EXPECTED_PARAMETER_FILE)

    def test_verb_dump_print(self):
        with self.launch_param_dump_command(
            arguments=[f'{TEST_NAMESPACE}/{TEST_NODE}', '--print']
        ) as param_dump_command:
            assert param_dump_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_dump_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_text=EXPECTED_PARAMETER_FILE + '\n',
            text=param_dump_command.output,
            strict=True
        )
        # If both '--output-dir' and '--print' options are provided, ensure it only prints
        # and no file is written
        with tempfile.TemporaryDirectory() as tmpdir:
            with self.launch_param_dump_command(
                arguments=[f'{TEST_NAMESPACE}/{TEST_NODE}', '--output-dir', tmpdir, '--print']
            ) as param_dump_command:
                assert param_dump_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
            assert param_dump_command.exit_code == launch_testing.asserts.EXIT_OK
            assert launch_testing.tools.expect_output(
                expected_text=EXPECTED_PARAMETER_FILE + '\n',
                text=param_dump_command.output,
                strict=True
            )
            # Make sure the file was not create, thus '--print' did preempt
            not_generated_param_file = os.path.join(tmpdir, self._output_file())
            assert not os.path.exists(not_generated_param_file)
