# Copyright 2021 PAL Robotics S.L.
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

import yaml

TEST_NODE = 'test_node'
TEST_NAMESPACE = '/foo'

TEST_TIMEOUT = 20.0

INPUT_PARAMETER_FILE = (
    f'{TEST_NAMESPACE}/{TEST_NODE}:\n'
    '  ros__parameters:\n'
    '    bool_array_param:\n'
    '    - true\n'
    '    - false\n'
    '    - false\n'
    '    bool_param: false\n'
    '    double_array_param:\n'
    '    - 2.125\n'
    '    - 1.25\n'
    '    - 2.5\n'
    '    double_param: 1.3\n'
    '    foo:\n'
    '      bar:\n'
    '        str_param: foo_bar\n'
    '      str_param: foobar\n'
    '    int_array_param:\n'
    '    - 42\n'
    '    - 3\n'
    '    - 3\n'
    '    int_param: -42\n'
    '    start_type_description_service: true\n'
    '    str_array_param:\n'
    '    - a_foo\n'
    '    - a_bar\n'
    '    - baz\n'
    '    str_param: Bye World\n'
    '    use_sim_time: false\n'
)
INPUT_WILDCARD_PARAMETER_FILE = (
    '/**:\n'
    '  ros__parameters:\n'
    '    str_param: Wildcard\n'
    '    int_param: 12345\n'
)
INPUT_NODE_OVERLAY_PARAMETER_FILE = (
    f'{TEST_NAMESPACE}/{TEST_NODE}:\n'
    '  ros__parameters:\n'
    '    str_param: Override\n'
)
INPUT_NS_NODE_OVERLAY_PARAMETER_FILE = (
    f'{TEST_NAMESPACE}:\n'
    f'  {TEST_NODE}:\n'
    '    ros__parameters:\n'
    '      str_param: Override\n'
)

# Skip cli tests on Windows while they exhibit pathological behavior
# https://github.com/ros2/build_farmer/issues/248
if sys.platform.startswith('win'):
    pytest.skip(
            'CLI tests can block for a pathological amount of time on Windows.',
            allow_module_level=True)


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


class TestVerbLoad(unittest.TestCase):

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
        def launch_param_load_command(self, arguments):
            param_load_command_action = ExecuteProcess(
                cmd=['ros2', 'param', 'load', *arguments],
                additional_env={
                    'RMW_IMPLEMENTATION': rmw_implementation,
                },
                name='ros2param-load-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, param_load_command_action, proc_info, proc_output,
                output_filter=rmw_implementation_filter
            ) as param_load_command:
                yield param_load_command
        cls.launch_param_load_command = launch_param_load_command

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

    def _write_param_file(self, tmpdir, filename, contents=INPUT_PARAMETER_FILE):
        yaml_path = os.path.join(tmpdir, filename)
        with open(yaml_path, 'w') as f:
            f.write(contents)
            return yaml_path

    def _output_file(self):
        return f'{TEST_NAMESPACE}/{TEST_NODE}'.lstrip('/').replace('/', '__') + '.yaml'

    def test_verb_load_missing_args(self):
        with self.launch_param_load_command(arguments=[]) as param_load_command:
            assert param_load_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_load_command.exit_code != launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['ros2 param load: error: the following arguments are required: '
                            'node_name, parameter_file'],
            text=param_load_command.output,
            strict=False
        )
        with self.launch_param_load_command(arguments=['some_node']) as param_load_command:
            assert param_load_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_load_command.exit_code != launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['ros2 param load: error: the following arguments are required: '
                            'parameter_file'],
            text=param_load_command.output,
            strict=False
        )

    def test_verb_load_invalid_node(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = self._write_param_file(tmpdir, 'params.yaml')
            with self.launch_param_load_command(
                arguments=['invalid_node', filepath]
            ) as param_load_command:
                assert param_load_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
            assert param_load_command.exit_code != launch_testing.asserts.EXIT_OK
            assert launch_testing.tools.expect_output(
                expected_lines=['Node not found'],
                text=param_load_command.output,
                strict=True
            )
            with self.launch_param_load_command(
                arguments=[f'invalid_ns/{TEST_NODE}', filepath]
            ) as param_load_command:
                assert param_load_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
            assert param_load_command.exit_code != launch_testing.asserts.EXIT_OK
            assert launch_testing.tools.expect_output(
                expected_lines=['Node not found'],
                text=param_load_command.output,
                strict=True
            )

    def test_verb_load_invalid_path(self):
        with self.launch_param_load_command(
            arguments=[f'{TEST_NAMESPACE}/{TEST_NODE}', 'invalid_path']
        ) as param_load_command:
            assert param_load_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_load_command.exit_code != launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['No such file or directory'],
            text=param_load_command.output,
            strict=False
        )

    def test_verb_load_timeout(self):
        with self.launch_param_load_command(
            arguments=['invalid_node', 'invalid_path', '--timeout', '2']
        ) as param_load_command:
            assert param_load_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
        assert param_load_command.exit_code != launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Node not found'],
            text=param_load_command.output,
            strict=False
        )

    def test_verb_load(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = self._write_param_file(tmpdir, 'params.yaml')
            with self.launch_param_load_command(
                arguments=[f'{TEST_NAMESPACE}/{TEST_NODE}', filepath, '--timeout', '3']
            ) as param_load_command:
                assert param_load_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
            assert param_load_command.exit_code == launch_testing.asserts.EXIT_OK
            assert launch_testing.tools.expect_output(
                expected_lines=[''],
                text=param_load_command.output,
                strict=True
            )
            # Dump with ros2 param dump and compare that output matches input file
            with self.launch_param_dump_command(
                arguments=[f'{TEST_NAMESPACE}/{TEST_NODE}']
            ) as param_dump_command:
                assert param_dump_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
            assert param_dump_command.exit_code == launch_testing.asserts.EXIT_OK
            assert launch_testing.tools.expect_output(
                expected_text=INPUT_PARAMETER_FILE + '\n',
                text=param_dump_command.output,
                strict=True
            )

    def test_verb_load_wildcard(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            # Try param file with only wildcard
            filepath = self._write_param_file(tmpdir, 'params.yaml', INPUT_WILDCARD_PARAMETER_FILE)
            with self.launch_param_load_command(
                arguments=[f'{TEST_NAMESPACE}/{TEST_NODE}', filepath,
                           '--no-use-wildcard', '--timeout', '3']
            ) as param_load_command:
                assert param_load_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
            assert param_load_command.exit_code != launch_testing.asserts.EXIT_OK
            assert launch_testing.tools.expect_output(
                expected_lines=['Param file does not contain any valid parameters'],
                text=param_load_command.output,
                strict=False
            )

            with self.launch_param_load_command(
                arguments=[f'{TEST_NAMESPACE}/{TEST_NODE}', filepath]
            ) as param_load_command:
                assert param_load_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
            assert param_load_command.exit_code == launch_testing.asserts.EXIT_OK
            assert launch_testing.tools.expect_output(
                expected_lines=[''],
                text=param_load_command.output,
                strict=True
            )
            # Dump with ros2 param and check that wildcard parameters are loaded
            with self.launch_param_dump_command(
                arguments=[f'{TEST_NAMESPACE}/{TEST_NODE}']
            ) as param_dump_command:
                assert param_dump_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
            assert param_dump_command.exit_code == launch_testing.asserts.EXIT_OK
            loaded_params = yaml.safe_load(param_dump_command.output)
            params = loaded_params[f'{TEST_NAMESPACE}/{TEST_NODE}']['ros__parameters']
            assert params['str_param'] == 'Wildcard'
            assert params['int_param'] == 12345

            # Concatenate wildcard + some overlays with absolute node name
            filepath = self._write_param_file(tmpdir, 'params.yaml',
                                              INPUT_WILDCARD_PARAMETER_FILE + '\n' +
                                              INPUT_NODE_OVERLAY_PARAMETER_FILE)
            with self.launch_param_load_command(
                arguments=[f'{TEST_NAMESPACE}/{TEST_NODE}', filepath]
            ) as param_load_command:
                assert param_load_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
            assert param_load_command.exit_code == launch_testing.asserts.EXIT_OK

            # Dump and check that wildcard parameters were overriden if in node namespace
            with self.launch_param_dump_command(
                arguments=[f'{TEST_NAMESPACE}/{TEST_NODE}']
            ) as param_dump_command:
                assert param_dump_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
            assert param_dump_command.exit_code == launch_testing.asserts.EXIT_OK
            loaded_params = yaml.safe_load(param_dump_command.output)
            params = loaded_params[f'{TEST_NAMESPACE}/{TEST_NODE}']['ros__parameters']
            assert params['str_param'] == 'Override'  # Overriden
            assert params['int_param'] == 12345  # Wildcard namespace

            # Concatenate wildcard + some overlays with namespace and base node name
            filepath = self._write_param_file(tmpdir, 'params.yaml',
                                              INPUT_WILDCARD_PARAMETER_FILE + '\n' +
                                              INPUT_NS_NODE_OVERLAY_PARAMETER_FILE)
            with self.launch_param_load_command(
                arguments=[f'{TEST_NAMESPACE}/{TEST_NODE}', filepath]
            ) as param_load_command:
                assert param_load_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
            assert param_load_command.exit_code == launch_testing.asserts.EXIT_OK

            # Dump and check that wildcard parameters were overriden if in node namespace
            with self.launch_param_dump_command(
                arguments=[f'{TEST_NAMESPACE}/{TEST_NODE}']
            ) as param_dump_command:
                assert param_dump_command.wait_for_shutdown(timeout=TEST_TIMEOUT)
            assert param_dump_command.exit_code == launch_testing.asserts.EXIT_OK
            loaded_params = yaml.safe_load(param_dump_command.output)
            params = loaded_params[f'{TEST_NAMESPACE}/{TEST_NODE}']['ros__parameters']
            assert params['str_param'] == 'Override'  # Overriden
            assert params['int_param'] == 12345  # Wildcard namespace
