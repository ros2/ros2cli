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
import os
import sys
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

PARAM_NODE_PARAMETER_LIST = """\
  bool_array
  bool_param
  byte_array
  double_array
  double_param
  int_array
  int_param
  parameter_with_no_value
  str_array
  str_param
  use_sim_time
"""

PARAM_NODE_BOOL_PARAMETER_LIST = """\
  bool_array
  bool_param
"""

ALL_NODES_PARAMETER_LIST = """\
/_hidden_param_node:
  bool_array
  bool_param
  byte_array
  double_array
  double_param
  int_array
  int_param
  parameter_with_no_value
  str_array
  str_param
  use_sim_time
/launch_ros:
  use_sim_time
/param_node:
  bool_array
  bool_param
  byte_array
  double_array
  double_param
  int_array
  int_param
  parameter_with_no_value
  str_array
  str_param
  use_sim_time
"""

DESCRIBE_PARAMETER_TYPE = """\
Parameter name: {}
  Type: {}
  Constraints:{}
"""


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation, ready_fn):
    path_to_param_node_script = os.path.join(
        os.path.dirname(__file__), 'fixtures', 'param_node.py'
    )
    additional_env = {'RMW_IMPLEMENTATION': rmw_implementation}
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
                        # Add test fixture actions.
                        Node(
                            node_executable=sys.executable,
                            arguments=[path_to_param_node_script],
                            node_name='param_node',
                            additional_env=additional_env
                        ),
                        Node(
                            node_executable=sys.executable,
                            arguments=[path_to_param_node_script],
                            node_name='_hidden_param_node',
                            additional_env=additional_env
                        ),
                        OpaqueFunction(function=lambda context: ready_fn())
                    ],
                    additional_env=additional_env
                )
            ]
        ),
    ])


class TestROS2ParamCLI(unittest.TestCase):

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
                cmd=['ros2', 'param', *arguments],
                additional_env={
                    'RMW_IMPLEMENTATION': rmw_implementation,
                    'PYTHONUNBUFFERED': '1'
                },
                name='ros2param-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, node_command_action, proc_info, proc_output,
                output_filter=launch_testing_ros.tools.basic_output_filter(
                    # ignore launch_ros and ros2cli daemon nodes
                    filtered_patterns=['.*ros2cli.*'],
                    filtered_rmw_implementation=rmw_implementation
                )
            ) as node_command:
                yield node_command
        cls.launch_node_command = launch_node_command

    # Set verb tests
    @launch_testing.markers.retry_on_failure(times=3)
    def test_set_invalid_param_existing_node(self):
        with self.launch_node_command(
                arguments=['set', '/param_node', 'unexistent_parameter', '1']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                "Setting parameter failed: ('Invalid access to undeclared parameter(s)', [])"],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_set_valid_param_existing_node(self):
        with self.launch_node_command(
                arguments=['set', '/param_node', 'double_param', '3.14']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Set parameter successful'],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_set_invalid_param_hidden_node_no_hidden_argument(self):
        with self.launch_node_command(
                arguments=[
                    'set', '/_hidden_param_node', 'unexistent_parameter', '1']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Node not found'],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_set_invalid_param_hidden_node_hidden_argument(self):
        with self.launch_node_command(
                arguments=[
                    'set', '/_hidden_param_node',
                    'unexistent_parameter', '1', '--include-hidden-nodes']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                "Setting parameter failed: ('Invalid access to undeclared parameter(s)', [])"],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_set_inexisting_node(self):
        with self.launch_node_command(
                arguments=['set', '/foo/unexisting_node', 'test_param', '3.14']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Node not found'],
            text=node_command.output,
            strict=True
        )

    # Get verb tests without --hide-type flag
    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_invalid_param_existing_node(self):
        with self.launch_node_command(
                arguments=['get', '/param_node', 'unexistent_parameter']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                'Parameter not set'],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_valid_double_param_existing_node(self):
        with self.launch_node_command(
                arguments=['get', '/param_node', 'double_param']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Double value is: 1.23'],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_valid_boolean_param_existing_node(self):
        with self.launch_node_command(
                arguments=['get', '/param_node', 'bool_param']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Boolean value is: True'],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_valid_int_param_existing_node(self):
        with self.launch_node_command(
                arguments=['get', '/param_node', 'int_param']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Integer value is: 42'],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_valid_string_param_existing_node(self):
        with self.launch_node_command(
                arguments=['get', '/param_node', 'str_param']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['String value is: Hello World'],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_valid_string_array_param_existing_node(self):
        with self.launch_node_command(
                arguments=['get', '/param_node', 'str_array']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=["String values are: ['Hello', 'World']"],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_valid_int_array_param_existing_node(self):
        with self.launch_node_command(
                arguments=['get', '/param_node', 'int_array']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=["Integer values are: array('q', [1, 2, 3])"],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_valid_double_array_param_existing_node(self):
        with self.launch_node_command(
                arguments=['get', '/param_node', 'double_array']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=["Double values are: array('d', [1.0, 2.0, 3.0])"],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_valid_boolean_array_param_existing_node(self):
        with self.launch_node_command(
                arguments=['get', '/param_node', 'bool_array']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Boolean values are: [True, False, True]'],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_valid_byte_array_param_existing_node(self):
        with self.launch_node_command(
                arguments=['get', '/param_node', 'byte_array']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=["Byte values are: [b'p', b'v']"],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_valid_unset_param_existing_node(self):
        with self.launch_node_command(
                arguments=['get', '/param_node', 'parameter_with_no_value']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Parameter not set.'],
            text=node_command.output,
            strict=True
        )

    # Get verb tests with --hide-type flag
    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_valid_double_param_hide_type_existing_node(self):
        with self.launch_node_command(
                arguments=['get', '/param_node', 'double_param', '--hide-type']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['1.23'],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_valid_boolean_param_hide_type_existing_node(self):
        with self.launch_node_command(
                arguments=['get', '/param_node', 'bool_param', '--hide-type']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['True'],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_valid_int_param_hide_type_existing_node(self):
        with self.launch_node_command(
                arguments=['get', '/param_node', 'int_param', '--hide-type']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['42'],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_valid_string_param_hide_type_existing_node(self):
        with self.launch_node_command(
                arguments=['get', '/param_node', 'str_param', '--hide-type']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Hello World'],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_valid_string_array_param_hide_type_existing_node(self):
        with self.launch_node_command(
                arguments=['get', '/param_node', 'str_array', '--hide-type']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=["['Hello', 'World']"],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_valid_int_array_param_hide_type_existing_node(self):
        with self.launch_node_command(
                arguments=['get', '/param_node', 'int_array', '--hide-type']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=["array('q', [1, 2, 3])"],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_valid_double_array_param_hide_type_existing_node(self):
        with self.launch_node_command(
                arguments=['get', '/param_node', 'double_array', '--hide-type']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=["array('d', [1.0, 2.0, 3.0])"],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_valid_boolean_array_param_hide_type_existing_node(self):
        with self.launch_node_command(
                arguments=['get', '/param_node', 'bool_array', '--hide-type']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['[True, False, True]'],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_valid_byte_array_param_hide_type_existing_node(self):
        with self.launch_node_command(
                arguments=['get', '/param_node', 'byte_array', '--hide-type']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=["[b'p', b'v']"],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_inexisting_node(self):
        with self.launch_node_command(
                arguments=['get', '/foo/unexisting_node', 'test_param']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Node not found'],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_hidden_node_with_no_hidden_argument(self):
        with self.launch_node_command(
                arguments=['get', '/_hidden_param_node', 'test_param']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Node not found'],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_hidden_node_with_hidden_argument(self):
        with self.launch_node_command(
                arguments=[
                    'get', '/_hidden_param_node', 'int_param',
                    '--include-hidden-nodes']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Integer value is: 42'],
            text=node_command.output,
            strict=True
        )

    # List verb tests
    @launch_testing.markers.retry_on_failure(times=3)
    def test_list_existing_node(self):
        with self.launch_node_command(
                arguments=['list', '/param_node']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_text=PARAM_NODE_PARAMETER_LIST,
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_list_unexisting_node(self):
        with self.launch_node_command(
                arguments=['list', '/unexisting_node']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Node not found'],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_list_hidden_existing_node_no_hidden_flag(self):
        with self.launch_node_command(
                arguments=['list', '/_hidden_param_node']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Node not found'],
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_list_hidden_existing_node_hidden_flag(self):
        with self.launch_node_command(
                arguments=[
                    'list', '/_hidden_param_node', '--include-hidden-nodes']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_text=PARAM_NODE_PARAMETER_LIST,
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_list_existing_node_bool_prefix(self):
        with self.launch_node_command(
                arguments=[
                    'list', '--param-prefixes=bool', '/param_node']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_text=PARAM_NODE_BOOL_PARAMETER_LIST,
            text=node_command.output,
            strict=True
        )

    # TODO(BMarchi): This test fails for opensplice only. It hangs
    # in wait_for_shutdown method. It seems that it doesn't return
    # after getting all the nodes with their parameters.
    # @launch_testing.markers.retry_on_failure(times=3)
    # def test_list_all_nodes_all_parameters(self):
    #    with self.launch_node_command(
    #            arguments=['list', '--include-hidden-nodes']) as node_command:
    #        assert node_command.wait_for_shutdown(timeout=5)
    #    assert node_command.exit_code == launch_testing.asserts.EXIT_OK
    #    assert launch_testing.tools.expect_output(
    #        expected_text=ALL_NODES_PARAMETER_LIST,
    #        text=node_command.output,
    #        strict=True
    #    )

    # Describe verb tests
    @launch_testing.markers.retry_on_failure(times=3)
    def test_describe_existing_node_int_param(self):
        with self.launch_node_command(
                arguments=['describe', '/param_node', 'int_param']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_text=DESCRIBE_PARAMETER_TYPE.format('int_param', 'integer', ''),
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_describe_existing_node_double_param(self):
        with self.launch_node_command(
                arguments=['describe', '/param_node', 'double_param']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_text=DESCRIBE_PARAMETER_TYPE.format('double_param', 'double', ''),
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_describe_existing_node_bool_param(self):
        with self.launch_node_command(
                arguments=['describe', '/param_node', 'bool_param']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_text=DESCRIBE_PARAMETER_TYPE.format('bool_param', 'boolean', ''),
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_describe_existing_node_str_param(self):
        with self.launch_node_command(
                arguments=['describe', '/param_node', 'str_param']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_text=DESCRIBE_PARAMETER_TYPE.format('str_param', 'string', ''),
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_describe_existing_node_int_array_param(self):
        with self.launch_node_command(
                arguments=['describe', '/param_node', 'int_array']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_text=DESCRIBE_PARAMETER_TYPE.format('int_array', 'integer array', ''),
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_describe_existing_node_double_array_param(self):
        with self.launch_node_command(
                arguments=['describe', '/param_node', 'double_array']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_text=DESCRIBE_PARAMETER_TYPE.format('double_array', 'double array', ''),
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_describe_existing_node_bool_array_param(self):
        with self.launch_node_command(
                arguments=['describe', '/param_node', 'bool_array']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_text=DESCRIBE_PARAMETER_TYPE.format('bool_array', 'boolean array', ''),
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_describe_existing_node_string_array_param(self):
        with self.launch_node_command(
                arguments=['describe', '/param_node', 'str_array']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_text=DESCRIBE_PARAMETER_TYPE.format('str_array', 'string array', ''),
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_describe_existing_node_byte_array_param(self):
        with self.launch_node_command(
                arguments=['describe', '/param_node', 'byte_array']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_text=DESCRIBE_PARAMETER_TYPE.format('byte_array', 'byte array', ''),
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_describe_existing_node_not_set_param(self):
        with self.launch_node_command(
                arguments=['describe', '/param_node', 'parameter_with_no_value']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_text=DESCRIBE_PARAMETER_TYPE.format('parameter_with_no_value', 'not set', ''),
            text=node_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_describe_unexisting_node(self):
        with self.launch_node_command(
                arguments=['describe', '/foo/unexisting_node', 'int_param']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Node not found'],
            text=node_command.output,
            strict=True
        )
