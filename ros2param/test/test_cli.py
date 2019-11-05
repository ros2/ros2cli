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

import param_node

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

DESCRIBE_PARAMETER_NAME_TYPE = {
    'bool_param': 'boolean',
    'int_param': 'integer',
    'double_param': 'double',
    'str_param': 'string',
    'byte_array': 'byte array',
    'bool_array': 'boolean array',
    'int_array': 'integer array',
    'double_array': 'double array',
    'str_array': 'string array',
    'parameter_with_no_value': 'not set'
}


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation, ready_fn):
    path_to_param_node_script = os.path.join(
        os.path.dirname(__file__), 'param_node.py'
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
                    # ignore ros2cli daemon node
                    filtered_patterns=['.*ros2cli.*'],
                    filtered_rmw_implementation=rmw_implementation
                )
            ) as node_command:
                yield node_command
        cls.launch_node_command = launch_node_command
        cls.rmw_implementation = rmw_implementation

    def test_get_verb(self):
        parameters = param_node.ParamNode.get_node_parameters()
        for param_key, param_value in parameters.items():
            with self.launch_node_command(
                    arguments=['get', '/param_node', param_key]) as node_command:
                assert node_command.wait_for_shutdown(timeout=5)
            assert node_command.exit_code == launch_testing.asserts.EXIT_OK
            if param_value is None:
                param_value = ''
            assert launch_testing.tools.expect_output(
                expected_text=param_node.GET_VERB_PARAM_VALUES[param_key].format(str(param_value)),
                text=node_command.output,
                strict=True
            )

    def test_get_verb_hide_type(self):
        parameters = param_node.ParamNode.get_node_parameters()
        for param_key, param_value in parameters.items():
            with self.launch_node_command(
                    arguments=['get', '/param_node', param_key, '--hide-type']) as node_command:
                assert node_command.wait_for_shutdown(timeout=5)
            assert node_command.exit_code == launch_testing.asserts.EXIT_OK
            if param_value is None:
                param_value = ':None\n'
            assert launch_testing.tools.expect_output(
                expected_lines=[
                    param_node.GET_VERB_PARAM_VALUES[param_key].format(
                        str(param_value)).split(':')[1].strip()],
                text=node_command.output,
                strict=True
            )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_get_nonexistent_node(self):
        with self.launch_node_command(
                arguments=['get', '/foo/nonexistent_node', 'test_param']) as node_command:
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
            expected_text=param_node.GET_VERB_PARAM_VALUES['int_param'].format(str(42)),
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
    def test_list_nonexistent_node(self):
        with self.launch_node_command(
                arguments=['list', '/nonexistent_node']) as node_command:
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

    # TODO(BMarchi): This test fails for every implementation. It hangs
    # in wait_for_shutdown method. It seems that it doesn't return
    # after getting all the nodes with their parameters.
    # @launch_testing.markers.retry_on_failure(times=3)
    # def test_list_all_nodes_all_parameters(self):
    #    with self.launch_node_command(
    #            arguments=['list', '--include-hidden-nodes']) as node_command:
    #        assert node_command.wait_for_shutdown(timeout=10)
    #    assert node_command.exit_code == launch_testing.asserts.EXIT_OK
    #    assert launch_testing.tools.expect_output(
    #        expected_text=ALL_NODES_PARAMETER_LIST,
    #        text=node_command.output,
    #        strict=True
    #    )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_describe_existing_node_all_params(self):
        for param_key in param_node.ParamNode.get_node_parameters():
            with self.launch_node_command(
                    arguments=['describe', '/param_node', param_key]) as node_command:
                assert node_command.wait_for_shutdown(timeout=5)
            assert node_command.exit_code == launch_testing.asserts.EXIT_OK
            assert launch_testing.tools.expect_output(
                expected_text=DESCRIBE_PARAMETER_TYPE.format(
                    param_key, DESCRIBE_PARAMETER_NAME_TYPE[param_key], ''),
                text=node_command.output,
                strict=True
            )

    @launch_testing.markers.retry_on_failure(times=3)
    def test_describe_nonexistent_node(self):
        with self.launch_node_command(
                arguments=['describe', '/foo/nonexistent_node', 'int_param']) as node_command:
            assert node_command.wait_for_shutdown(timeout=5)
        assert node_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Node not found'],
            text=node_command.output,
            strict=True
        )
