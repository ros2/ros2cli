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
import functools
import math
import os
import re
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

from rclpy.utilities import get_available_rmw_implementations


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation, ready_fn):
    path_to_fixtures = os.path.join(os.path.dirname(__file__), 'fixtures')
    additional_env = {
        'RMW_IMPLEMENTATION': rmw_implementation, 'PYTHONUNBUFFERED': '1'
    }

    path_to_talker_node_script = os.path.join(path_to_fixtures, 'talker_node.py')
    path_to_listener_node_script = os.path.join(path_to_fixtures, 'listener_node.py')

    hidden_talker_node_action = Node(
        node_executable=sys.executable,
        arguments=[path_to_talker_node_script],
        remappings=[('chatter', '_hidden_chatter')],
        additional_env=additional_env
    )
    talker_node_action = Node(
        node_executable=sys.executable,
        arguments=[path_to_talker_node_script],
        additional_env=additional_env
    )
    listener_node_action = Node(
        node_executable=sys.executable,
        arguments=[path_to_listener_node_script],
        remappings=[('chatter', 'chit_chatter')],
        additional_env=additional_env
    )

    path_to_repeater_node_script = os.path.join(path_to_fixtures, 'repeater_node.py')

    array_repeater_node_action = Node(
        node_executable=sys.executable,
        arguments=[path_to_repeater_node_script, 'test_msgs/msg/Arrays'],
        node_name='array_repeater',
        remappings=[('/array_repeater/output', '/arrays')],
        output='screen',
        additional_env=additional_env
    )
    defaults_repeater_node_action = Node(
        node_executable=sys.executable,
        arguments=[path_to_repeater_node_script, 'test_msgs/msg/Defaults'],
        node_name='defaults_repeater',
        remappings=[('/defaults_repeater/output', '/defaults')],
        additional_env=additional_env,
    )
    bounded_sequences_repeater_node_action = Node(
        node_executable=sys.executable,
        arguments=[
            path_to_repeater_node_script, 'test_msgs/msg/BoundedSequences'
        ],
        node_name='bounded_sequences_repeater',
        remappings=[('/bounded_sequences_repeater/output', '/bounded_sequences')],
        additional_env=additional_env
    )
    unbounded_sequences_repeater_node_action = Node(
        node_executable=sys.executable,
        arguments=[
            path_to_repeater_node_script, 'test_msgs/msg/UnboundedSequences'
        ],
        node_name='unbounded_sequences_repeater',
        remappings=[('/unbounded_sequences_repeater/output', '/unbounded_sequences')],
        additional_env=additional_env
    )

    path_to_controller_node_script = os.path.join(path_to_fixtures, 'controller_node.py')

    cmd_vel_controller_node_action = Node(
        node_executable=sys.executable,
        arguments=[path_to_controller_node_script],
        additional_env=additional_env
    )

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
                        # Add talker/listener pair.
                        talker_node_action,
                        listener_node_action,
                        # Add hidden talker.
                        hidden_talker_node_action,
                        # Add topic repeaters.
                        array_repeater_node_action,
                        defaults_repeater_node_action,
                        bounded_sequences_repeater_node_action,
                        unbounded_sequences_repeater_node_action,
                        # Add stamped data publisher.
                        cmd_vel_controller_node_action,
                        OpaqueFunction(function=lambda context: ready_fn())
                    ],
                    additional_env=additional_env
                )
            ]
        ),
    ]), locals()


class TestROS2TopicCLI(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output,
        rmw_implementation,
        listener_node_action
    ):
        rmw_implementation_filter = launch_testing_ros.tools.basic_output_filter(
            filtered_patterns=['WARNING: topic .* does not appear to be published yet'],
            filtered_rmw_implementation=rmw_implementation
        )

        @contextlib.contextmanager
        def launch_topic_command(self, arguments):
            topic_command_action = ExecuteProcess(
                cmd=['ros2', 'topic', *arguments],
                additional_env={
                    'RMW_IMPLEMENTATION': rmw_implementation,
                    'PYTHONUNBUFFERED': '1'
                },
                name='ros2topic-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, topic_command_action, proc_info, proc_output,
                output_filter=rmw_implementation_filter
            ) as topic_command:
                yield topic_command
        cls.launch_topic_command = launch_topic_command

        cls.listener_node = launch_testing.tools.ProcessProxy(
            listener_node_action, proc_info, proc_output,
            output_filter=rmw_implementation_filter
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_list_topics(self):
        with self.launch_topic_command(arguments=['list']) as topic_command:
            assert topic_command.wait_for_shutdown(timeout=10)
        assert topic_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                '/arrays',
                '/bounded_sequences',
                '/chatter',
                '/chit_chatter',
                '/cmd_vel',
                '/defaults',
                '/parameter_events',
                '/rosout',
                '/unbounded_sequences',
            ],
            text=topic_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_list_all_topics(self):
        with self.launch_topic_command(
            arguments=['list', '--include-hidden-topics']
        ) as topic_command:
            assert topic_command.wait_for_shutdown(timeout=10)
        assert topic_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                '/_hidden_chatter',
                '/arrays',
                '/bounded_sequences',
                '/chatter',
                '/chit_chatter',
                '/cmd_vel',
                '/defaults',
                '/parameter_events',
                '/rosout',
                '/unbounded_sequences',
            ],
            text=topic_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_list_with_types(self):
        with self.launch_topic_command(arguments=['list', '-t']) as topic_command:
            assert topic_command.wait_for_shutdown(timeout=10)
        assert topic_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                '/arrays [test_msgs/msg/Arrays]',
                '/bounded_sequences [test_msgs/msg/BoundedSequences]',
                '/chatter [std_msgs/msg/String]',
                '/chit_chatter [std_msgs/msg/String]',
                '/cmd_vel [geometry_msgs/msg/TwistStamped]',
                '/defaults [test_msgs/msg/Defaults]',
                '/parameter_events [rcl_interfaces/msg/ParameterEvent]',
                '/rosout [rcl_interfaces/msg/Log]',
                '/unbounded_sequences [test_msgs/msg/UnboundedSequences]',
            ],
            text=topic_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_list_count(self):
        with self.launch_topic_command(arguments=['list', '-c']) as topic_command:
            assert topic_command.wait_for_shutdown(timeout=10)
        assert topic_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = topic_command.output.splitlines()
        assert len(output_lines) == 1
        assert int(output_lines[0]) == 9

    @launch_testing.markers.retry_on_failure(times=5)
    def test_topic_info(self):
        with self.launch_topic_command(arguments=['info', '/chatter']) as topic_command:
            assert topic_command.wait_for_shutdown(timeout=10)
        assert topic_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                'Type: std_msgs/msg/String',
                'Publisher count: 1',
                'Subscriber count: 0'
            ],
            text=topic_command.output,
            strict=True
        )

    def test_info_on_unknown_topic(self):
        with self.launch_topic_command(arguments=['info', '/unknown_topic']) as topic_command:
            assert topic_command.wait_for_shutdown(timeout=10)
        assert launch_testing.tools.expect_output(
            expected_lines=[
                "Unknown topic '/unknown_topic'",
            ],
            text=topic_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_topic_type(self):
        with self.launch_topic_command(arguments=['type', '/chatter']) as topic_command:
            assert topic_command.wait_for_shutdown(timeout=10)
        assert topic_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['std_msgs/msg/String'],
            text=topic_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_hidden_topic_type(self):
        with self.launch_topic_command(
            arguments=['type', '/_hidden_chatter']
        ) as topic_command:
            assert topic_command.wait_for_shutdown(timeout=10)
        assert topic_command.exit_code == 1
        assert topic_command.output == ''

    @launch_testing.markers.retry_on_failure(times=5)
    def test_find_topic_type(self):
        with self.launch_topic_command(
            arguments=['find', 'rcl_interfaces/msg/Log']
        ) as topic_command:
            assert topic_command.wait_for_shutdown(timeout=10)
        assert topic_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['/rosout'], text=topic_command.output, strict=True
        )

    def test_find_not_a_topic_typename(self):
        with self.launch_topic_command(
            arguments=['find', 'rcl_interfaces/msg/NotAMessageTypeName']
        ) as topic_command:
            assert topic_command.wait_for_shutdown(timeout=10)
        assert topic_command.exit_code == launch_testing.asserts.EXIT_OK
        assert not topic_command.output

    @launch_testing.markers.retry_on_failure(times=5)
    def test_topic_echo(self):
        with self.launch_topic_command(
            arguments=['echo', '/chatter']
        ) as topic_command:
            assert topic_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r"data: 'Hello World: \d+'"),
                    '---'
                ], strict=True
            ), timeout=10)
        assert topic_command.wait_for_shutdown(timeout=10)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_no_str_topic_echo(self):
        with self.launch_topic_command(
            arguments=['echo', '--no-str', '/chatter']
        ) as topic_command:
            assert topic_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r"data: '<string length: <\d+>>'"),
                    '---'
                ], strict=True
            ), timeout=10)
        assert topic_command.wait_for_shutdown(timeout=10)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_csv_topic_echo(self):
        with self.launch_topic_command(
            arguments=['echo', '--csv', '/defaults']
        ) as topic_command:
            assert topic_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    "True,b'2',100,1.125,1.125,-50,200,-1000,2000,-30000,60000,-40000000,50000000"
                ], strict=True
            ), timeout=10)
        assert topic_command.wait_for_shutdown(timeout=10)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_no_arr_topic_echo_on_array_message(self):
        with self.launch_topic_command(
            arguments=['echo', '--no-arr', '/arrays'],
        ) as topic_command:
            assert topic_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    "bool_values: '<array type: boolean[3]>'",
                    "byte_values: '<array type: octet[3]>'",
                    "char_values: '<array type: uint8[3]>'",
                    "float32_values: '<array type: float[3]>'",
                    "float64_values: '<array type: double[3]>'",
                    "int8_values: '<array type: int8[3]>'",
                    "uint8_values: '<array type: uint8[3]>'",
                    "int16_values: '<array type: int16[3]>'",
                    "uint16_values: '<array type: uint16[3]>'",
                    "int32_values: '<array type: int32[3]>'",
                    "uint32_values: '<array type: uint32[3]>'",
                    "int64_values: '<array type: int64[3]>'",
                    "uint64_values: '<array type: uint64[3]>'",
                    "string_values: '<array type: string[3]>'",
                    "basic_types_values: '<array type: test_msgs/msg/BasicTypes[3]>'",
                    "constants_values: '<array type: test_msgs/msg/Constants[3]>'",
                    "defaults_values: '<array type: test_msgs/msg/Defaults[3]>'",
                    "bool_values_default: '<array type: boolean[3]>'",
                    "byte_values_default: '<array type: octet[3]>'",
                    "char_values_default: '<array type: uint8[3]>'",
                    "float32_values_default: '<array type: float[3]>'",
                    "float64_values_default: '<array type: double[3]>'",
                    "int8_values_default: '<array type: int8[3]>'",
                    "uint8_values_default: '<array type: uint8[3]>'",
                    "int16_values_default: '<array type: int16[3]>'",
                    "uint16_values_default: '<array type: uint16[3]>'",
                    "int32_values_default: '<array type: int32[3]>'",
                    "uint32_values_default: '<array type: uint32[3]>'",
                    "int64_values_default: '<array type: int64[3]>'",
                    "uint64_values_default: '<array type: uint64[3]>'",
                    "string_values_default: '<array type: string[3]>'",
                    'alignment_check: 0',
                    '---'
                ], strict=False
            ), timeout=10), 'Output does not match: ' + topic_command.output
        assert topic_command.wait_for_shutdown(timeout=10)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_no_arr_topic_echo_on_seq_message(self):
        with self.launch_topic_command(
            arguments=['echo', '--no-arr', '/unbounded_sequences'],
        ) as topic_command:
            assert topic_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    "bool_values: '<sequence type: boolean, length: 0>'",
                    "byte_values: '<sequence type: octet, length: 0>'",
                    "char_values: '<sequence type: uint8, length: 0>'",
                    "float32_values: '<sequence type: float, length: 0>'",
                    "float64_values: '<sequence type: double, length: 0>'",
                    "int8_values: '<sequence type: int8, length: 0>'",
                    "uint8_values: '<sequence type: uint8, length: 0>'",
                    "int16_values: '<sequence type: int16, length: 0>'",
                    "uint16_values: '<sequence type: uint16, length: 0>'",
                    "int32_values: '<sequence type: int32, length: 0>'",
                    "uint32_values: '<sequence type: uint32, length: 0>'",
                    "int64_values: '<sequence type: int64, length: 0>'",
                    "uint64_values: '<sequence type: uint64, length: 0>'",
                    "string_values: '<sequence type: string, length: 0>'",
                    "basic_types_values: '<sequence type: test_msgs/msg/BasicTypes, length: 0>'",
                    "constants_values: '<sequence type: test_msgs/msg/Constants, length: 0>'",
                    "defaults_values: '<sequence type: test_msgs/msg/Defaults, length: 0>'",
                    "bool_values_default: '<sequence type: boolean, length: 3>'",
                    "byte_values_default: '<sequence type: octet, length: 3>'",
                    "char_values_default: '<sequence type: uint8, length: 3>'",
                    "float32_values_default: '<sequence type: float, length: 3>'",
                    "float64_values_default: '<sequence type: double, length: 3>'",
                    "int8_values_default: '<sequence type: int8, length: 3>'",
                    "uint8_values_default: '<sequence type: uint8, length: 3>'",
                    "int16_values_default: '<sequence type: int16, length: 3>'",
                    "uint16_values_default: '<sequence type: uint16, length: 3>'",
                    "int32_values_default: '<sequence type: int32, length: 3>'",
                    "uint32_values_default: '<sequence type: uint32, length: 3>'",
                    "int64_values_default: '<sequence type: int64, length: 3>'",
                    "uint64_values_default: '<sequence type: uint64, length: 3>'",
                    "string_values_default: '<sequence type: string, length: 3>'",
                    'alignment_check: 0',
                    '---'
                ], strict=True
            ), timeout=10)
        assert topic_command.wait_for_shutdown(timeout=10)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_no_arr_topic_echo_on_bounded_seq_message(self):
        with self.launch_topic_command(
            arguments=['echo', '--no-arr', '/bounded_sequences'],
        ) as topic_command:
            assert topic_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    "bool_values: '<sequence type: boolean[3], length: 0>'",
                    "byte_values: '<sequence type: octet[3], length: 0>'",
                    "char_values: '<sequence type: uint8[3], length: 0>'",
                    "float32_values: '<sequence type: float[3], length: 0>'",
                    "float64_values: '<sequence type: double[3], length: 0>'",
                    "int8_values: '<sequence type: int8[3], length: 0>'",
                    "uint8_values: '<sequence type: uint8[3], length: 0>'",
                    "int16_values: '<sequence type: int16[3], length: 0>'",
                    "uint16_values: '<sequence type: uint16[3], length: 0>'",
                    "int32_values: '<sequence type: int32[3], length: 0>'",
                    "uint32_values: '<sequence type: uint32[3], length: 0>'",
                    "int64_values: '<sequence type: int64[3], length: 0>'",
                    "uint64_values: '<sequence type: uint64[3], length: 0>'",
                    "string_values: '<sequence type: string[3], length: 0>'",
                    'basic_types_values: '
                    "'<sequence type: test_msgs/msg/BasicTypes[3], length: 0>'",
                    "constants_values: '<sequence type: test_msgs/msg/Constants[3], length: 0>'",
                    "defaults_values: '<sequence type: test_msgs/msg/Defaults[3], length: 0>'",
                    "bool_values_default: '<sequence type: boolean[3], length: 3>'",
                    "byte_values_default: '<sequence type: octet[3], length: 3>'",
                    "char_values_default: '<sequence type: uint8[3], length: 3>'",
                    "float32_values_default: '<sequence type: float[3], length: 3>'",
                    "float64_values_default: '<sequence type: double[3], length: 3>'",
                    "int8_values_default: '<sequence type: int8[3], length: 3>'",
                    "uint8_values_default: '<sequence type: uint8[3], length: 3>'",
                    "int16_values_default: '<sequence type: int16[3], length: 3>'",
                    "uint16_values_default: '<sequence type: uint16[3], length: 3>'",
                    "int32_values_default: '<sequence type: int32[3], length: 3>'",
                    "uint32_values_default: '<sequence type: uint32[3], length: 3>'",
                    "int64_values_default: '<sequence type: int64[3], length: 3>'",
                    "uint64_values_default: '<sequence type: uint64[3], length: 3>'",
                    "string_values_default: '<sequence type: string[3], length: 3>'",
                    'alignment_check: 0',
                    '---'
                ], strict=True
            ), timeout=10)
        assert topic_command.wait_for_shutdown(timeout=10)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_truncate_length_topic_echo(self):
        with self.launch_topic_command(
            arguments=['echo', '--truncate-length', '5', '/chatter'],
        ) as topic_command:
            assert topic_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'data: Hello...'),
                    '---'
                ], strict=True
            ), timeout=10)
        assert topic_command.wait_for_shutdown(timeout=10)

    def test_topic_pub(self):
        with self.launch_topic_command(
            arguments=['pub', '/chit_chatter', 'std_msgs/msg/String', '{data: foo}'],
        ) as topic_command:
            assert topic_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    'publisher: beginning loop',
                    "publishing #1: std_msgs.msg.String(data='foo')",
                    ''
                ], strict=True
            ), timeout=10)
            assert self.listener_node.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'\[INFO\] \[\d+.\d*\] \[listener\]: I heard: \[foo\]')
                ] * 3, strict=False
            ), timeout=10)
        assert topic_command.wait_for_shutdown(timeout=10)

    def test_topic_pub_once(self):
        with self.launch_topic_command(
            arguments=[
                'pub', '--once',
                '/chit_chatter',
                'std_msgs/msg/String',
                '{data: bar}'
            ]
        ) as topic_command:
            assert topic_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    'publisher: beginning loop',
                    "publishing #1: std_msgs.msg.String(data='bar')",
                    ''
                ], strict=True
            ), timeout=10)
            assert topic_command.wait_for_shutdown(timeout=10)
            assert self.listener_node.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'\[INFO\] \[\d+.\d*\] \[listener\]: I heard: \[bar\]')
                ], strict=False
            ), timeout=10)
        assert topic_command.exit_code == launch_testing.asserts.EXIT_OK

    def test_topic_pub_print_every_two(self):
        with self.launch_topic_command(
            arguments=[
                'pub',
                '-p', '2',
                '/chit_chatter',
                'std_msgs/msg/String',
                '{data: fizz}'
            ]
        ) as topic_command:
            assert topic_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    'publisher: beginning loop',
                    "publishing #2: std_msgs.msg.String(data='fizz')",
                    '',
                    "publishing #4: std_msgs.msg.String(data='fizz')",
                    ''
                ], strict=True
            ), timeout=10), 'Output does not match: ' + topic_command.output
            assert self.listener_node.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'\[INFO\] \[\d+.\d*\] \[listener\]: I heard: \[fizz\]')
                ], strict=False
            ), timeout=10)
        assert topic_command.wait_for_shutdown(timeout=10)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_topic_delay(self):
        average_delay_line_pattern = re.compile(r'average delay: (\d+.\d{3})')
        stats_line_pattern = re.compile(
            r'\s*min: \d+.\d{3}s max: \d+.\d{3}s std dev: \d+.\d{5}s window: \d+'
        )
        with self.launch_topic_command(arguments=['delay', '/cmd_vel']) as topic_command:
            assert topic_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    average_delay_line_pattern, stats_line_pattern
                ], strict=True
            ), timeout=10)
        assert topic_command.wait_for_shutdown(timeout=10)

        head_line = topic_command.output.splitlines()[0]
        average_delay = float(average_delay_line_pattern.match(head_line).group(1))
        assert math.isclose(average_delay, 0.0, abs_tol=10e-3)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_topic_hz(self):
        average_rate_line_pattern = re.compile(r'average rate: (\d+.\d{3})')
        stats_line_pattern = re.compile(
            r'\s*min: \d+.\d{3}s max: \d+.\d{3}s std dev: \d+.\d{5}s window: \d+'
        )
        with self.launch_topic_command(arguments=['hz', '/chatter']) as topic_command:
            assert topic_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    average_rate_line_pattern, stats_line_pattern
                ], strict=True
            ), timeout=10)
        assert topic_command.wait_for_shutdown(timeout=10)

        head_line = topic_command.output.splitlines()[0]
        average_rate = float(average_rate_line_pattern.match(head_line).group(1))
        assert math.isclose(average_rate, 1., rel_tol=1e-3)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_filtered_topic_hz(self):
        average_rate_line_pattern = re.compile(r'average rate: (\d+.\d{3})')
        stats_line_pattern = re.compile(
            r'\s*min: \d+.\d{3}s max: \d+.\d{3}s std dev: \d+.\d{5}s window: \d+'
        )
        with self.launch_topic_command(
            arguments=[
                'hz',
                '--filter',
                'int(m.data.rpartition(\":\")[-1]) % 2 == 0',
                '/chatter'
            ]
        ) as topic_command:
            assert topic_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    average_rate_line_pattern, stats_line_pattern
                ], strict=True
            ), timeout=10), 'Output does not match: ' + topic_command.output
        assert topic_command.wait_for_shutdown(timeout=10)

        head_line = topic_command.output.splitlines()[0]
        average_rate = float(average_rate_line_pattern.match(head_line).group(1))
        assert math.isclose(average_rate, 0.5, rel_tol=1e-3)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_topic_bw(self):
        with self.launch_topic_command(arguments=['bw', '/defaults']) as topic_command:
            assert topic_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    'Subscribed to [/defaults]',
                    re.compile(r'average: \d{2}\.\d{2}B/s'),
                    re.compile(
                        r'\s*mean: \d{2}\.\d{2}B/s min: \d{2}\.\d{2}B/s'
                        r' max: \d{2}\.\d{2}B/s window: \d+'
                    )
                ], strict=True
            ), timeout=10)
        assert topic_command.wait_for_shutdown(timeout=10)
