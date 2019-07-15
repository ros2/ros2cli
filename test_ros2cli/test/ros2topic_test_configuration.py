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

import itertools

import os
import re
import sys
import time

from launch_testing_ros.tools import basic_output_filter

sys.path.append(os.path.dirname(__file__))

from cli_test_configuration import CLITestConfiguration  # noqa
from fixture_actions import get_talker_node_action  # noqa
from fixture_actions import get_listener_node_action  # noqa
from fixture_actions import get_dummy_base_controller_node_action  # noqa
from fixture_actions import get_publisher_node_action  # noqa


def get_test_configurations(rmw_implementation):
    now = time.time()

    return [
        CLITestConfiguration(
            command='topic',
            arguments=['list'],
            fixture_actions=[
                get_talker_node_action(),
                get_talker_node_action(topic_name='_chatter')
            ],
            expected_output=[
                '/my_ns/chatter',
                '/my_ns/parameter_events',
                '/my_ns/rosout',
                '/parameter_events',
                '/rosout'
            ],
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            )
        ),
        CLITestConfiguration(
            command='topic',
            arguments=['list', '--include-hidden-topics'],
            fixture_actions=[
                get_talker_node_action(),
                get_talker_node_action(topic_name='_chatter')
            ],
            expected_output=[
                '/my_ns/_chatter',
                '/my_ns/chatter',
                '/my_ns/parameter_events',
                '/my_ns/rosout',
                '/parameter_events',
                '/rosout'
            ],
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            )
        ),
        CLITestConfiguration(
            command='topic',
            arguments=['list', '-t'],
            fixture_actions=[get_talker_node_action()],
            expected_output=[
                '/my_ns/chatter [std_msgs/msg/String]',
                '/my_ns/parameter_events [rcl_interfaces/msg/ParameterEvent]',
                '/my_ns/rosout [rcl_interfaces/msg/Log]',
                '/parameter_events [rcl_interfaces/msg/ParameterEvent]',
                '/rosout [rcl_interfaces/msg/Log]'
            ],
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            )
        ),
        CLITestConfiguration(
            command='topic',
            arguments=['list', '-c'],
            fixture_actions=[get_talker_node_action()],
            expected_output=[lambda line: int(line) == 5],
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            )
        ),
        CLITestConfiguration(
            command='topic',
            arguments=['info', '/my_ns/chatter'],
            fixture_actions=[get_talker_node_action()],
            expected_output=[
                'Topic: /my_ns/chatter',
                'Publisher count: 1',
                'Subscriber count: 0'
            ],
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            )
        ),
        CLITestConfiguration(
            command='topic',
            arguments=['info', '/some_chatter'],
            fixture_actions=[get_talker_node_action()],
            expected_output=[
                'Topic: /some_chatter',
                'Publisher count: 0',
                'Subscriber count: 0'
            ],
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            )
        ),
        CLITestConfiguration(
            command='topic',
            arguments=['type', '/my_ns/chatter'],
            fixture_actions=[get_talker_node_action()],
            expected_output=['std_msgs/msg/String'],
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            )
        ),
        CLITestConfiguration(
            command='topic',
            arguments=['type', '/my_ns/_chatter'],
            fixture_actions=[get_talker_node_action(topic_name='_chatter')],
            expected_output=None,
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            ),
            exit_codes=[1]
        ),
        CLITestConfiguration(
            command='topic',
            arguments=['find', 'rcl_interfaces/msg/Log'],
            fixture_actions=[get_talker_node_action()],
            expected_output=[
                '/my_ns/rosout',
                '/rosout'
            ]
        ),
        CLITestConfiguration(
            command='topic',
            arguments=['find', 'rcl_interfaces/msg/NotAMessageType'],
            fixture_actions=[get_talker_node_action()],
            expected_output=None,
        ),
        CLITestConfiguration(
            command='topic',
            arguments=['find', 'rcl_interfaces/msg/NotAMessageType'],
            fixture_actions=[get_talker_node_action()],
            expected_output=None,
        ),
        CLITestConfiguration(
            command='topic',
            arguments=['echo', '/my_ns/chatter'],
            fixture_actions=[get_talker_node_action()],
            expected_output=itertools.cycle([
                re.compile(r"data: 'Hello World: \d+'"),
                '---'
            ]),
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            ),
            self_terminates=False,
            exit_codes=[2],
            timeout=5.0
        ),
        CLITestConfiguration(
            command='topic',
            arguments=['echo', '--no-str', '/my_ns/chatter'],
            fixture_actions=[get_talker_node_action()],
            expected_output=itertools.cycle([
                re.compile(r"data: '<string length: <\d+>>'"),
                '---'
            ]),
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            ),
            self_terminates=False,
            exit_codes=[2],
            timeout=5.0
        ),
        CLITestConfiguration(
            command='topic',
            arguments=['echo', '--csv', '/my_ns/test_defaults'],
            fixture_actions=[get_publisher_node_action(
                topic_name='/my_ns/test_defaults',
                topic_type='test_msgs/msg/Defaults'
            )],
            expected_output=itertools.repeat(
                "True,b'2',100,1.125,1.125,-50,200,-1000,2000,-30000,60000,-40000000,50000000"
            ),
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            ),
            self_terminates=False,
            exit_codes=[2],
            timeout=5.0
        ),
        CLITestConfiguration(
            command='topic',
            arguments=['echo', '--no-arr', '/my_ns/test_arrays'],
            fixture_actions=[get_publisher_node_action(
                topic_name='/my_ns/test_arrays',
                topic_type='test_msgs/msg/Arrays'
            )],
            expected_output=itertools.cycle([
                "byte_values: '<array type: octet[3]>'",
                "float64_values: '<array type: double[3]>'",
                "int8_values: '<array type: int8[3]>'",
                "uint8_values: '<array type: uint8[3]>'",
                "int16_values: '<array type: int16[3]>'",
                "uint16_values: '<array type: uint16[3]>'",
                "int32_values: '<array type: int32[3]>'",
                "uint32_values: '<array type: uint32[3]>'",
                "int64_values: '<array type: int64[3]>'",
                "uint64_values: '<array type: uint64[3]>'",
                "string_values: '<array type: unknown[3]>'",
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
                "string_values_default: '<array type: unknown[3]>'",
                'alignment_check: 0',
                '---'
            ]),
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            ),
            self_terminates=False,
            exit_codes=[2],
            timeout=5.0
        ),
        CLITestConfiguration(
            command='topic',
            arguments=['echo', '--no-arr', '/my_ns/test_unbounded_seq'],
            fixture_actions=[get_publisher_node_action(
                topic_name='/my_ns/test_unbounded_seq',
                topic_type='test_msgs/msg/UnboundedSequence'
            )],
            expected_output=itertools.cycle([
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
            ]),
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            ),
            self_terminates=False,
            exit_codes=[2],
            timeout=5.0
        ),
        CLITestConfiguration(
            command='topic',
            arguments=['echo', '--no-arr', '/my_ns/test_bounded_seq'],
            fixture_actions=[get_publisher_node_action(
                topic_name='/my_ns/test_bounded_seq',
                topic_type='test_msgs/msg/BoundedSequence'
            )],
            expected_output=itertools.cycle([
                "int8_values: '<sequence type: int8[3], length: 0>'",
                "uint8_values: '<sequence type: uint8[3], length: 0>'",
                "int16_values: '<sequence type: int16[3], length: 0>'",
                "uint16_values: '<sequence type: uint16[3], length: 0>'",
                "int32_values: '<sequence type: int32[3], length: 0>'",
                "uint32_values: '<sequence type: uint32[3], length: 0>'",
                "int64_values: '<sequence type: int64[3], length: 0>'",
                "uint64_values: '<sequence type: uint64[3], length: 0>'",
                "string_values: '<sequence type: string[3], length: 0>'",
                "basic_types_values: '<sequence type: test_msgs/msg/BasicTypes[3], length: 0>'",
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
            ]),
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            ),
            self_terminates=False,
            exit_codes=[2],
            timeout=5.0
        ),
        CLITestConfiguration(
            command='topic',
            arguments=['echo', '--truncate-length', '5', '/my_ns/chatter'],
            fixture_actions=[get_talker_node_action()],
            expected_output=itertools.cycle([
                re.compile(r'data: Hello...'),
                '---'
            ]),
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            ),
            self_terminates=False,
            exit_codes=[2],
            timeout=5.0,
        ),
        CLITestConfiguration(
            command='topic',
            arguments=['pub', '/my_ns/chatter', 'std_msgs/msg/String', '{data: test}'],
            fixture_actions=[get_listener_node_action(node_name='my_listener')],
            expected_output={
                'cli': itertools.chain(
                    ['publisher: beginning loop'],
                    itertools.cycle([
                        re.compile(r"publishing #\d+: std_msgs\.msg\.String\(data='test'\)"),
                        ''
                    ])
                ),
                'my_listener': itertools.repeat('[INFO] [my_ns.my_listener]: I heard: [test]')
            },
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            ),
            self_terminates=False,
            exit_codes=[2],
            timeout=5.0
        ),
        CLITestConfiguration(
            command='topic',
            arguments=['pub', '--once', '/my_ns/chatter', 'std_msgs/msg/String', '{data: test}'],
            expected_output=[
                'publisher: beginning loop',
                re.compile(r"publishing #1: std_msgs\.msg\.String\(data='test'\)"),
                ''
            ],
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            )
        ),
        CLITestConfiguration(
            command='topic',
            arguments=[
                'pub',
                '-p', '2',
                '/my_ns/chatter',
                'std_msgs/msg/String',
                '{data: test}'
            ],
            expected_output=itertools.chain(
                ['publisher: beginning loop'],
                itertools.cycle([
                    lambda line: int(re.match(
                        "publishing #(\d+): std_msgs\.msg\.String\(data='test'\)", line
                    ).group(1)) % 2 == 0,
                    ''
                ])
            ),
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            ),
            self_terminates=False,
            exit_codes=[2],
            timeout=5.0
        ),
        CLITestConfiguration(
            command='topic',
            arguments=['delay', '/my_ns/cmd_vel'],
            fixture_actions=[get_dummy_base_controller_node_action(start_time=now)],
            expected_output=itertools.cycle([
                lambda line: (
                    re.match(r'average delay: \d+.\d{3}', line) is not None
                    and abs(float(line.split(':')[-1]) - (time.time() - now)) < 5.
                ),
                re.compile(
                    r'\s*min: \d+.\d{3}s max: \d+.\d{3}s std dev: \d+.\d{5}s window: \d+'
                )
            ]),
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            ),
            self_terminates=False,
            exit_codes=[2],
            timeout=5.0
        ),
        CLITestConfiguration(
            command='topic',
            arguments=['hz', '/my_ns/chatter'],
            fixture_actions=[get_talker_node_action()],
            expected_output=itertools.cycle([
                lambda line: (
                    re.match(r'average rate: \d\.\d{3}', line) is not None
                    and abs(float(line[-5:]) - 1.) < 0.01
                ),
                re.compile(r'\s*min: \d\.\d{3}s max: \d\.\d{3}s std dev: \d\.\d{5}s window: \d+')
            ]),
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            ),
            self_terminates=False,
            exit_codes=[2],
            timeout=5.0
        ),
        CLITestConfiguration(
            command='topic',
            arguments=[
                'hz',
                '--filter',
                'int(m.data.rpartition(\":\")[-1]) % 2 == 0',
                '/my_ns/chatter'
            ],
            fixture_actions=[get_talker_node_action()],
            expected_output=itertools.cycle([
                lambda line: (
                    re.match(r'average rate: \d\.\d{3}', line) is not None
                    and abs(float(line[-5:]) - 0.5) < 0.01
                ),
                re.compile(r'\s*min: \d\.\d{3}s max: \d\.\d{3}s std dev: \d\.\d{5}s window: \d+')
            ]),
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            ),
            self_terminates=False,
            exit_codes=[2],
            timeout=10.0
        ),
        CLITestConfiguration(
            command='topic',
            arguments=['bw', '/my_ns/chatter'],
            fixture_actions=[get_talker_node_action()],
            expected_output=itertools.chain(
                ['Subscribed to [/my_ns/chatter]'],
                itertools.cycle([
                    re.compile(r'average: 2\d\.\d{2}B/s'),
                    re.compile(
                        r'\s*mean: 2\d\.\d{2}B/s min: 2\d\.\d{2}B/s max: 2\d\.\d{2}B/s window: \d+'
                    )
                ])
            ),
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            ),
            self_terminates=False,
            exit_codes=[2],
            timeout=5.0
        ),
    ]
