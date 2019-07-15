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

import os
import re
import sys

from launch_testing_ros.tools import basic_output_filter

import yaml

sys.path.append(os.path.dirname(__file__))

from cli_test_configuration import CLITestConfiguration  # noqa
from fixture_actions import get_fibonacci_action_server_node_action  # noqa


some_actions_from_action_tutorials = [
    'action_tutorials/action/Fibonacci'
]


def get_fibonacci_send_goal_output(*, order=1, with_feedback=False):
    assert order > 0
    output = [
        'Waiting for an action server to become available...',
        'Sending goal:',
        '     order: {}'.format(order),
        '',
        re.compile('Goal accepted with ID: [a-f0-9]+'),
        '',
    ]
    sequence = [0, 1]
    for _ in range(order - 1):
        sequence.append(sequence[-1] + sequence[-2])
        if with_feedback:
            output.append('Feedback:')
            output.extend(('    ' + yaml.dump({
                'partial_sequence': sequence
            })).splitlines())
            output.append('')
    output.append('Result:'),
    output.extend(('    ' + yaml.dump({
        'sequence': sequence
    })).splitlines())
    output.append('')
    output.append('Goal finished with status: SUCCEEDED')
    return output


def get_fibonacci_info_output(*, server_name='/fibonacci_action_server',
                              count_only=False, include_types=False):
    output = [
        'Action: /fibonacci',
        'Action clients: 0',
        'Action servers: 1',
    ]
    if not count_only:
        server_info = server_name
        if include_types:
            server_info += ' [action_tutorials/action/Fibonacci]'
        output.append(server_info)
    return output


def get_test_configurations(rmw_implementation):
    return [
        CLITestConfiguration(
            command='action',
            arguments=['info', '/not_an_action'],
            expected_output=[
                'Action: /not_an_action',
                'Action clients: 0',
                'Action servers: 0',
            ],
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            )
        ),
        CLITestConfiguration(
            command='action',
            arguments=['info', '/fibonacci'],
            fixture_actions=[get_fibonacci_action_server_node_action()],
            expected_output=get_fibonacci_info_output(),
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            )
        ),
        CLITestConfiguration(
            command='action',
            arguments=['info', '-t', '/fibonacci'],
            fixture_actions=[get_fibonacci_action_server_node_action()],
            expected_output=get_fibonacci_info_output(include_types=True),
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            )
        ),
        CLITestConfiguration(
            command='action',
            arguments=['info', '-c', '/fibonacci'],
            fixture_actions=[get_fibonacci_action_server_node_action()],
            expected_output=get_fibonacci_info_output(count_only=True),
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            )
        ),
        CLITestConfiguration(
            command='action',
            arguments=['list'],
            fixture_actions=[get_fibonacci_action_server_node_action()],
            expected_output=['/fibonacci'],
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            )
        ),
        CLITestConfiguration(
            command='action',
            arguments=['list', '-t'],
            fixture_actions=[get_fibonacci_action_server_node_action()],
            expected_output=['/fibonacci [action_tutorials/action/Fibonacci]'],
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            )
        ),
        CLITestConfiguration(
            command='action',
            arguments=['list', '-c'],
            fixture_actions=[get_fibonacci_action_server_node_action()],
            expected_output=[lambda line: int(line) == 1],
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            )
        ),
        CLITestConfiguration(
            command='action',
            arguments=[
                'send_goal',
                '/fibonacci',
                'action_tutorials/action/Fibonacci',
                '{order: 5}'
            ],
            fixture_actions=[get_fibonacci_action_server_node_action()],
            expected_output=get_fibonacci_send_goal_output(order=5),
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            )
        ),
        CLITestConfiguration(
            command='action',
            arguments=[
                'send_goal',
                '-f',
                '/fibonacci',
                'action_tutorials/action/Fibonacci',
                '{order: 5}'
            ],
            fixture_actions=[get_fibonacci_action_server_node_action()],
            expected_output=get_fibonacci_send_goal_output(order=5, with_feedback=True),
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            )
        ),
        CLITestConfiguration(
            command='action',
            arguments=['show', 'action_tutorials/action/Fibonacci'],
            expected_output=[
                'int32 order',
                '---',
                'int32[] sequence',
                '---',
                'int32[] partial_sequence'
            ],
        ),
        CLITestConfiguration(
            command='action',
            arguments=['show', 'not_a_package/action/Fibonacci'],
            expected_output=['Unknown package name'],
            exit_codes=[1]
        ),
        CLITestConfiguration(
            command='action',
            arguments=['show', 'action_tutorials/action/NotAnActionType'],
            expected_output=['Unknown action type'],
            exit_codes=[1]
        ),
    ]
