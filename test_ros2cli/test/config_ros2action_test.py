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
import sys

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))

from test_config import TestConfig  # noqa


def get_action_server_node_action():
    return Node(
        package='action_tutorials_cpp',
        node_executable='fibonacci_action_server',
        sigterm_timeout=LaunchConfiguration('sigterm_timeout', default=30)
    )


common_info_output = [
    'Action: /fibonacci',
    'Action clients: 0',
    'Action servers: 1',
]

common_send_goal_output = [
    'Waiting for an action server to become available...',
    'Sending goal:',
    'order: 5',
    'Goal accepted with ID:',
    'Result:',
    'sequence:',
    '0',
    '1',
    '2',
    '3',
    '5',
    'Goal finished with status: SUCCEEDED',
]

configs = [
    TestConfig(
        command='action',
        arguments=['info', '/fibonacci'],
        actions=[get_action_server_node_action()],
        expected_output=common_info_output + ['/fibonacci_action_server'],
    ),
    TestConfig(
        command='action',
        arguments=['info', '-t', '/fibonacci'],
        actions=[get_action_server_node_action()],
        expected_output=common_info_output + [
            '/fibonacci_action_server [action_tutorials_interfaces/action/Fibonacci]'
        ],
    ),
    TestConfig(
        command='action',
        arguments=['info', '-c', '/fibonacci'],
        actions=[get_action_server_node_action()],
        expected_output=common_info_output,
    ),
    TestConfig(
        command='action',
        arguments=['list'],
        actions=[get_action_server_node_action()],
        expected_output=['/fibonacci'],
    ),
    TestConfig(
        command='action',
        arguments=['list', '-t'],
        actions=[get_action_server_node_action()],
        expected_output=['/fibonacci [action_tutorials_interfaces/action/Fibonacci]'],
    ),
    TestConfig(
        command='action',
        arguments=['list', '-c'],
        actions=[get_action_server_node_action()],
        expected_output=['1'],
    ),
    TestConfig(
        command='action',
        arguments=[
            'send_goal',
            '/fibonacci',
            'action_tutorials_interfaces/action/Fibonacci',
            '{order: 5}'
        ],
        actions=[get_action_server_node_action()],
        expected_output=common_send_goal_output,
    ),
    TestConfig(
        command='action',
        arguments=[
            'send_goal',
            '-f',
            '/fibonacci',
            'action_tutorials_interfaces/action/Fibonacci',
            '{order: 5}'
        ],
        actions=[get_action_server_node_action()],
        expected_output=common_send_goal_output + [
            'Feedback:',
            'partial_sequence:',
        ],
    ),
    TestConfig(
        command='action',
        arguments=['show', 'action_tutorials_interfaces/action/Fibonacci'],
        expected_output=[
            'int32 order',
            '---',
            'int32[] sequence',
            '---',
            'int32[] partial_sequence'
        ],
    ),
]
