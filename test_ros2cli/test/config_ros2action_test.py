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


config = TestConfig()

config.command = 'action'

config.options = [
    'info',
    'info -t',
    'info -c',
    'list',
    'list -t',
    'list -c',
    'send_goal',
    'send_goal -f',
    'show',
]

config.arguments_by_option = {
    'info': ['info', '/fibonacci'],
    'info -t': ['info', '-t', '/fibonacci'],
    'info -c': ['info', '-c', '/fibonacci'],
    'list': ['list'],
    'list -t': ['list', '-t'],
    'list -c': ['list', '-c'],
    'send_goal': ['send_goal', '/fibonacci', 'action_tutorials/action/Fibonacci', '{order: 5}'],
    'send_goal -f':
        ['send_goal', '-f', '/fibonacci', 'action_tutorials/action/Fibonacci', '{order: 5}'],
    'show': ['show', 'action_tutorials/action/Fibonacci'],
}


def get_action_server_node_action():
    return Node(
        package='action_tutorials',
        node_executable='fibonacci_action_server.py',
        sigterm_timeout=LaunchConfiguration('sigterm_timeout', default=30)
    )


config.actions_by_option = {
    'info': [get_action_server_node_action()],
    'info -t': [get_action_server_node_action()],
    'info -c': [get_action_server_node_action()],
    'list': [get_action_server_node_action()],
    'list -t': [get_action_server_node_action()],
    'list -c': [get_action_server_node_action()],
    'send_goal': [get_action_server_node_action()],
    'send_goal -f': [get_action_server_node_action()],
    'show': [],
}

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
    'sequence: [0, 1, 1, 2, 3, 5]',
    'Goal finished with status: SUCCEEDED',
]

config.msgs_by_option = {
    'info': common_info_output + ['/fibonacci_action_server'],
    'info -t':
        common_info_output +
        ['/fibonacci_action_server [action_tutorials/action/Fibonacci]'],
    'info -c': common_info_output,
    'list': ['/fibonacci'],
    'list -t': ['/fibonacci [action_tutorials/action/Fibonacci]'],
    'list -c': ['1'],
    'send_goal': common_send_goal_output,
    'send_goal -f': common_send_goal_output + [
        'Feedback:',
        'partial_sequence:',
        '[0, 1, 1]',
        '[0, 1, 1, 2]',
        '[0, 1, 1, 2, 3]',
        '[0, 1, 1, 2, 3, 5]'],
    'show': [
        'int32 order',
        '---',
        'int32[] sequence',
        '---',
        'int32[] partial_sequence'
    ],
}
