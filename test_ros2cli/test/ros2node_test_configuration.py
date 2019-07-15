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

from launch_testing_ros.tools import basic_output_filter

sys.path.append(os.path.dirname(__file__))

from cli_test_configuration import CLITestConfiguration  # noqa
from fixture_actions import get_talker_node_action  # noqa
from fixture_actions import get_listener_node_action  # noqa


def get_test_configurations(rmw_implementation):
    return [
        CLITestConfiguration(
            command='node',
            arguments=['list'],
            fixture_actions=[
                get_talker_node_action(),
                get_listener_node_action(),
                get_listener_node_action(node_name='_my_hidden_listener'),
            ],
            expected_output=[
                '/my_ns/my_listener',
                '/my_ns/my_talker'
            ],
            output_filter=basic_output_filter(
                filtered_patterns=['.*launch_ros.*']
            )
        ),
        CLITestConfiguration(
            command='node',
            arguments=['list', '-a'],
            fixture_actions=[
                get_talker_node_action(),
                get_listener_node_action(),
                get_listener_node_action(node_name='_my_hidden_listener'),
            ],
            expected_output=[
                '/my_ns/_my_hidden_listener',
                '/my_ns/my_listener',
                '/my_ns/my_talker',
            ],
            output_filter=basic_output_filter(
                filtered_patterns=['.*launch_ros.*', '.*ros2cli.*']
            )
        ),
        CLITestConfiguration(
            command='node',
            arguments=['list', '-c'],
            fixture_actions=[
                get_talker_node_action(),
                get_listener_node_action(),
                get_listener_node_action(node_name='_my_hidden_listener'),
            ],
            # Fixture nodes plus launch_ros node.
            expected_output=[lambda line: int(line) == 3],
        ),
        CLITestConfiguration(
            command='node',
            arguments=['list', '-c', '-a'],
            fixture_actions=[
                get_talker_node_action(),
                get_listener_node_action(),
                get_listener_node_action(node_name='_my_hidden_listener'),
            ],
            # Fixture nodes plus launch_ros and CLI daemon nodes.
            expected_output=[lambda line: int(line) > 3],
        ),
        CLITestConfiguration(
            command='node',
            arguments=['info', '/my_ns/my_talker'],
            fixture_actions=[get_talker_node_action()],
            expected_output=itertools.chain([
                '/my_ns/my_talker',
                '  Subscribers:',
                '',
                '  Publishers:',
                '    /my_ns/chatter: std_msgs/msg/String',
                '    /my_ns/parameter_events: rcl_interfaces/msg/ParameterEvent',
                '    /my_ns/rosout: rcl_interfaces/msg/Log',
                '  Services:',
            ], itertools.repeat(re.compile(
                r'\s*/my_ns/my_talker/.*parameter.*: rcl_interfaces/srv/.*Parameter.*'
            ), 6)),
            output_filter=basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            )
        )
    ]
