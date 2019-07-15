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
from ros2msg_test_configuration import some_messages_from_std_msgs  # noqa
from ros2srv_test_configuration import some_services_from_std_srvs  # noqa
from ros2action_test_configuration import some_actions_from_action_tutorials  # noqa


some_interfaces = (some_messages_from_std_msgs +
                   some_services_from_std_srvs +
                   some_actions_from_action_tutorials)


def get_test_configurations(*args, **kwargs):
    return [
        CLITestConfiguration(
            command='interface',
            arguments=['list', '-m'],
            expected_output=itertools.chain(
                ['Messages:'], itertools.repeat(re.compile(r'.*/msg/.*'))
            )
        ),
        CLITestConfiguration(
            command='interface',
            arguments=['list', '-s'],
            expected_output=itertools.chain(
                ['Services:'], itertools.repeat(re.compile(r'.*/srv/.*'))
            )
        ),
        CLITestConfiguration(
            command='interface',
            arguments=['list', '-a'],
            expected_output=itertools.chain(
                ['Actions:'], itertools.repeat(re.compile(r'.*/action/.*'))
            )
        ),
        CLITestConfiguration(
            command='interface',
            arguments=['list'],
            expected_output=(
                lambda lines: all(
                    any(interface in line for line in lines)
                    for interface in some_interfaces
                )
            ),
        ),
        CLITestConfiguration(
            command='interface',
            arguments=['package', 'not_a_package'],
            expected_output=['Unknown package not_a_package'],
            exit_codes=[1]
        ),
        CLITestConfiguration(
            command='interface',
            arguments=['package', 'std_msgs'],
            expected_output=(
                lambda lines: (
                    all(msg in lines for msg in some_messages_from_std_msgs) and
                    all(re.match(r'.*/msg/.*', line) is not None for line in lines)
                )
            )
        ),
        CLITestConfiguration(
            command='interface',
            arguments=['package', 'action_tutorials'],
            expected_output=(
                lambda lines: (
                    all(msg in lines for msg in some_actions_from_action_tutorials) and
                    all(re.match(r'.*/action/.*', line) is not None for line in lines)
                )
            )
        ),
        CLITestConfiguration(
            command='interface',
            arguments=['package', 'rcl_interfaces'],
            expected_output=(
                lambda lines: (
                    any(re.match(r'.*/msg/.*', line) is not None for line in lines) and
                    any(re.match(r'.*/srv/.*', line) is not None for line in lines)
                )
            )
        ),
        CLITestConfiguration(
            command='interface',
            arguments=['packages', '-m'],
            expected_output=(
                lambda lines: ('std_msgs' in lines and
                               'std_srvs' not in lines and
                               'action_tutorials' not in lines)
            )
        ),
        CLITestConfiguration(
            command='interface',
            arguments=['packages', '-s'],
            expected_output=(
                lambda lines: ('std_msgs' not in lines and
                               'std_srvs' in lines and
                               'action_tutorials' not in lines)
            )
        ),
        CLITestConfiguration(
            command='interface',
            arguments=['packages', '-a'],
            expected_output=(
                lambda lines: ('std_msgs' not in lines and
                               'std_srvs' not in lines and
                               'action_tutorials' in lines)
            )
        ),
        CLITestConfiguration(
            command='interface',
            arguments=['packages'],
            expected_output=(
                lambda lines: ('std_msgs' in lines and
                               'std_srvs' in lines and
                               'action_tutorials' in lines)
            )
        ),
        CLITestConfiguration(
            command='interface',
            arguments=['show', 'std_msgs/msg/String'],
            expected_output=[
                '',
                '',
                'module std_msgs {',
                '  module msg {',
                '    struct String {',
                '      string data;',
                '    };',
                '  };',
                '};'
            ],
            output_filter=basic_output_filter(
                filtered_prefixes=['//']
            ),
        ),
    ]
