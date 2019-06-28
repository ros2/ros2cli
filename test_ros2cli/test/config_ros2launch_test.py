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

from launch_ros.substitutions import FindPackage

sys.path.append(os.path.dirname(__file__))

from test_config import TestConfig  # noqa


config = TestConfig()
config.verb = 'launch'

config.options = [
    'launch',
    'launch -d',
    'launch_absolute_path',
]

config.arguments_by_option = {
    'launch': ['demo_nodes_cpp', 'add_two_ints.launch.py'],
    'launch -d': ['-d', 'demo_nodes_cpp', 'add_two_ints.launch.py'],
    'launch_absolute_path': [[
        FindPackage('demo_nodes_cpp'),
        '/share/demo_nodes_cpp/launch/services/add_two_ints.launch.py',
    ]],
}

config.actions_by_option = {
    'launch': [],
    'launch -d': [],
    'launch_absolute_path': [],
}

common_output = [
    'Incoming request',
    'a: 2 b: 3',
    'Result of add_two_ints: 5',
    'process has finished cleanly',
    '[INFO]',
]

config.msgs_by_option = {
    'launch': common_output,
    'launch -d': common_output + ['[DEBUG]'],
    'launch_absolute_path': common_output,
}
