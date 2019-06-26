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


class TestConfig():
    """A class for documenting how to configure `test_process_output_customizable.py.in`."""

    __slots__ = (
        # `ros2cli` command being tested
        'command',
        # A list of options for parametrizing the test.
        # Each element is used as a key for accessing the other dicts.
        # Usually, it's a string containing the verb and arguments being tested.
        'options',
        # A dict, where the keys are the options and
        # the values are a list specifing the verb and remaining arguments.
        # e.g.: for `ros2 topic list -t` the list should be `['list', '-t']`.
        'arguments_by_option',
        # A dict, where the keys are the options and the values are a list of actions.
        # The actions are launched before the `ros2cli` command is executed.
        'actions_by_option',
        # A dict, where the keys are the options and the values are a list of strings.
        # After the `ros2cli` command finishes, all the strings are checked to be in its
        # stdout.
        'msgs_by_option',
    )
