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

from typing import Iterable
from typing import Optional

from launch import Action
from launch import SomeSubstitutionsType
from launch.utilities import normalize_to_list_of_substitutions


class TestConfig():
    """A class for configuring the `test_process_output_customizable.py.in` test."""

    def __init__(
        self,
        *,
        command: str,
        arguments: Iterable[SomeSubstitutionsType],
        actions: Iterable[Action] = [],
        expected_output: Iterable[str] = [],
        description: Optional[str] = None
    ):
        """
        Constructor.

        :param command: `ros2` command to be tested.
        :param description: description of the test being done. command to be tested.
            It usually contains the verb and arguments being tested.
        :param arguments: A list of `SomeSubstitutionsType`, which are passed
            as arguments to the command.
        :param actions: A list of actions, which are launched before the `ros2` command.
        :param expected_output: A list of str, which are checked to be contained in the output
            of the `ros2` command.
        """
        self.command = command
        self.arguments = [normalize_to_list_of_substitutions(arg) for arg in arguments]
        self.actions = actions
        self.expected_output = expected_output
        self.description = description

        def describe(some_subs: SomeSubstitutionsType):
            return ''.join([sub.describe() for sub in some_subs])
        if description is None:
            self.description = 'ros2 {} {}'.format(
                command,
                ' '.join([describe(some_subs) for some_subs in self.arguments]))

    def __repr__(self):
        """Return the description."""
        return self.description
