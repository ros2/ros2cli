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

from typing import Callable
from typing import Iterable
from typing import Mapping
from typing import Optional
from typing import Pattern
from typing import Union

from launch import Action
from launch import SomeSubstitutionsType
from launch.utilities import normalize_to_list_of_substitutions

import launch_testing.asserts

ExpectedOutputType = Union[
    Iterable[Union[str, Pattern[str], Callable[[str], bool]]],
    Callable[[Iterable[str]], bool]
]
SomeExpectedOutputType = Union[ExpectedOutputType, Mapping[str, ExpectedOutputType]]


class CLITestConfiguration(object):
    """A class for configuring the `test_process_output_customizable.py.in` test."""

    def __init__(
        self,
        *,
        command: str,
        arguments: Iterable[SomeSubstitutionsType],
        description: Optional[str] = None,
        fixture_actions: Optional[Iterable[Action]] = None,
        setup_delay: Optional[int] = None,
        expected_output: Optional[SomeExpectedOutputType] = None,
        output_filter: Optional[Callable[[str], str]] = None,
        exit_codes: Optional[Iterable[int]] = None,
        self_terminates: bool = True,
        timeout: int = 30.
    ):
        """
        Constructor.

        :param command: `ros2` command to be tested.
        :param arguments: A list of `SomeSubstitutionsType`, which are passed
            as arguments to the command.
        :param description: description of the test being done. command to be tested.
            It usually contains the verb and arguments being tested.
        :param fixture_actions: A list of actions, which are launched before the `ros2`
            command under test.
        :param setup_delay: A period in seconds to wait for fixture actions to start up
            before launching the `ros2` command under test. If none is given, an appropriate
            default is automatically provided.
        :param expected_output: Output expectations for the `ros2` command under test and/or
            fixture actions. In its most explicit form it is a mapping from process names to
            either a list of expected lines, which may be expressed as plain strings, compiled
            regexes, or a callable for custom line assertion, or a callable that takes all lines
            for custom output assertions. Empty lists express that no output is expected. The
            `ros2` command under test can be referenced as 'cli'. Alternatively, this mapping
            can be elided if expectations are only defined for the `ros2` command under test.
            In that case, only the corresponding mapping value has to be provided.
        :param output_filter: A callable to filter output before testing it against expectations.
        :param exit_codes: A list of allowed exit codes for the `ros2` command under test.
        :param self_terminates: A flag for command termination policy. Non-self terminating
            commands are forced to terminate after the specified timeout has elapsed.
        :param timeout: A finite test timeout in seconds.
        """
        self.command = command
        self.arguments = [normalize_to_list_of_substitutions(arg) for arg in arguments]
        if description is None:
            description = 'ros2 {} {}'.format(
                command, ' '.join([
                    ''.join([sub.describe() for sub in some_subs]) for some_subs in self.arguments
                ])
            )
        self.description = description
        if fixture_actions is None:
            fixture_actions = []
        self.fixture_actions = fixture_actions
        self.expected_output = expected_output
        self.output_filter = output_filter
        if exit_codes is None:
            exit_codes = [launch_testing.asserts.EXIT_OK]
        self.exit_codes = exit_codes
        self.self_terminates = self_terminates
        self.setup_delay = setup_delay
        self.timeout = timeout

    def __repr__(self):
        """Return the description."""
        return self.description
