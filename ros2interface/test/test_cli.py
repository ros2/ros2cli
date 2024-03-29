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

import contextlib
import itertools
import re
import sys
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools

import pytest


# Skip cli tests on Windows while they exhibit pathological behavior
# https://github.com/ros2/build_farmer/issues/248
if sys.platform.startswith('win'):
    pytest.skip(
            'CLI tests can block for a pathological amount of time on Windows.',
            allow_module_level=True)


some_messages_from_test_msgs = [
    'test_msgs/msg/BasicTypes',
    'test_msgs/msg/Constants',
    'test_msgs/msg/Strings',
]

some_services_from_test_msgs = [
    'test_msgs/srv/Arrays',
    'test_msgs/srv/BasicTypes',
    'test_msgs/srv/Empty',
]

some_actions_from_test_msgs = [
    'test_msgs/action/Fibonacci'
]

some_interfaces = (
    some_messages_from_test_msgs +
    some_services_from_test_msgs +
    some_actions_from_test_msgs
)


@pytest.mark.rostest
@launch_testing.markers.keep_alive
def generate_test_description():
    return LaunchDescription([launch_testing.actions.ReadyToTest()])


class TestROS2InterfaceCLI(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output
    ):
        @contextlib.contextmanager
        def launch_interface_command(self, arguments, prepend_arguments=[], shell=False):
            interface_command_action = ExecuteProcess(
                cmd=[*prepend_arguments, 'ros2', 'interface', *arguments],
                additional_env={'PYTHONUNBUFFERED': '1'},
                name='ros2interface-cli',
                shell=shell,
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, interface_command_action, proc_info, proc_output
            ) as interface_command:
                yield interface_command
        cls.launch_interface_command = launch_interface_command

    def test_list_interfaces(self):
        with self.launch_interface_command(arguments=['list']) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        filter_ = launch_testing.tools.basic_output_filter(
            filtered_prefixes=['Messages:', 'Services:', 'Actions:']
        )
        output_lines = filter_(interface_command.output).splitlines()
        assert launch_testing.tools.expect_output(
            expected_lines=itertools.repeat(
                re.compile(r'\s*[A-z0-9_]+(/[A-z0-9_]+)+'), len(output_lines)
            ),
            lines=output_lines,
            strict=True
        )
        assert launch_testing.tools.expect_output(
            expected_lines=some_interfaces,
            lines=output_lines,
            strict=False
        )

    def test_list_messages(self):
        with self.launch_interface_command(arguments=['list', '-m']) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = interface_command.output.splitlines()
        assert launch_testing.tools.expect_output(
            expected_lines=itertools.chain(
                ['Messages:'], itertools.repeat(
                    re.compile(r'\s*[A-z0-9_]+(/[A-z0-9_]+)+'), len(output_lines) - 1
                )
            ),
            lines=output_lines,
            strict=True
        )
        assert launch_testing.tools.expect_output(
            expected_lines=some_messages_from_test_msgs,
            lines=output_lines,
            strict=False
        )

    def test_list_services(self):
        with self.launch_interface_command(arguments=['list', '-s']) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = interface_command.output.splitlines()
        assert launch_testing.tools.expect_output(
            expected_lines=itertools.chain(
                ['Services:'], itertools.repeat(
                    re.compile(r'\s*[A-z0-9_]+(/[A-z0-9_]+)+'), len(output_lines) - 1
                )
            ),
            lines=output_lines,
            strict=True
        )
        assert launch_testing.tools.expect_output(
            expected_lines=some_services_from_test_msgs,
            lines=output_lines,
            strict=False
        )

    def test_list_actions(self):
        with self.launch_interface_command(arguments=['list', '-a']) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = interface_command.output.splitlines()
        assert launch_testing.tools.expect_output(
            expected_lines=itertools.chain(
                ['Actions:'], itertools.repeat(
                    re.compile(r'\s*[A-z0-9_]+(/[A-z0-9_]+)+'), len(output_lines) - 1
                )
            ),
            lines=output_lines,
            strict=True
        )
        assert launch_testing.tools.expect_output(
            expected_lines=some_actions_from_test_msgs,
            lines=output_lines,
            strict=False
        )

    def test_package_on_nonexistent_package(self):
        with self.launch_interface_command(
            arguments=['package', 'not_a_package']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=["Unknown package 'not_a_package'"],
            text=interface_command.output,
            strict=True
        )

    def test_package_on_test_msgs(self):
        with self.launch_interface_command(
            arguments=['package', 'test_msgs']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = interface_command.output.splitlines()
        assert launch_testing.tools.expect_output(
            expected_lines=itertools.repeat(
                re.compile(r'test_msgs/(msg|srv|action)/[A-z0-9_]+'), len(output_lines)
            ),
            lines=output_lines,
            strict=True
        )
        assert all(interface in output_lines for interface in some_interfaces)

    def test_package_on_test_msgs_only_msgs(self):
        with self.launch_interface_command(
            arguments=['package', 'test_msgs', '-m']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=5)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = interface_command.output.splitlines()
        assert launch_testing.tools.expect_output(
            expected_lines=itertools.repeat(
                re.compile(r'test_msgs/msg/[A-z0-9_]+'), len(output_lines)
            ),
            lines=output_lines,
            strict=True
        )
        assert all(interface in output_lines for interface in some_messages_from_test_msgs)

    def test_package_on_test_msgs_only_srvs(self):
        with self.launch_interface_command(
            arguments=['package', 'test_msgs', '-s']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=5)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = interface_command.output.splitlines()
        assert launch_testing.tools.expect_output(
            expected_lines=itertools.repeat(
                re.compile(r'test_msgs/srv/[A-z0-9_]+'), len(output_lines)
            ),
            lines=output_lines,
            strict=True
        )
        assert all(interface in output_lines for interface in some_services_from_test_msgs)

    def test_package_on_test_msgs_only_actions(self):
        with self.launch_interface_command(
            arguments=['package', 'test_msgs', '-a']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=5)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = interface_command.output.splitlines()
        assert launch_testing.tools.expect_output(
            expected_lines=itertools.repeat(
                re.compile(r'test_msgs/action/[A-z0-9_]+'), len(output_lines)
            ),
            lines=output_lines,
            strict=True
        )
        assert all(interface in output_lines for interface in some_actions_from_test_msgs)

    def test_packages(self):
        with self.launch_interface_command(arguments=['packages']) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = interface_command.output.splitlines()
        assert 'test_msgs' in output_lines

    def test_packages_with_messages(self):
        with self.launch_interface_command(
            arguments=['packages', '-m']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = interface_command.output.splitlines()
        assert 'test_msgs' in output_lines

    def test_packages_with_services(self):
        with self.launch_interface_command(
            arguments=['packages', '-s']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = interface_command.output.splitlines()
        assert 'test_msgs' in output_lines

    def test_packages_with_actions(self):
        with self.launch_interface_command(
            arguments=['packages', '-a']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = interface_command.output.splitlines()
        assert 'test_msgs' in output_lines

    def test_show_message(self):
        with self.launch_interface_command(
            arguments=['show', 'ros2cli_test_interfaces/msg/ShortVariedMultiNested']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                '# A short, varied, and nested type',
                'ShortVariedNested short_varied_nested # Comment - Nesting Level 3: 1 of 1',
                '\tShortVaried short_varied',
                '\t\tbool BOOL_CONST=true',
                '\t\tbool bool_value',
                '\t\tbool[<=3] bool_values',
                '# Trailing comment',
            ],
            text=interface_command.output,
            strict=True
        )

    def test_show_message_with_all_comments(self):
        with self.launch_interface_command(
            arguments=[
                'show', 'ros2cli_test_interfaces/msg/ShortVariedMultiNested', '--all-comments']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                '# A short, varied, and nested type',
                'ShortVariedNested short_varied_nested # Comment - Nesting Level 3: 1 of 1',
                '\t# A short, varied type',
                '\tShortVaried short_varied # Comment - Nesting Level 2: 1 of 1',
                '\t\t# A constant',
                '\t\tbool BOOL_CONST=true # Comment - Nesting Level 1: 1 of 2',
                '',
                '\t\t# Bool and array of bools',
                '\t\tbool bool_value',
                '\t\tbool[<=3] bool_values # Comment - Nesting Level 1: 2 of 2',
                '',
                '\t\t# Trailing comment',
                '\t# Trailing comment',
                '# Trailing comment',
            ],
            text=interface_command.output,
            strict=True
        )

    def test_show_message_with_no_comments(self):
        with self.launch_interface_command(
            arguments=[
                'show', 'ros2cli_test_interfaces/msg/ShortVariedMultiNested', '--no-comments']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                'ShortVariedNested short_varied_nested',
                '\tShortVaried short_varied',
                '\t\tbool BOOL_CONST=true',
                '\t\tbool bool_value',
                '\t\tbool[<=3] bool_values',
            ],
            text=interface_command.output,
            strict=True
        )

    def test_show_service(self):
        with self.launch_interface_command(
            arguments=['show', 'ros2cli_test_interfaces/srv/ShortVariedMultiNested']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                '# Request',
                'ShortVariedNested short_varied_nested # Comment - Nesting Level 3: 1 of 2',
                '\tShortVaried short_varied',
                '\t\tbool BOOL_CONST=true',
                '\t\tbool bool_value',
                '\t\tbool[<=3] bool_values',
                '---',
                '# Response',
                'bool bool_value # Comment - Nesting Level 3: 2 of 2',
            ],
            text=interface_command.output,
            strict=True
        )

    def test_show_service_with_all_comments(self):
        with self.launch_interface_command(
                arguments=[
                    'show', 'ros2cli_test_interfaces/srv/ShortVariedMultiNested', '--all-comments']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                '# Request',
                'ShortVariedNested short_varied_nested # Comment - Nesting Level 3: 1 of 2',
                '\t# A short, varied type',
                '\tShortVaried short_varied # Comment - Nesting Level 2: 1 of 1',
                '\t\t# A constant',
                '\t\tbool BOOL_CONST=true # Comment - Nesting Level 1: 1 of 2',
                '',
                '\t\t# Bool and array of bools',
                '\t\tbool bool_value',
                '\t\tbool[<=3] bool_values # Comment - Nesting Level 1: 2 of 2',
                '',
                '\t\t# Trailing comment',
                '\t# Trailing comment',
                '---',
                '# Response',
                'bool bool_value # Comment - Nesting Level 3: 2 of 2',
            ],
            text=interface_command.output,
            strict=True
        )

    def test_show_service_with_no_comments(self):
        with self.launch_interface_command(
                arguments=[
                    'show', 'ros2cli_test_interfaces/srv/ShortVariedMultiNested', '--no-comments']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                'ShortVariedNested short_varied_nested',
                '\tShortVaried short_varied',
                '\t\tbool BOOL_CONST=true',
                '\t\tbool bool_value',
                '\t\tbool[<=3] bool_values',
                '---',
                'bool bool_value',
            ],
            text=interface_command.output,
            strict=True
        )

    def test_show_action(self):
        with self.launch_interface_command(
                arguments=['show', 'ros2cli_test_interfaces/action/ShortVariedMultiNested']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                '# Goal definition',
                'ShortVariedNested short_varied_nested # Comment - Nesting Level 3: 1 of 2',
                '\tShortVaried short_varied',
                '\t\tbool BOOL_CONST=true',
                '\t\tbool bool_value',
                '\t\tbool[<=3] bool_values',
                '---',
                '# Result definition',
                'bool bool_value # Comment - Nesting Level 3: 2 of 2',
                '---',
                '# Feedback definition',
                'bool[3] bool_values',
            ],
            text=interface_command.output,
            strict=True
        )

    def test_show_action_with_all_comments(self):
        with self.launch_interface_command(
                arguments=[
                    'show',
                    'ros2cli_test_interfaces/action/ShortVariedMultiNested',
                    '--all-comments']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                '# Goal definition',
                'ShortVariedNested short_varied_nested # Comment - Nesting Level 3: 1 of 2',
                '\t# A short, varied type',
                '\tShortVaried short_varied # Comment - Nesting Level 2: 1 of 1',
                '\t\t# A constant',
                '\t\tbool BOOL_CONST=true # Comment - Nesting Level 1: 1 of 2',
                '',
                '\t\t# Bool and array of bools',
                '\t\tbool bool_value',
                '\t\tbool[<=3] bool_values # Comment - Nesting Level 1: 2 of 2',
                '',
                '\t\t# Trailing comment',
                '\t# Trailing comment',
                '---',
                '# Result definition',
                'bool bool_value # Comment - Nesting Level 3: 2 of 2',
                '---',
                '# Feedback definition',
                'bool[3] bool_values',
            ],
            text=interface_command.output,
            strict=True
        )

    def test_show_action_with_no_comments(self):
        with self.launch_interface_command(
                arguments=[
                    'show',
                    'ros2cli_test_interfaces/action/ShortVariedMultiNested',
                    '--no-comments']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                'ShortVariedNested short_varied_nested',
                '\tShortVaried short_varied',
                '\t\tbool BOOL_CONST=true',
                '\t\tbool bool_value',
                '\t\tbool[<=3] bool_values',
                '---',
                'bool bool_value',
                '---',
                'bool[3] bool_values',
            ],
            text=interface_command.output,
            strict=True
        )

    def test_show_not_a_package(self):
        with self.launch_interface_command(
            arguments=['show', 'not_a_package/msg/String']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=["Unknown package 'not_a_package'"],
            text=interface_command.output,
            strict=True
        )

    def test_show_not_an_interface(self):
        with self.launch_interface_command(
            arguments=['show', 'test_msgs/msg/NotAMessageTypeName']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=[re.compile(
                r"Could not find the interface '.+NotAMessageTypeName\.idl'"
            )],
            text=interface_command.output,
            strict=True
        )

    def test_show_stdin(self):
        with self.launch_interface_command(
            arguments=['show', '-'],
            prepend_arguments=[
                sys.executable, '-c', r'"print(\"test_msgs/msg/BasicTypes\")"', '|'
            ],
            shell=True
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                'bool bool_value',
                'byte byte_value',
                'char char_value',
                'float32 float32_value',
                'float64 float64_value',
                'int8 int8_value',
                'uint8 uint8_value',
                'int16 int16_value',
                'uint16 uint16_value',
                'int32 int32_value',
                'uint32 uint32_value',
                'int64 int64_value',
                'uint64 uint64_value',
            ],
            text=interface_command.output,
            strict=True
        )
