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
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction

import launch_testing
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools

import pytest


some_messages_from_std_msgs = [
    'std_msgs/msg/Bool',
    'std_msgs/msg/Float32',
    'std_msgs/msg/Float64',
]

some_services_from_std_srvs = [
    'std_srvs/srv/Empty',
    'std_srvs/srv/SetBool',
    'std_srvs/srv/Trigger',
]

some_actions_from_test_msgs = [
    'test_msgs/action/Fibonacci'
]

some_interfaces = (
    some_messages_from_std_msgs +
    some_services_from_std_srvs +
    some_actions_from_test_msgs
)


@pytest.mark.rostest
@launch_testing.markers.keep_alive
def generate_test_description(ready_fn):
    return LaunchDescription([OpaqueFunction(function=lambda context: ready_fn())])


class TestROS2InterfaceCLI(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output
    ):
        @contextlib.contextmanager
        def launch_interface_command(self, arguments):
            interface_command_action = ExecuteProcess(
                cmd=['ros2', 'interface', *arguments],
                additional_env={'PYTHONUNBUFFERED': '1'},
                name='ros2interface-cli',
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
                re.compile(r'\s*[A-z0-9_]+/[A-z0-9_]+'), len(output_lines)
            ),
            lines=output_lines,
            strict=True
        )
        some_interfaces_without_ns = []
        for ifc in some_interfaces:
            parts = ifc.split('/')
            some_interfaces_without_ns.append(
                '/'.join([parts[0], *parts[2:]])
            )
        assert launch_testing.tools.expect_output(
            expected_lines=some_interfaces_without_ns,
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
                    re.compile(r'\s*[A-z0-9_]+/[A-z0-9_]+'), len(output_lines) - 1
                )
            ),
            lines=output_lines,
            strict=True
        )
        some_messages_from_std_msgs_without_ns = []
        for msg in some_messages_from_std_msgs:
            parts = msg.split('/')
            some_messages_from_std_msgs_without_ns.append(
                '/'.join([parts[0], *parts[2:]])
            )
        assert launch_testing.tools.expect_output(
            expected_lines=some_messages_from_std_msgs_without_ns,
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
                    re.compile(r'\s*[A-z0-9_]+/[A-z0-9_]+'), len(output_lines) - 1
                )
            ),
            lines=output_lines,
            strict=True
        )
        some_services_from_std_srvs_without_ns = []
        for srv in some_services_from_std_srvs:
            parts = srv.split('/')
            some_services_from_std_srvs_without_ns.append(
                '/'.join([parts[0], *parts[2:]])
            )
        assert launch_testing.tools.expect_output(
            expected_lines=some_services_from_std_srvs_without_ns,
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
                    re.compile(r'\s*[A-z0-9_]+/[A-z0-9_]+'), len(output_lines) - 1
                )
            ),
            lines=output_lines,
            strict=True
        )
        some_actions_from_test_msgs_without_ns = []
        for action in some_actions_from_test_msgs:
            parts = action.split('/')
            some_actions_from_test_msgs_without_ns.append(
                '/'.join([parts[0], *parts[2:]])
            )
        assert launch_testing.tools.expect_output(
            expected_lines=some_actions_from_test_msgs_without_ns,
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

    def test_package_on_std_msgs(self):
        with self.launch_interface_command(
            arguments=['package', 'std_msgs']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = interface_command.output.splitlines()
        assert launch_testing.tools.expect_output(
            expected_lines=itertools.repeat(
                re.compile(r'std_msgs/msg/[A-z0-9_]+'), len(output_lines)
            ),
            lines=output_lines,
            strict=True
        )
        assert all(msg in output_lines for msg in some_messages_from_std_msgs)

    def test_package_on_std_srvs(self):
        with self.launch_interface_command(
            arguments=['package', 'std_srvs']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = interface_command.output.splitlines()
        assert launch_testing.tools.expect_output(
            expected_lines=itertools.repeat(
                re.compile(r'std_srvs/srv/[A-z0-9_]+'), len(output_lines)
            ),
            lines=output_lines,
            strict=True
        )
        assert all(srv in output_lines for srv in some_services_from_std_srvs)

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
        assert all(action in output_lines for action in some_actions_from_test_msgs)

    def test_packages(self):
        with self.launch_interface_command(arguments=['packages']) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = interface_command.output.splitlines()
        assert 'std_msgs' in output_lines
        assert 'std_srvs' in output_lines
        assert 'test_msgs' in output_lines

    def test_packages_with_messages(self):
        with self.launch_interface_command(
            arguments=['packages', '-m']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = interface_command.output.splitlines()
        assert 'std_msgs' in output_lines
        assert 'std_srvs' not in output_lines
        assert 'test_msgs' in output_lines

    def test_packages_with_services(self):
        with self.launch_interface_command(
            arguments=['packages', '-s']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = interface_command.output.splitlines()
        assert 'std_msgs' not in output_lines
        assert 'std_srvs' in output_lines
        assert 'test_msgs' in output_lines

    def test_packages_with_actions(self):
        with self.launch_interface_command(
            arguments=['packages', '-a']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = interface_command.output.splitlines()
        assert 'std_msgs' not in output_lines
        assert 'std_srvs' not in output_lines
        assert 'test_msgs' in output_lines

    def test_show_message(self):
        with self.launch_interface_command(
            arguments=['show', 'std_msgs/msg/String']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                'string data'
            ],
            text=interface_command.output,
            strict=True
        )

    def test_show_service(self):
        with self.launch_interface_command(
            arguments=['show', 'std_srvs/srv/SetBool']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                'bool data # e.g. for hardware enabling / disabling',
                '---',
                'bool success   # indicate successful run of triggered service',
                'string message # informational, e.g. for error messages'
            ],
            text=interface_command.output,
            strict=True
        )

    def test_show_action(self):
        with self.launch_interface_command(
            arguments=['show', 'test_msgs/action/Fibonacci']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                '#goal definition',
                'int32 order',
                '---',
                '#result definition',
                'int32[] sequence',
                '---',
                '#feedback',
                'int32[] sequence',
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
            arguments=['show', 'std_msgs/msg/NotAMessageTypeName']
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
