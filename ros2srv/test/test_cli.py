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


@pytest.mark.rostest
@launch_testing.markers.keep_alive
def generate_test_description(ready_fn):
    return LaunchDescription([OpaqueFunction(function=lambda context: ready_fn())])


some_services_from_std_srvs = [
    'std_srvs/srv/Empty',
    'std_srvs/srv/SetBool',
    'std_srvs/srv/Trigger',
]


class TestROS2SrvCLI(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output
    ):
        @contextlib.contextmanager
        def launch_srv_command(self, arguments):
            srv_command_action = ExecuteProcess(
                cmd=['ros2', 'srv', *arguments],
                additional_env={'PYTHONUNBUFFERED': '1'},
                name='ros2srv-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, srv_command_action, proc_info, proc_output
            ) as srv_command:
                yield srv_command
        cls.launch_srv_command = launch_srv_command

    def test_list_service_types(self):
        with self.launch_srv_command(arguments=['list']) as srv_command:
            assert srv_command.wait_for_shutdown(timeout=10)
        assert srv_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = srv_command.output.splitlines()
        assert all(srv in output_lines for srv in some_services_from_std_srvs)
        assert all(re.match(r'.*/srv/.*', line) is not None for line in output_lines)

    def test_list_service_types_in_a_package(self):
        with self.launch_srv_command(arguments=['package', 'std_srvs']) as srv_command:
            assert srv_command.wait_for_shutdown(timeout=10)
        assert srv_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = srv_command.output.splitlines()
        assert all(srv in output_lines for srv in some_services_from_std_srvs)
        assert all(re.match(r'std_srvs/srv/.*', line) is not None for line in output_lines)

    def test_list_service_types_in_not_a_package(self):
        with self.launch_srv_command(arguments=['package', 'not_a_package']) as srv_command:
            assert srv_command.wait_for_shutdown(timeout=10)
        assert srv_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Unknown package name'],
            text=srv_command.output
        )

    def test_list_packages_with_service_types(self):
        with self.launch_srv_command(arguments=['packages']) as srv_command:
            assert srv_command.wait_for_shutdown(timeout=10)
        assert srv_command.exit_code == launch_testing.asserts.EXIT_OK
        assert 'std_srvs' in srv_command.output.splitlines()

    def test_show_service_type(self):
        with self.launch_srv_command(
            arguments=['show', 'std_srvs/srv/SetBool']
        ) as srv_command:
            assert srv_command.wait_for_shutdown(timeout=10)
        assert srv_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                'bool data',
                '---',
                'bool success',
                'string message'
            ],
            text=srv_command.output,
            strict=False
        )

        with self.launch_srv_command(
            arguments=['show', 'std_srvs/srv/Trigger']
        ) as srv_command:
            assert srv_command.wait_for_shutdown(timeout=10)
        assert srv_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                '---',
                'bool success',
                'string message'
            ],
            text=srv_command.output,
            strict=False
        )

    def test_show_not_a_service_typename(self):
        with self.launch_srv_command(
            arguments=['show', 'std_srvs/srv/NotAServiceTypeName']
        ) as srv_command:
            assert srv_command.wait_for_shutdown(timeout=10)
        assert srv_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Unknown service name'],
            text=srv_command.output,
            strict=True
        )

    # TODO(hidmic): make 'ros2 srv show' more robust
    # def test_show_not_a_service_ns(self):
    #     with self.launch_srv_command(
    #         arguments=['show', 'std_srvs/foo/Empty']
    #     ) as srv_command:
    #         assert srv_command.wait_for_shutdown(timeout=10)
    #     assert srv_command.exit_code == 1
    #     assert launch_testing.tools.expect_output(
    #         expected_lines=['Unknown service name'],
    #         text=srv_command.output,
    #         strict=True
    #     )

    def test_show_not_a_package(self):
        with self.launch_srv_command(
            arguments=['show', 'not_a_package/srv/Empty']
        ) as srv_command:
            assert srv_command.wait_for_shutdown(timeout=10)
        assert srv_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Unknown package name'],
            text=srv_command.output,
            strict=True
        )

    def test_show_not_a_service_type(self):
        with self.launch_srv_command(
            arguments=['show', 'not_a_service_type']
        ) as srv_command:
            assert srv_command.wait_for_shutdown(timeout=10)
        assert srv_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['The passed service type is invalid'],
            text=srv_command.output,
            strict=True
        )
