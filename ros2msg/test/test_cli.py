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


some_messages_from_std_msgs = [
    'std_msgs/msg/Bool',
    'std_msgs/msg/Float32',
    'std_msgs/msg/Float64',
]


class TestROS2MsgCLI(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output
    ):
        @contextlib.contextmanager
        def launch_msg_command(self, arguments):
            msg_command_action = ExecuteProcess(
                cmd=['ros2', 'msg', *arguments],
                additional_env={'PYTHONUNBUFFERED': '1'},
                name='ros2msg-cli', output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, msg_command_action, proc_info, proc_output,
                output_filter=launch_testing.tools.basic_output_filter(
                    filtered_patterns=[
                        r".*'ros2 msg' is deprecated and will be removed in a future ROS release.*"
                    ]
                )
            ) as msg_command:
                yield msg_command
        cls.launch_msg_command = launch_msg_command

    def test_list_messages(self):
        with self.launch_msg_command(arguments=['list']) as msg_command:
            assert msg_command.wait_for_shutdown(timeout=10)
        assert msg_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = msg_command.output.splitlines()
        assert all(msg in output_lines for msg in some_messages_from_std_msgs)
        assert all(re.match(r'.*/msg/.*', line) is not None for line in output_lines)

    def test_package_messages(self):
        with self.launch_msg_command(arguments=['package', 'std_msgs']) as msg_command:
            assert msg_command.wait_for_shutdown(timeout=10)
        assert msg_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = msg_command.output.splitlines()
        assert all(msg in output_lines for msg in some_messages_from_std_msgs)
        assert all(re.match(r'std_msgs/msg/.*', line) is not None for line in output_lines)

    def test_not_a_package_messages(self):
        with self.launch_msg_command(arguments=['package', 'not_a_package']) as msg_command:
            assert msg_command.wait_for_shutdown(timeout=10)
        assert msg_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Unknown package name'],
            text=msg_command.output, strict=True
        )

    def test_list_packages_with_messages(self):
        with self.launch_msg_command(arguments=['packages']) as msg_command:
            assert msg_command.wait_for_shutdown(timeout=10)
        assert msg_command.exit_code == launch_testing.asserts.EXIT_OK
        assert 'std_msgs' in msg_command.output.splitlines()

    def test_show_message(self):
        with self.launch_msg_command(
            arguments=['show', 'std_msgs/msg/String']
        ) as msg_command:
            assert msg_command.wait_for_shutdown(timeout=10)
        assert msg_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['string data'],
            text=msg_command.output, strict=False
        )

    def test_show_not_a_message_typename(self):
        with self.launch_msg_command(
            arguments=['show', 'std_msgs/msg/NotAMessageTypeName']
        ) as msg_command:
            assert msg_command.wait_for_shutdown(timeout=10)
        assert msg_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Unknown message name'],
            text=msg_command.output, strict=True
        )

    # TODO(hidmic): make 'ros2 msg show' fail accordingly
    # def test_show_not_a_message_ns(self):
    #     with self.launch_msg_command(
    #         arguments=['show', 'std_msgs/foo/String']
    #     ) as msg_command:
    #         assert msg_command.wait_for_shutdown(timeout=10)
    #     assert msg_command.exit_code == 1
    #     assert launch_testing.tools.expect_output(
    #         expected_lines=['Unknown message name'],
    #         text=msg_command.output, strict=True
    #     )

    def test_show_not_a_package(self):
        with self.launch_msg_command(
            arguments=['show', 'not_a_package/msg/String']
        ) as msg_command:
            assert msg_command.wait_for_shutdown(timeout=10)
        assert msg_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Unknown package name'],
            text=msg_command.output, strict=True
        )

    def test_show_not_a_message_type(self):
        with self.launch_msg_command(
            arguments=['show', 'not_a_message_type']
        ) as msg_command:
            assert msg_command.wait_for_shutdown(timeout=10)
        assert msg_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['The passed message type is invalid'],
            text=msg_command.output, strict=True
        )
