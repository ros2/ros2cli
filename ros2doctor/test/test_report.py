# Copyright 2025 Open Source Robotics Foundation, Inc.
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
import os
import sys
from typing import Any
from typing import Dict
from typing import Generator
from typing import Tuple
import unittest

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess

from launch_ros.actions import Node

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
import launch_testing_ros.tools

import pytest

from rclpy.utilities import get_available_rmw_implementations
from ros2cli.helpers import get_rmw_additional_env


# Skip cli tests on Windows while they exhibit pathological behavior
# https://github.com/ros2/build_farmer/issues/248
if sys.platform.startswith('win'):
    pytest.skip(
        'CLI tests can block for a pathological amount of time on Windows.',
        allow_module_level=True)


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation: str) -> Tuple[LaunchDescription,
                                                                Dict[str, Any]]:
    path_to_fixtures = os.path.join(os.path.dirname(__file__), 'fixtures')
    additional_env = get_rmw_additional_env(rmw_implementation)
    additional_env['PYTHONUNBUFFERED'] = '1'

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'daemon', 'stop'],
            name='daemon-stop',
            on_exit=[
                ExecuteProcess(
                    cmd=['ros2', 'daemon', 'start'],
                    name='daemon-start',
                    additional_env=additional_env,
                    on_exit=[
                        Node(
                            executable=sys.executable,
                            arguments=[os.path.join(path_to_fixtures, 'report_node.py')],
                            additional_env=additional_env
                        ),
                        launch_testing.actions.ReadyToTest()
                    ]
                )
            ]
        )
    ]), locals()


class TestROS2DoctorReport(unittest.TestCase):

    @classmethod
    def setUpClass(
            cls,
            launch_service: LaunchService,
            proc_info: launch_testing.tools.process.ActiveProcInfoHandler,
            proc_output: launch_testing.tools.process.ActiveIoHandler,
            rmw_implementation: str,
    ) -> None:
        cls.rmw_implementation = rmw_implementation
        rmw_implementation_filter = launch_testing_ros.tools.basic_output_filter(
            filtered_patterns=['WARNING: topic .* does not appear to be published yet'],
            filtered_rmw_implementation=rmw_implementation
        )

        @contextlib.contextmanager
        def launch_doctor_command(
            self,
            arguments
        ) -> Generator[launch_testing.tools.process.ProcessProxy, None, None]:
            additional_env = get_rmw_additional_env(rmw_implementation)
            additional_env['PYTHONUNBUFFERED'] = '1'
            doctor_command_action = ExecuteProcess(
                cmd=['ros2', 'doctor', *arguments],
                additional_env=additional_env,
                name='ros2doctor-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                    launch_service, doctor_command_action, proc_info, proc_output,
                    output_filter=rmw_implementation_filter
            ) as doctor_command:
                yield doctor_command
        cls.launch_doctor_command = launch_doctor_command

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_check(self) -> None:
        with self.launch_doctor_command(
                arguments=[]
        ) as doctor_command:
            assert doctor_command.wait_for_shutdown(timeout=10)
        assert doctor_command.exit_code == launch_testing.asserts.EXIT_OK
        assert doctor_command.output

        lines_list = [line for line in doctor_command.output.splitlines() if line]
        assert 'All' in lines_list[-1]
        assert 'checks passed' in lines_list[-1]

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_report(self) -> None:
        # TODO(@fujitatomoya): rmw_zenoh_cpp is instable to find the endpoints, it does not
        # matter if DaemonNode or DirectNode is used. For now, skip the test for rmw_zenoh_cpp.
        if self.rmw_implementation == 'rmw_zenoh_cpp':
            raise unittest.SkipTest()

        for argument in ['-r', '--report']:
            with self.launch_doctor_command(
                    arguments=[argument]
            ) as doctor_command:
                assert doctor_command.wait_for_shutdown(timeout=10)
            assert doctor_command.exit_code == launch_testing.asserts.EXIT_OK

            assert launch_testing.tools.expect_output(
                expected_lines=[
                    'topic               : /msg',
                    'publisher count     : 1',
                    'subscriber count    : 1'
                ],
                text=doctor_command.output
            )

            assert launch_testing.tools.expect_output(
                expected_lines=[
                    'service          : /bar',
                    'service count    : 1',
                    'client count     : 0',
                    'service          : /baz',
                    'service count    : 0',
                    'client count     : 1',
                    'service          : /report_node/get_type_description',
                    'service count    : 1',
                    'client count     : 0'
                ],
                text=doctor_command.output)

            assert launch_testing.tools.expect_output(
                expected_lines=[
                    'action                 : /fibonacci',
                    'action server count    : 1',
                    'action client count    : 1'
                ],
                text=doctor_command.output)
