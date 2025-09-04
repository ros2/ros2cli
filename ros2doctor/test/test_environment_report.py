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
from typing import Generator
import unittest

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.actions import ResetEnvironment
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnShutdown

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
from launch_testing_ros.actions import EnableRmwIsolation

import pytest

from rclpy.utilities import get_available_rmw_implementations
from ros2cli.helpers import get_rmw_additional_env


# Skip cli tests on Windows while they exhibit pathological behavior
# https://github.com/ros2/build_farmer/issues/248
if sys.platform.startswith('win'):
    pytest.skip(
        'CLI tests can block for a pathological amount of time on Windows.',
        allow_module_level=True)


CYCLONEDDS_XML = '<CycloneDDS><Domain><General></General></Domain></CycloneDDS>'


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
@launch_testing.markers.keep_alive
def generate_test_description(rmw_implementation: str) -> tuple[LaunchDescription,
                                                                dict[str, Any]]:
    additional_env = get_rmw_additional_env(rmw_implementation)
    additional_env['PYTHONUNBUFFERED'] = '1'
    set_env_actions = [SetEnvironmentVariable(k, v) for k, v in additional_env.items()]

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'daemon', 'stop'],
            name='daemon-stop',
            on_exit=[
                *set_env_actions,
                EnableRmwIsolation(),
                RegisterEventHandler(OnShutdown(on_shutdown=[
                    # Stop daemon in isolated environment with proper ROS_DOMAIN_ID
                    ExecuteProcess(
                        cmd=['ros2', 'daemon', 'stop'],
                        name='daemon-stop-isolated',
                        # Use the same isolated environment
                        additional_env=dict(additional_env),
                    ),
                    # This must be done after stopping the daemon in the isolated environment
                    ResetEnvironment(),
                ])),
                ExecuteProcess(
                    cmd=['ros2', 'daemon', 'start'],
                    name='daemon-start',
                    additional_env=additional_env,
                    on_exit=[
                        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET'),
                        SetEnvironmentVariable('ROS_DISTRO', 'rolling'),
                        SetEnvironmentVariable('ROS_DISABLE_LOANED_MESSAGES', '0'),
                        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '0'),
                        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv6'),
                        SetEnvironmentVariable('ZENOH_ROUTER_CHECK_ATTEMPTS', '10'),
                        SetEnvironmentVariable('RMW_CONNEXT_INITIAL_PEERS', 'CONNEXTBOO'),
                        SetEnvironmentVariable('CYCLONEDDS_URI', CYCLONEDDS_XML),
                        launch_testing.actions.ReadyToTest()
                    ]
                )
            ]
        )
    ]), locals()


class TestEnvironmentReport(unittest.TestCase):

    @classmethod
    def setUpClass(
            cls,
            launch_service: LaunchService,
            proc_info: launch_testing.tools.process.ActiveProcInfoHandler,
            proc_output: launch_testing.tools.process.ActiveIoHandler,
            rmw_implementation: str,
    ) -> None:
        cls.rmw_implementation = rmw_implementation

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
                name='ros2doctor-environment-report',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                    launch_service, doctor_command_action, proc_info, proc_output
            ) as doctor_command:
                yield doctor_command

        cls.launch_doctor_command = launch_doctor_command

        if rmw_implementation == 'rmw_cyclonedds_cpp':
            cls.expected_line = f'CYCLONEDDS_URI={CYCLONEDDS_XML}'
        elif rmw_implementation == 'rmw_connextdds':
            cls.expected_line = 'RMW_CONNEXT_INITIAL_PEERS=CONNEXTBOO'
        elif rmw_implementation == 'rmw_zenoh_cpp':
            config = os.environ['ZENOH_CONFIG_OVERRIDE']
            cls.expected_line = f'RUST_LOG=z=error, ZENOH_CONFIG_OVERRIDE={config}, '
            'ZENOH_ROUTER_CHECK_ATTEMPTS=10'
        elif rmw_implementation == 'rmw_fastrtps_cpp':
            cls.expected_line = 'FASTDDS_BUILTIN_TRANSPORTS=UDPv6'
        else:
            cls.expected_line = ''

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_environment_report(self) -> None:

        for argument in ['-r', '--report']:
            with self.launch_doctor_command(
                    arguments=[argument]
            ) as doctor_command:
                assert doctor_command.wait_for_shutdown(timeout=10)
            assert doctor_command.exit_code == launch_testing.asserts.EXIT_OK

            # Due to isolation ROS_DOMAIN_ID is unique.
            # ROS_DOMAIN_ID Does not seem to be set for zenoh with EnableRmwIsolation.
            domain_id_line = ''
            if self.rmw_implementation == 'rmw_zenoh_cpp':
                domain_id_line = ''
            else:
                domain_id = os.environ['ROS_DOMAIN_ID']
                domain_id_line = f', ROS_DOMAIN_ID={domain_id}'

            assert launch_testing.tools.expect_output(
                expected_lines=[
                    'ROS environment variables        : ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET, '
                    f'ROS_DISABLE_LOANED_MESSAGES=0, ROS_DISTRO=rolling{domain_id_line}',
                    'rcutils environment variables    : RCUTILS_COLORIZED_OUTPUT=0',
                    f'rmw environment variables        : {self.expected_line}'
                ],
                text=doctor_command.output
            )
