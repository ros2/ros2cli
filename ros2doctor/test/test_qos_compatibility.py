# Copyright 2021 Open Source Robotics Foundation, Inc.
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
import re
import sys
import unittest

from launch import LaunchDescription
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


# Skip cli tests on Windows while they exhibit pathological behavior
# https://github.com/ros2/build_farmer/issues/248
if sys.platform.startswith('win'):
    pytest.skip(
        'CLI tests can block for a pathological amount of time on Windows.',
        allow_module_level=True)


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation):
    path_to_fixtures = os.path.join(os.path.dirname(__file__), 'fixtures')
    additional_env = {
        'RMW_IMPLEMENTATION': rmw_implementation, 'PYTHONUNBUFFERED': '1'
    }

    path_to_incompatible_talker_node_script = os.path.join(
        path_to_fixtures, 'talker_node_with_best_effort_qos.py')
    path_to_compatible_talker_node_script = os.path.join(
        path_to_fixtures, 'talker_node_with_reliable_qos.py')

    path_to_listener_node_script = os.path.join(
        path_to_fixtures, 'listener_node_with_reliable_qos.py')

    talker_node_compatible = Node(
        executable=sys.executable,
        arguments=[path_to_compatible_talker_node_script],
        remappings=[('chatter', 'compatible_chatter')],
        additional_env=additional_env
    )
    listener_node_compatible = Node(
        executable=sys.executable,
        arguments=[path_to_listener_node_script],
        remappings=[('chatter', 'compatible_chatter')],
        additional_env=additional_env
    )
    talker_node_incompatible = Node(
        executable=sys.executable,
        arguments=[path_to_incompatible_talker_node_script],
        remappings=[('chatter', 'incompatible_chatter')],
        additional_env=additional_env
    )
    listener_node_incompatible = Node(
        executable=sys.executable,
        arguments=[path_to_listener_node_script],
        remappings=[('chatter', 'incompatible_chatter')],
        additional_env=additional_env
    )

    return LaunchDescription([
        # Always restart daemon to isolate tests.
        ExecuteProcess(
            cmd=['ros2', 'daemon', 'stop'],
            name='daemon-stop',
            on_exit=[
                ExecuteProcess(
                    cmd=['ros2', 'daemon', 'start'],
                    name='daemon-start',
                    on_exit=[
                        # Add incompatible talker/listener pair.
                        talker_node_incompatible,
                        listener_node_incompatible,
                        talker_node_compatible,
                        listener_node_compatible,
                        launch_testing.actions.ReadyToTest()
                    ],
                    additional_env=additional_env
                )
            ]
        ),
    ]), locals()


class TestROS2DoctorQoSCompatibility(unittest.TestCase):

    @classmethod
    def setUpClass(
            cls,
            launch_service,
            proc_info,
            proc_output,
            rmw_implementation,
    ):
        rmw_implementation_filter = launch_testing_ros.tools.basic_output_filter(
            filtered_patterns=['WARNING: topic .* does not appear to be published yet'],
            filtered_rmw_implementation=rmw_implementation
        )

        @contextlib.contextmanager
        def launch_doctor_command(self, arguments):
            doctor_command_action = ExecuteProcess(
                cmd=['ros2', 'doctor', *arguments],
                additional_env={
                    'RMW_IMPLEMENTATION': rmw_implementation,
                    'PYTHONUNBUFFERED': '1'
                },
                name='ros2doctor-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                    launch_service, doctor_command_action, proc_info, proc_output,
                    output_filter=rmw_implementation_filter
            ) as doctor_command:
                yield doctor_command
        cls.launch_doctor_command = launch_doctor_command

    def test_check(self):
        with self.launch_doctor_command(
                arguments=[]
        ) as doctor_command:
            assert doctor_command.wait_for_shutdown(timeout=10)
        assert doctor_command.exit_code == launch_testing.asserts.EXIT_OK
        assert doctor_command.output

        lines_list = [line for line in doctor_command.output.splitlines() if line]
        assert lines_list[-1] == 'Failed modules: middleware'
        assert re.search(r'^1/\d+ check\(s\) failed$', lines_list[-2])

    def test_report(self):
        for argument in ['-r', '--report']:
            with self.launch_doctor_command(
                    arguments=[argument]
            ) as doctor_command:
                assert doctor_command.wait_for_shutdown(timeout=10)
            assert doctor_command.exit_code == launch_testing.asserts.EXIT_OK
            assert ("topic [type]            : /compatible_chatter [std_msgs/msg/String]\n"
                    "publisher node          : talker_node\n"
                    "subscriber node         : listener\n"
                    "compatibility status    : OK") in doctor_command.output
            assert ("topic [type]            : /incompatible_chatter [std_msgs/msg/String]\n"
                    "publisher node          : talker_node\n"
                    "subscriber node         : listener\n"
                    "compatibility status    : "
                    "ERROR: Best effort publisher and reliable subscription;") \
                in doctor_command.output
