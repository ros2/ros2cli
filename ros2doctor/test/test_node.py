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
import unittest


from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.actions import ResetEnvironment
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnShutdown

from launch_ros.actions import Node

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
from launch_testing_ros.actions import EnableRmwIsolation
import launch_testing_ros.tools

import pytest

from rclpy.utilities import get_available_rmw_implementations
from ros2cli.helpers import get_rmw_additional_env
from ros2doctor.api.node import find_duplicates
from ros2doctor.api.node import NodeCheck
from ros2doctor.api.node import NodeReport


def test_find_duplicates():
    assert not find_duplicates([])
    assert not find_duplicates(['ns_foo/foo', 'ns_foo/bar'])
    assert find_duplicates(['ns_foo/foo'] * 2) == ['ns_foo/foo']
    assert find_duplicates(['ns_foo/foo'] * 3) == ['ns_foo/foo']
    assert find_duplicates(['ns_foo/foo', 'ns_foo/foo', 'ns_foo/bar']) == ['ns_foo/foo']


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation):
    path_to_complex_node_script = os.path.join(
        os.path.dirname(__file__), 'fixtures', 'complex_node.py'
    )
    additional_env = get_rmw_additional_env(rmw_implementation)
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
                        additional_env=dict(additional_env),
                    ),
                    # Run after stopping the daemon in the isolated environment
                    ResetEnvironment(),
                ])),
                ExecuteProcess(
                    cmd=['ros2', 'daemon', 'start'],
                    name='daemon-start',
                    on_exit=[
                        Node(
                            executable=sys.executable,
                            arguments=[path_to_complex_node_script],
                            name='complex_node',
                        ),
                        Node(
                            executable=sys.executable,
                            arguments=[path_to_complex_node_script],
                            name='complex_node',
                        ),
                        Node(
                            executable=sys.executable,
                            arguments=[path_to_complex_node_script],
                            name='_hidden_complex_node',
                        ),
                        launch_testing.actions.ReadyToTest(),
                    ],
                )
            ]
        ),
    ])


class TestROS2DoctorNodeAPI(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output,
        rmw_implementation
    ):
        @contextlib.contextmanager
        def launch_doctor_command(self, arguments):
            additional_env = {
                'PYTHONUNBUFFERED': '1',
            }
            doctor_command_action = ExecuteProcess(
                cmd=['ros2', 'doctor', *arguments],
                additional_env=additional_env,
                name='ros2doctor-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, doctor_command_action, proc_info, proc_output,
                output_filter=launch_testing_ros.tools.basic_output_filter(
                    filtered_patterns=['.*launch_ros.*', '.*ros2cli.*'],
                    filtered_rmw_implementation=rmw_implementation
                )
            ) as doctor_command:
                yield doctor_command
        cls.launch_doctor_command = launch_doctor_command

    # Test the Node api
    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_node_report(self):
        """Test NodeReport lists all nodes including hidden ones."""
        report = NodeReport().report()
        assert report.name == 'NODE LIST'

        reported_nodes = [v for k, v in report.items if k == 'node']

        # Verify nodes are listed as per creation
        assert '/complex_node' in reported_nodes
        assert '/_hidden_complex_node' in reported_nodes

        # Verify node count
        items_dict = {item[0]: item[1] for item in report.items}
        node_count = items_dict.get('node count')
        assert node_count is not None
        # We launched: 2 complex_node, 1 _hidden_complex_node = 3 nodes
        assert node_count >= 3

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_node_check_duplicates(self):
        """Test NodeCheck detects and issues warnings for duplicates node names."""
        node_check = NodeCheck()
        check_result = node_check.check()

        # We launched two nodes with the same name "complex_node" , so 1 warning issued
        assert check_result.error == 0
        assert check_result.warning >= 1

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_doctor_check_cli(self):
        """Test ros2 doctor CLI runs check without errors."""
        with self.launch_doctor_command(arguments=[]) as doctor_command:
            assert doctor_command.wait_for_shutdown(timeout=20)
        # Check may print warnings due to duplicates, but should not fail
        assert doctor_command.output

    @launch_testing.markers.retry_on_failure(times=5, delay=1)
    def test_doctor_report_cli(self):
        """Test ros2 doctor --report CLI includes node information."""
        with self.launch_doctor_command(arguments=['--report']) as doctor_command:
            assert doctor_command.wait_for_shutdown(timeout=20)
        assert doctor_command.exit_code == launch_testing.asserts.EXIT_OK

        # Verify NODE LIST section is present in report
        assert 'NODE LIST' in doctor_command.output or 'node count' in doctor_command.output
