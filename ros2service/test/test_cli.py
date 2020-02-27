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
import functools
import itertools
import os
import re
import sys
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

import launch_testing
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
import launch_testing_ros.tools

import pytest

from rclpy.utilities import get_available_rmw_implementations

from test_msgs.srv import BasicTypes


def get_echo_call_output(**kwargs):
    request = BasicTypes.Request()
    for field_name, field_value in kwargs.items():
        setattr(request, field_name, field_value)
    response = BasicTypes.Response()
    for field_name, field_value in kwargs.items():
        setattr(response, field_name, field_value)
    return [
        'requester: making request: ' + repr(request),
        '',
        'response:',
        repr(response),
        ''
    ]


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation, ready_fn):
    path_to_echo_server_script = os.path.join(
        os.path.dirname(__file__), 'fixtures', 'echo_server.py'
    )
    additional_env = {'RMW_IMPLEMENTATION': rmw_implementation}
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
                        # Add test fixture actions.
                        Node(
                            node_executable=sys.executable,
                            arguments=[path_to_echo_server_script],
                            node_name='echo_server',
                            node_namespace='my_ns',
                            additional_env=additional_env,
                        ),
                        Node(
                            node_executable=sys.executable,
                            arguments=[path_to_echo_server_script],
                            node_name='_hidden_echo_server',
                            node_namespace='my_ns',
                            remappings=[('echo', '_echo')],
                            additional_env=additional_env,
                        ),
                        OpaqueFunction(function=lambda context: ready_fn())
                    ],
                    additional_env=additional_env
                )
            ]
        ),
    ])


class TestROS2ServiceCLI(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output,
        rmw_implementation
    ):
        @contextlib.contextmanager
        def launch_service_command(self, arguments):
            service_command_action = ExecuteProcess(
                cmd=['ros2', 'service', *arguments],
                additional_env={
                    'RMW_IMPLEMENTATION': rmw_implementation,
                    'PYTHONUNBUFFERED': '1'
                },
                name='ros2service-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, service_command_action, proc_info, proc_output,
                output_filter=launch_testing_ros.tools.basic_output_filter(
                    filtered_prefixes=[
                        'waiting for service to become available...',
                        '/launch_ros'  # cope with launch_ros internal node.
                    ],
                    filtered_rmw_implementation=rmw_implementation
                )
            ) as service_command:
                yield service_command
        cls.launch_service_command = launch_service_command

    @launch_testing.markers.retry_on_failure(times=5)
    def test_list_services(self):
        with self.launch_service_command(arguments=['list']) as service_command:
            assert service_command.wait_for_shutdown(timeout=10)
        assert service_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=itertools.chain(
                ['/my_ns/echo'],
                itertools.repeat(re.compile(
                    r'/my_ns/echo_server/.*parameter.*'
                ), 6)
            ),
            text=service_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_list_hidden(self):
        with self.launch_service_command(
            arguments=['--include-hidden-services', 'list']
        ) as service_command:
            assert service_command.wait_for_shutdown(timeout=10)
        assert service_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=itertools.chain(
                ['/my_ns/_echo'],
                itertools.repeat(re.compile(
                    r'/my_ns/_hidden_echo_server/.*parameter.*'
                ), 6),
                ['/my_ns/echo'],
                itertools.repeat(re.compile(
                    r'/my_ns/echo_server/.*parameter.*'
                ), 6)
            ),
            text=service_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_list_with_types(self):
        with self.launch_service_command(arguments=['list', '-t']) as service_command:
            assert service_command.wait_for_shutdown(timeout=10)
        assert service_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=itertools.chain(
                ['/my_ns/echo [test_msgs/srv/BasicTypes]'],
                itertools.repeat(re.compile(
                    r'/my_ns/echo_server/.*parameter.*'
                    r' \[rcl_interfaces/srv/.*Parameter.*\]'
                ), 6)
            ),
            text=service_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_list_count(self):
        with self.launch_service_command(arguments=['list', '-c']) as service_command:
            assert service_command.wait_for_shutdown(timeout=10)
        assert service_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = service_command.output.splitlines()
        assert len(output_lines) == 1
        assert int(output_lines[0]) == 7 + 6  # cope with launch_ros internal node.

    @launch_testing.markers.retry_on_failure(times=5)
    def test_find(self):
        with self.launch_service_command(
            arguments=['find', 'test_msgs/srv/BasicTypes']
        ) as service_command:
            assert service_command.wait_for_shutdown(timeout=10)
        assert service_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['/my_ns/echo'],
            text=service_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_find_hidden(self):
        with self.launch_service_command(
            arguments=['find', '--include-hidden-services', 'test_msgs/srv/BasicTypes']
        ) as service_command:
            assert service_command.wait_for_shutdown(timeout=10)
        assert service_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['/my_ns/_echo', '/my_ns/echo'],
            text=service_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_find_count(self):
        with self.launch_service_command(
            arguments=['find', '-c', 'test_msgs/srv/BasicTypes']
        ) as service_command:
            assert service_command.wait_for_shutdown(timeout=10)
        assert service_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = service_command.output.splitlines()
        assert len(output_lines) == 1
        assert int(output_lines[0]) == 1

    def test_find_not_a_service_type(self):
        with self.launch_service_command(
            arguments=['find', 'not_a_service_type']
        ) as service_command:
            assert service_command.wait_for_shutdown(timeout=10)
        assert service_command.exit_code == launch_testing.asserts.EXIT_OK
        assert service_command.output == ''

    def test_type(self):
        with self.launch_service_command(
            arguments=['type', '/my_ns/echo']
        ) as service_command:
            assert service_command.wait_for_shutdown(timeout=10)
        assert service_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['test_msgs/srv/BasicTypes'],
            text=service_command.output,
            strict=True
        )

    def test_type_on_not_a_service(self):
        with self.launch_service_command(
            arguments=['type', '/not_a_service']
        ) as service_command:
            assert service_command.wait_for_shutdown(timeout=10)
        assert service_command.exit_code == 1
        assert service_command.output == ''

    @launch_testing.markers.retry_on_failure(times=5)
    def test_call_no_args(self):
        with self.launch_service_command(
            arguments=['call', '/my_ns/echo', 'test_msgs/srv/BasicTypes']
        ) as service_command:
            assert service_command.wait_for_shutdown(timeout=10)
        assert service_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=get_echo_call_output(),
            text=service_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_call(self):
        with self.launch_service_command(
            arguments=[
                'call',
                '/my_ns/echo',
                'test_msgs/srv/BasicTypes',
                '{bool_value: false, int32_value: -1, float64_value: 0.1, string_value: bazbar}'
            ]
        ) as service_command:
            assert service_command.wait_for_shutdown(timeout=10)
        assert service_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=get_echo_call_output(
                bool_value=False,
                int32_value=-1,
                float64_value=0.1,
                string_value='bazbar'
            ),
            text=service_command.output,
            strict=True
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_repeated_call(self):
        with self.launch_service_command(
            arguments=[
                'call',
                '-r', '1',
                '/my_ns/echo',
                'test_msgs/srv/BasicTypes',
                '{bool_value: true, int32_value: 1, float64_value: 1.0, string_value: foobar}'
            ],
        ) as service_command:
            assert service_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=2 * get_echo_call_output(
                    bool_value=True, int32_value=1, float64_value=1.0, string_value='foobar'
                )
            ), timeout=10)
