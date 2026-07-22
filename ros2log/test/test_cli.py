# Copyright 2026 Tomoya Fujita, Fumiya Ohnishi
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
import os
import re
import sys
import time
import unittest
import xmlrpc.client

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

import rclpy
from rclpy.utilities import get_available_rmw_implementations
from ros2cli.helpers import get_rmw_additional_env
from ros2cli.node.strategy import NodeStrategy

from ros2node.api import get_node_names


# Skip cli tests on Windows while they exhibit pathological behavior
# https://github.com/ros2/build_farmer/issues/248
if sys.platform.startswith('win'):
    pytest.skip(
        'CLI tests can block for a pathological amount of time on Windows.',
        allow_module_level=True)


TEST_TIMEOUT = 20.0
DISCOVERY_POLL_INTERVAL = 0.1


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation):
    path_to_fixtures = os.path.join(os.path.dirname(__file__), 'fixtures')
    additional_env = get_rmw_additional_env(rmw_implementation)
    additional_env['PYTHONUNBUFFERED'] = '1'
    set_env_actions = [SetEnvironmentVariable(k, v) for k, v in additional_env.items()]

    path_to_talker_node_script = os.path.join(path_to_fixtures, 'talker_node.py')
    path_to_listener_node_script = os.path.join(path_to_fixtures, 'listener_node.py')

    talker_node_action = Node(
        executable=sys.executable,
        arguments=[path_to_talker_node_script],
        name='talker',
    )

    listener_node_action = Node(
        executable=sys.executable,
        arguments=[path_to_listener_node_script],
        name='listener',
    )

    return LaunchDescription([
        # Always restart daemon to isolate tests.
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
                    on_exit=[
                        talker_node_action,
                        listener_node_action,
                        launch_testing.actions.ReadyToTest(),
                    ],
                )
            ]
        ),
    ])


class TestROS2LogCLI(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output,
        rmw_implementation
    ):
        cls.path_to_set_logger_level_script = os.path.join(
            os.path.dirname(__file__), 'fixtures', 'set_logger_level.py')
        rmw_implementation_filter = launch_testing_ros.tools.basic_output_filter(
            filtered_patterns=['WARNING:.*'],
            filtered_rmw_implementation=rmw_implementation
        )

        @contextlib.contextmanager
        def launch_process_command(self, command, *, name):
            command_action = ExecuteProcess(
                cmd=command,
                name=name,
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, command_action, proc_info, proc_output,
                output_filter=rmw_implementation_filter
            ) as command_process:
                yield command_process
        cls.launch_process_command = launch_process_command

        @contextlib.contextmanager
        def launch_log_command(self, arguments):
            with self.launch_process_command(
                ['ros2', 'log', *arguments],
                name='ros2log-cli',
            ) as log_command:
                yield log_command
        cls.launch_log_command = launch_log_command

    def setUp(self):
        start_time = time.time()
        timed_out = True
        with NodeStrategy(None) as node:
            while (time.time() - start_time) < TEST_TIMEOUT:
                try:
                    node_names = {
                        discovered_node.full_name
                        for discovered_node in get_node_names(node=node)
                    }
                    talker_services = node.get_service_names_and_types_by_node('talker', '/')
                except rclpy.node.NodeNameNonExistentError:
                    time.sleep(DISCOVERY_POLL_INTERVAL)
                    continue
                except ConnectionRefusedError:
                    time.sleep(DISCOVERY_POLL_INTERVAL)
                    continue
                except xmlrpc.client.Fault as exc:
                    if 'NodeNameNonExistentError' in exc.faultString:
                        time.sleep(DISCOVERY_POLL_INTERVAL)
                        continue
                    raise

                talker_service_names = {name for name, _ in talker_services}
                if (
                    '/talker' in node_names and
                    '/listener' in node_names and
                    '/talker/get_logger_levels' in talker_service_names and
                    '/talker/set_logger_levels' in talker_service_names
                ):
                    timed_out = False
                    break
                time.sleep(DISCOVERY_POLL_INTERVAL)

        if timed_out:
            self.fail(f'CLI daemon failed to find test nodes after {TEST_TIMEOUT} seconds')

        self._set_logger_level_directly('/talker', 'UNSET')

    def _set_logger_level_directly(self, node_name, level_name):
        # Launch a fresh helper process so each parametrized RMW test gets a
        # clean rclpy context instead of reusing the one from a previous run.
        with self.launch_process_command(
            [sys.executable, self.path_to_set_logger_level_script, node_name, level_name],
            name='ros2log-set-level-helper',
        ) as helper_command:
            assert helper_command.wait_for_shutdown(timeout=TEST_TIMEOUT)

        if helper_command.exit_code != launch_testing.asserts.EXIT_OK:
            self.fail(
                'Failed to set logger level directly:\n'
                f'{helper_command.output}'
            )

    @launch_testing.markers.retry_on_failure(times=2, delay=1)
    def test_watch_basic(self):
        """Test basic ros2 log watch command."""
        with self.launch_log_command(arguments=['watch']) as log_command:
            assert log_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'.*\[INFO\].*\[(talker|listener)\] : Info message:'),
                ], strict=False
            ), timeout=10)
        assert log_command.wait_for_shutdown(timeout=10)

    @launch_testing.markers.retry_on_failure(times=2, delay=1)
    def test_watch_level_filter(self):
        """Test ros2 log watch with level filter."""
        with self.launch_log_command(
            arguments=['watch', '--level', 'ERROR']
        ) as log_command:
            assert log_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'.*\[ERROR\].*\[(talker|listener)\] : Error message'),
                ], strict=False
            ), timeout=10)
        assert log_command.wait_for_shutdown(timeout=10)

    @launch_testing.markers.retry_on_failure(times=2, delay=1)
    def test_watch_logger_filter(self):
        """Test ros2 log watch with logger name filter."""
        with self.launch_log_command(
            arguments=['watch', '--logger', 'talker']
        ) as log_command:
            assert log_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'.*\[INFO\].*\[talker\] : Info message'),
                ], strict=False
            ), timeout=10)
        assert log_command.wait_for_shutdown(timeout=10)

    @launch_testing.markers.retry_on_failure(times=2, delay=1)
    def test_watch_regex_filter(self):
        """Test ros2 log watch with regex filter."""
        with self.launch_log_command(
            arguments=['watch', '--regex', 'Publishing.*']
        ) as log_command:
            assert log_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'.*Publishing: Hello World'),
                ], strict=False
            ), timeout=10)
        assert log_command.wait_for_shutdown(timeout=10)

    @launch_testing.markers.retry_on_failure(times=2, delay=1)
    def test_watch_no_color(self):
        """Test ros2 log watch with color disabled."""
        with self.launch_log_command(
            arguments=['watch', '--no-color']
        ) as log_command:
            assert log_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'.*\[INFO\].*\[(talker|listener)\] : Info message'),
                ], strict=False
            ), timeout=10)
        assert log_command.wait_for_shutdown(timeout=10)
        # Check that no ANSI escape codes are present
        assert '\033[' not in log_command.output

    @launch_testing.markers.retry_on_failure(times=2, delay=1)
    def test_watch_no_timestamp(self):
        """Test ros2 log watch with timestamp disabled."""
        with self.launch_log_command(
            arguments=['watch', '--no-timestamp']
        ) as log_command:
            assert log_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'.*\[INFO\].*\[(talker|listener)\] : Info message'),
                ], strict=False
            ), timeout=10)
        assert log_command.wait_for_shutdown(timeout=10)

    @launch_testing.markers.retry_on_failure(times=2, delay=1)
    def test_watch_function_detail(self):
        """Test ros2 log watch with function details enabled."""
        with self.launch_log_command(
            arguments=['watch', '--function-detail']
        ) as log_command:
            assert log_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'.*\[INFO\].*\[(talker|listener)\] \[.*@.*:\d+\] : Info message'),
                ], strict=False
            ), timeout=10)
        assert log_command.wait_for_shutdown(timeout=10)

    @launch_testing.markers.retry_on_failure(times=2, delay=1)
    def test_watch_combined_filters(self):
        """Test ros2 log watch with multiple filters."""
        with self.launch_log_command(
            arguments=[
                'watch',
                '--level', 'INFO',
                '--logger', 'talker',
                '--no-color',
                '--no-timestamp'
            ]
        ) as log_command:
            assert log_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'.*\[INFO\].*\[talker\] : Info message:'),
                ], strict=False
            ), timeout=10)
        assert log_command.wait_for_shutdown(timeout=10)
        # Should have no ANSI codes
        assert '\033[' not in log_command.output

    @launch_testing.markers.retry_on_failure(times=2, delay=1)
    def test_watch_with_debug_flag(self):
        """Test ros2 log watch with global debug flag."""
        with self.launch_log_command(
            arguments=['--debug', 'watch', '--logger', 'talker']
        ) as log_command:
            assert log_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'.*\[INFO\].*\[talker\] : Info message'),
                ], strict=False
            ), timeout=10)
        assert log_command.wait_for_shutdown(timeout=10)

    @launch_testing.markers.retry_on_failure(times=2, delay=1)
    def test_watch_with_qos_options(self):
        """Test ros2 log watch with QoS options."""
        with self.launch_log_command(
            arguments=[
                'watch',
                '--qos-reliability', 'reliable',
                '--qos-durability', 'transient_local'
            ]
        ) as log_command:
            assert log_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'.*\[INFO\].*\[(talker|listener)\] : Info message'),
                ], strict=False
            ), timeout=10)
        assert log_command.wait_for_shutdown(timeout=10)

    @launch_testing.markers.retry_on_failure(times=2, delay=1)
    def test_levels_basic(self):
        """Test ros2 log levels command."""
        with self.launch_log_command(arguments=['levels']) as log_command:
            assert log_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'^UNSET\s+:.*'),
                    re.compile(r'^DEBUG\s+:.*'),
                    re.compile(r'^INFO\s+:.*'),
                    re.compile(r'^WARN\s+:.*'),
                    re.compile(r'^ERROR\s+:.*'),
                    re.compile(r'^FATAL\s+:.*'),
                ], strict=False
            ), timeout=10)
        assert log_command.wait_for_shutdown(timeout=10)

    @launch_testing.markers.retry_on_failure(times=2, delay=1)
    def test_levels_with_value(self):
        """Test ros2 log levels --value command."""
        with self.launch_log_command(arguments=['levels', '--value']) as log_command:
            assert log_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'^UNSET\s+\(\s*\d+\)\s*:.*'),
                    re.compile(r'^DEBUG\s+\(\s*\d+\)\s*:.*'),
                ], strict=False
            ), timeout=10)
        assert log_command.wait_for_shutdown(timeout=10)

    @launch_testing.markers.retry_on_failure(times=2, delay=1)
    def test_list_logger_service_nodes(self):
        """Test ros2 log list command."""
        with self.launch_log_command(arguments=['list']) as log_command:
            assert log_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'^/talker$'),
                ], strict=False
            ), timeout=10)
        assert log_command.wait_for_shutdown(timeout=10)
        assert '/no_logger_service' not in log_command.output

    @launch_testing.markers.retry_on_failure(times=2, delay=1)
    def test_get_single_node(self):
        """Test ros2 log get for a single node."""
        with self.launch_log_command(arguments=['get', '/talker']) as log_command:
            assert log_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'^UNSET$'),
                ], strict=False
            ), timeout=10)
        assert log_command.wait_for_shutdown(timeout=10)

    @launch_testing.markers.retry_on_failure(times=2, delay=1)
    def test_get_all_nodes(self):
        """Test ros2 log get --all."""
        with self.launch_log_command(arguments=['get', '--all']) as log_command:
            assert log_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'^/talker: UNSET$'),
                ], strict=False
            ), timeout=10)
        assert log_command.wait_for_shutdown(timeout=10)
        assert '/listener:' not in log_command.output

    @launch_testing.markers.retry_on_failure(times=2, delay=1)
    def test_get_reports_missing_logger_service(self):
        """Test ros2 log get on a node without logger services."""
        with self.launch_log_command(arguments=['get', '/listener']) as log_command:
            assert log_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r"Logger service not available for node '/listener'\."),
                    re.compile(r'.*enable_logger_service.*'),
                ], strict=False
            ), timeout=10)
        assert log_command.wait_for_shutdown(timeout=10)

    @launch_testing.markers.retry_on_failure(times=2, delay=1)
    def test_set_single_node(self):
        """Test ros2 log set for a single node."""
        with self.launch_log_command(arguments=['set', '/talker', 'DEBUG']) as log_command:
            assert log_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'^Set logger level successful$'),
                ], strict=False
            ), timeout=10)
        assert log_command.wait_for_shutdown(timeout=10)

        with self.launch_log_command(arguments=['get', '/talker']) as log_command:
            assert log_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'^DEBUG$'),
                ], strict=False
            ), timeout=10)
        assert log_command.wait_for_shutdown(timeout=10)

    @launch_testing.markers.retry_on_failure(times=2, delay=1)
    def test_set_all_nodes(self):
        """Test ros2 log set --all."""
        with self.launch_log_command(arguments=['set', '--all', 'ERROR']) as log_command:
            assert log_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'^/talker: Set logger level successful$'),
                ], strict=False
            ), timeout=10)
        assert log_command.wait_for_shutdown(timeout=10)

        with self.launch_log_command(arguments=['get', '--all']) as log_command:
            assert log_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'^/talker: ERROR$'),
                ], strict=False
            ), timeout=10)
        assert log_command.wait_for_shutdown(timeout=10)
