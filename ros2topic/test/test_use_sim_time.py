# Copyright 2020 Sony Corporation.
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

import re
import functools
import sys
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
import launch_testing_ros.tools

import pytest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.utilities import get_rmw_implementation_identifier

from std_msgs.msg import String
from rosgraph_msgs.msg import Clock


# Skip cli tests on Windows while they exhibit pathological behavior
# https://github.com/ros2/build_farmer/issues/248
if sys.platform.startswith('win'):
    pytest.skip(
            'CLI tests can block for a pathological amount of time on Windows.',
            allow_module_level=True)


TEST_NODE = 'cli_use_sim_time_test_node'
TEST_NAMESPACE = 'cli_use_sim_time'


@pytest.mark.rostest
@launch_testing.markers.keep_alive
def generate_test_description():
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
                        launch_testing.actions.ReadyToTest()
                    ],
                )
            ]
        )
    ])


class TestROS2TopicUseSimTime(unittest.TestCase):

    def timer_callback(self):
        msg = Clock()
        msg.clock.sec = self.clock_sec
        self.publisher.publish(msg)
        self.clock_sec += 1

    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node(
            TEST_NODE, namespace=TEST_NAMESPACE, context=self.context
        )
        self.publisher = self.node.create_publisher(Clock, '/clock', 10)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_pub_times(self, launch_service, proc_info, proc_output):
        self.clock_sec = 0
        clock_timer = self.node.create_timer(1.0, self.timer_callback)

        try:
            command_action = ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '-t', '5', '-w', '0', '-s',
                     '/clitest/topic/pub', 'std_msgs/String', 'data: hello'],
                additional_env={
                    'PYTHONUNBUFFERED': '1'
                },
            )
            with launch_testing.tools.launch_process(
                launch_service, command_action, proc_info, proc_output,
                output_filter=launch_testing_ros.tools.basic_output_filter(
                    filtered_rmw_implementation=get_rmw_implementation_identifier()
                )
            ) as command:
                # The future won't complete - we will hit the timeout
                self.executor.spin_until_future_complete(
                    rclpy.task.Future(), timeout_sec=10
                )
                assert command.wait_for_shutdown(timeout=5)
            assert launch_testing.tools.expect_output(
                expected_lines=[
                    'publisher: beginning loop',
                    "publishing #1: std_msgs.msg.String(data='hello')",
                    '',
                    "publishing #2: std_msgs.msg.String(data='hello')",
                    '',
                    "publishing #3: std_msgs.msg.String(data='hello')",
                    '',
                    "publishing #4: std_msgs.msg.String(data='hello')",
                    '',
                    "publishing #5: std_msgs.msg.String(data='hello')",
                    '',
                ],
                text=command.output,
                strict=True
            )

        finally:
            # Cleanup
            self.node.destroy_timer(clock_timer)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_pub_hz(self, launch_service, proc_info, proc_output):
        average_rate_line_pattern = re.compile(r'average rate: (\d+.\d{3})')
        stats_line_pattern = re.compile(
            r'\s*min: \d+.\d{3}s max: \d+.\d{3}s std dev: \d+.\d{5}s window: \d+'
        )
        topic = '/clitest/topic/hz'
        publisher = self.node.create_publisher(String, topic, 10)
        assert publisher

        def publish_message():
            publisher.publish(String(data='hello'))

        publish_timer = self.node.create_timer(0.5, publish_message)

        self.clock_sec = 0
        clock_timer = self.node.create_timer(1.0, self.timer_callback)

        try:
            command_action = ExecuteProcess(
                cmd=['ros2', 'topic', 'hz', '-s', topic],
                additional_env={
                    'PYTHONUNBUFFERED': '1'
                },
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, command_action, proc_info, proc_output,
                output_filter=launch_testing_ros.tools.basic_output_filter(
                    filtered_rmw_implementation=get_rmw_implementation_identifier()
                )
            ) as command:
                # The future won't complete - we will hit the timeout
                self.executor.spin_until_future_complete(
                    rclpy.task.Future(), timeout_sec=5
                )
                assert command.wait_for_output(functools.partial(
                    launch_testing.tools.expect_output, expected_lines=[
                        average_rate_line_pattern,
                        stats_line_pattern
                    ], strict=True
                ), timeout=10), 'Echo CLI did not print expected message'
            assert command.wait_for_shutdown(timeout=10)

        finally:
            # Cleanup
            self.node.destroy_timer(publish_timer)
            self.node.destroy_timer(clock_timer)
            self.node.destroy_publisher(publisher)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_pub_bw(self, launch_service, proc_info, proc_output):
        topic = '/clitest/topic/bw'
        publisher = self.node.create_publisher(String, topic, 10)
        assert publisher

        def publish_message():
            publisher.publish(String(data='hello'))

        publish_timer = self.node.create_timer(1.0, publish_message)

        self.clock_sec = 0
        clock_timer = self.node.create_timer(1.0, self.timer_callback)

        try:
            command_action = ExecuteProcess(
                cmd=['ros2', 'topic', 'bw', '-s', topic],
                additional_env={
                    'PYTHONUNBUFFERED': '1'
                },
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, command_action, proc_info, proc_output,
                output_filter=launch_testing_ros.tools.basic_output_filter(
                    filtered_rmw_implementation=get_rmw_implementation_identifier()
                )
            ) as command:
                # The future won't complete - we will hit the timeout
                self.executor.spin_until_future_complete(
                    rclpy.task.Future(), timeout_sec=5
                )
                assert command.wait_for_output(functools.partial(
                    launch_testing.tools.expect_output, expected_lines=[
                        'Subscribed to [{}]'.format(topic),
                        re.compile(r'\d+ B/s from \d+ messages'),
                        re.compile(r'\s*Message size mean: \d+ B min: \d+ B max: \d+ B')
                    ], strict=True
                ), timeout=5), 'Echo CLI did not print expected message'
            assert command.wait_for_shutdown(timeout=10)

        finally:
            # Cleanup
            self.node.destroy_timer(publish_timer)
            self.node.destroy_timer(clock_timer)
            self.node.destroy_publisher(publisher)
