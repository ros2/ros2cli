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

import re
import sys
import unittest

from geometry_msgs.msg import PointStamped  # Used because delay requires a message with a header

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.actions import ResetEnvironment
from launch.event_handlers import OnShutdown

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
from launch_testing_ros.actions import EnableRmwIsolation
import launch_testing_ros.tools

import pytest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy
from rclpy.qos import qos_check_compatible
from rclpy.qos import QoSCompatibility
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.utilities import get_rmw_implementation_identifier


# Skip cli tests on Windows while they exhibit pathological behavior
# https://github.com/ros2/build_farmer/issues/248
if sys.platform.startswith('win'):
    pytest.skip(
            'CLI tests can block for a pathological amount of time on Windows.',
            allow_module_level=True)


TEST_NODE = 'cli_bw_delay_hz_test_node'
TEST_NAMESPACE = 'cli_bw_delay_hz'


@pytest.mark.rostest
@launch_testing.markers.keep_alive
def generate_test_description():
    return LaunchDescription([
        # Always restart daemon to isolate tests.
        ExecuteProcess(
            cmd=['ros2', 'daemon', 'stop'],
            name='daemon-stop',
            on_exit=[
                EnableRmwIsolation(),
                RegisterEventHandler(OnShutdown(on_shutdown=[
                    # Stop daemon in isolated environment with proper ROS_DOMAIN_ID
                    ExecuteProcess(
                        cmd=['ros2', 'daemon', 'stop'],
                        name='daemon-stop-isolated',
                    ),
                    # This must be done after stopping the daemon in the isolated environment
                    ResetEnvironment(),
                ])),
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


class TestROS2TopicBwDelayHz(unittest.TestCase):

    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node(
            TEST_NODE, namespace=TEST_NAMESPACE, context=self.context
        )
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    def helper_verb_basic(self, launch_service, proc_info, proc_output, verb, success_regex):
        params = [
            (f'/clitest/topic/{verb}_basic', False, True),
            (f'/clitest/topic/{verb}_compatible_qos', True, True),
            (f'/clitest/topic/{verb}_incompatible_qos', True, False),
        ]
        for topic, provide_qos, compatible_qos in params:
            with self.subTest(topic=topic, provide_qos=provide_qos, compatible_qos=compatible_qos):
                # Check for inconsistent arguments
                assert provide_qos if not compatible_qos else True
                verb_extra_options = []
                publisher_qos_profile = 10
                if provide_qos:
                    if compatible_qos:
                        # For compatible test, put publisher at very high quality
                        # and subscription at low
                        verb_extra_options = [
                            '--qos-reliability', 'best_effort',
                            '--qos-durability', 'volatile']
                        publisher_qos_profile = QoSProfile(
                            depth=10,
                            reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
                    else:
                        # For an incompatible example, reverse the quality extremes
                        # and expect no messages to arrive
                        verb_extra_options = [
                            '--qos-reliability', 'reliable',
                            '--qos-durability', 'transient_local']
                        # This QoS profile matched with the extra options defined above
                        rostopic_qos_profile = QoSProfile(
                            depth=10,
                            reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
                        publisher_qos_profile = QoSProfile(
                            depth=10,
                            reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE)
                        # Skip this test if the QoS between the publisher and subscription
                        # are compatible according to the underlying middleware.
                        comp, reason = qos_check_compatible(
                            rostopic_qos_profile, publisher_qos_profile)
                        if comp == QoSCompatibility.OK or comp == QoSCompatibility.WARNING:
                            raise unittest.SkipTest()

                publisher = self.node.create_publisher(PointStamped, topic, publisher_qos_profile)
                assert publisher

                def publish_message():
                    msg = PointStamped()
                    # msg.header.stamp = self.node.get_clock().now()
                    publisher.publish(msg)

                publish_timer = self.node.create_timer(0.5, publish_message)

                # Wait for the publisher to be discovered
                publisher_count = 0
                timeout_count = 0
                while publisher_count == 0 and timeout_count < 10:
                    self.executor.spin_once(timeout_sec=0.1)
                    publisher_count = self.node.count_publishers(topic)
                    timeout_count += 1
                assert publisher_count > 0, 'Publisher was not discovered'

                try:
                    command_action = ExecuteProcess(
                        cmd=(['ros2', 'topic', verb] +
                             verb_extra_options +
                             [topic]),
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
                    command.wait_for_shutdown(timeout=10)
                    # Check results
                    if compatible_qos:
                        assert command.output, f'{verb} CLI printed no output'
                        assert re.search(success_regex, command.output, flags=re.MULTILINE), (
                            f'{verb} CLI did not print expected message'
                        )
                    else:
                        assert command.output, (
                            f'{verb} CLI did not print incompatible QoS warning'
                        )
                        assert ("New publisher discovered on topic '{}', offering incompatible"
                                ' QoS.'.format(topic) in command.output), (
                                f'{verb} CLI did not print expected incompatible QoS warning'
                            )
                finally:
                    # Cleanup
                    self.node.destroy_timer(publish_timer)
                    self.node.destroy_publisher(publisher)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_bw_basic(self, launch_service, proc_info, proc_output):
        self.helper_verb_basic(
            launch_service,
            proc_info,
            proc_output,
            'bw',
            r'^[0-9]+ B/s from [0-9]+ messages$'
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_delay_basic(self, launch_service, proc_info, proc_output):
        self.helper_verb_basic(
            launch_service,
            proc_info,
            proc_output,
            'delay',
            r'^average delay: [0-9\.]+$'
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_hw_basic(self, launch_service, proc_info, proc_output):
        self.helper_verb_basic(
            launch_service,
            proc_info,
            proc_output,
            'hz',
            r'^average rate: [0-9\.]+$'
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_hz_all_topics(self, launch_service, proc_info, proc_output):
        topics = [
            f'/{TEST_NAMESPACE}/test_topic_1',
            f'/{TEST_NAMESPACE}/test_topic_2',
            f'/{TEST_NAMESPACE}/test_topic_3',
        ]
        publishers = []
        timers = []

        for topic in topics:
            publisher = self.node.create_publisher(PointStamped, topic, 10)
            publishers.append(publisher)

            def publish_message(pub=publisher):
                msg = PointStamped()
                pub.publish(msg)

            timer = self.node.create_timer(0.5, publish_message)
            timers.append(timer)

        # Wait for all the publishers to be discovered
        timeout_count = 0
        all_discovered = False
        while not all_discovered and timeout_count < 30:
            self.executor.spin_once(timeout_sec=0.1)
            all_discovered = all(
                self.node.count_publishers(topic) > 0 for topic in topics
            )
            timeout_count += 1
        assert all_discovered, 'Not all publishers were discovered'

        try:
            command_action = ExecuteProcess(
                cmd=['ros2', 'topic', 'hz', '--all'],
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
                # Let it run for a few seconds to collect statistics
                self.executor.spin_until_future_complete(
                    rclpy.task.Future(), timeout_sec=5
                )
            command.wait_for_shutdown(timeout=10)

            assert command.output, 'hz --all CLI printed no output'
            assert re.search(
                r'Subscribing to all \d+ available topics',
                command.output
            ), 'hz --all did not print subscription message'

            for topic in topics:
                assert topic in command.output, (
                    f'Topic {topic} not found in hz --all output'
                )

            # Check that the output contains rate information (table format for multiple topics)
            # The table should have headers: topic, rate, min_delta, max_delta, std_dev, window
            assert 'topic' in command.output, 'Output missing topic header'
            assert 'rate' in command.output, 'Output missing rate header'

        finally:
            # Cleanup
            for timer in timers:
                self.node.destroy_timer(timer)
            for publisher in publishers:
                self.node.destroy_publisher(publisher)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_hz_no_arguments_error(self, launch_service, proc_info, proc_output):
        """Test that hz fails when neither --all nor topic names are provided."""
        command_action = ExecuteProcess(
            cmd=['ros2', 'topic', 'hz'],
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
            command.wait_for_shutdown(timeout=10)

        # Should fail with non-zero exit code
        assert command.exit_code != launch_testing.asserts.EXIT_OK, (
            'hz command should fail when no arguments provided'
        )

        # Should print the error message
        assert command.output, 'hz command printed no output'
        assert 'Either specify topic names or use --all/-a option' in command.output, (
            'hz command did not print expected error message'
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_hz_both_all_and_topics_error(self, launch_service, proc_info, proc_output):
        """Test that hz fails when both --all and topic names are provided."""
        command_action = ExecuteProcess(
            cmd=['ros2', 'topic', 'hz', '--all', '/some_topic'],
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
            command.wait_for_shutdown(timeout=10)

        # Should fail with non-zero exit code
        assert command.exit_code != launch_testing.asserts.EXIT_OK, (
            'hz command should fail when both --all and topic names provided'
        )

        # Should print the error message
        assert command.output, 'hz command printed no output'
        assert 'Cannot specify both --all/-a and topic names' in command.output, (
            'hz command did not print expected error message'
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_bw_multiple_topics(self, launch_service, proc_info, proc_output):
        topic1 = '/clitest/topic/bw_multi_1'
        topic2 = '/clitest/topic/bw_multi_2'

        publisher1 = self.node.create_publisher(PointStamped, topic1, 10)
        publisher2 = self.node.create_publisher(PointStamped, topic2, 10)
        assert publisher1
        assert publisher2

        def publish_messages():
            msg = PointStamped()
            publisher1.publish(msg)
            publisher2.publish(msg)

        publish_timer = self.node.create_timer(0.5, publish_messages)

        # Wait for the publishers to be discovered
        timeout_count = 0
        while timeout_count < 10:
            self.executor.spin_once(timeout_sec=0.1)
            if (self.node.count_publishers(topic1) > 0 and
                    self.node.count_publishers(topic2) > 0):
                break
            timeout_count += 1
        assert self.node.count_publishers(topic1) > 0, 'Publisher 1 was not discovered'
        assert self.node.count_publishers(topic2) > 0, 'Publisher 2 was not discovered'

        try:
            command_action = ExecuteProcess(
                cmd=['ros2', 'topic', 'bw', topic1, topic2],
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
            command.wait_for_shutdown(timeout=10)
            # Check results
            assert command.output, 'bw CLI printed no output'
            # Should contain both topic names
            assert topic1 in command.output, f'Output should contain {topic1}'
            assert topic2 in command.output, f'Output should contain {topic2}'
            # Should contain table headers for multiple topics
            assert 'bandwidth' in command.output, 'Output should contain bandwidth header'
            assert 'window' in command.output, 'Output should contain window header'
        finally:
            # Cleanup
            self.node.destroy_timer(publish_timer)
            self.node.destroy_publisher(publisher1)
            self.node.destroy_publisher(publisher2)
