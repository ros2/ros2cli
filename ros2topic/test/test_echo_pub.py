# Copyright 2019 Amazon.com, Inc. or its affiliates. All rights reserved.
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

import os
import signal
import subprocess

import pytest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String

TEST_NODE = 'cli_echo_pub_test_node'
TEST_NAMESPACE = 'cli_echo_pub'


@pytest.fixture
def echo_pub_node():
    """Set up the global rclpy context and node for this test module."""
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node(TEST_NODE, namespace=TEST_NAMESPACE, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    yield node, executor, context
    node.destroy_node()
    rclpy.shutdown(context=context)


@pytest.mark.skipif(os.name == 'nt', reason='Subprocess bug in Windows Python 3.7.4')
@pytest.mark.timeout(60)
@pytest.mark.parametrize(
    'topic,provide_qos,compatible_qos', [
        ('/clitest/topic/pub_basic', False, True),
        ('/clitest/topic/pub_compatible_qos', True, True),
        ('/clitest/topic/pub_incompatible_qos', True, False)
    ]
)
def test_pub_basic(echo_pub_node, topic: str, provide_qos: bool, compatible_qos: bool):
    # Check for inconsistent arguments
    assert provide_qos if not compatible_qos else True
    node, executor, context = echo_pub_node
    received_message_count = 0
    expected_minimum_message_count = 1
    expected_maximum_message_count = 5
    future = rclpy.task.Future()
    pub_extra_options = []
    subscription_qos_profile = 10
    if provide_qos:
        if compatible_qos:
            # For compatible test, put publisher at very high quality and subscription at low
            pub_extra_options = [
                '--qos-reliability', 'reliable',
                '--qos-durability', 'transient_local']
            subscription_qos_profile = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE)
        else:
            # For an incompatible example, reverse the quality extremes
            # and expect no messages to arrive
            pub_extra_options = [
                '--qos-reliability', 'best_effort',
                '--qos-durability', 'volatile']
            subscription_qos_profile = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            expected_maximum_message_count = 0
            expected_minimum_message_count = 0

    process = subprocess.Popen(
        ['ros2', 'topic', 'pub'] + pub_extra_options + [topic, 'std_msgs/String', 'data: hello'],
        stdout=subprocess.PIPE,
        stdin=subprocess.DEVNULL)

    def shutdown():
        process.send_signal(signal.SIGINT)
        future.set_result(True)

    def message_callback(msg):
        """If we receive one message, the test has succeeded."""
        nonlocal received_message_count
        received_message_count += 1
        shutdown()

    subscription = node.create_subscription(
        String, topic, message_callback, subscription_qos_profile)
    assert subscription

    executor.spin_until_future_complete(future, timeout_sec=3)
    try:
        process.terminate()
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        process.kill()
        assert False, "CLI subprocess didn't shut down correctly"

    # Cleanup
    node.destroy_subscription(subscription)

    # Check results
    assert (
        received_message_count >= expected_minimum_message_count and
        received_message_count <= expected_maximum_message_count), \
        'Received {} messages from pub, which is not in expected range {}-{}'.format(
            received_message_count, expected_minimum_message_count, expected_maximum_message_count
        )


@pytest.mark.skipif(os.name == 'nt', reason='Subprocess bug in Windows Python 3.7.4')
@pytest.mark.timeout(60)
@pytest.mark.parametrize(
    'topic,provide_qos,compatible_qos', [
        ('/clitest/topic/echo_basic', False, True),
        ('/clitest/topic/echo_compatible_qos', True, True),
        ('/clitest/topic/echo_incompatible_qos', True, False)
    ]
)
def test_echo_basic(echo_pub_node, topic: str, provide_qos: bool, compatible_qos: bool):
    """Run a local publisher, check that `ros2 topic echo` receives at least one message."""
    # Check for inconsistent arguments
    assert provide_qos if not compatible_qos else True
    node, executor, context = echo_pub_node
    echo_extra_options = []
    publisher_qos_profile = 10
    if provide_qos:
        if compatible_qos:
            # For compatible test, put publisher at very high quality and subscription at low
            echo_extra_options = [
                '--qos-reliability', 'best_effort',
                '--qos-durability', 'volatile']
            publisher_qos_profile = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        else:
            # For an incompatible example, reverse the quality extremes
            # and expect no messages to arrive
            echo_extra_options = [
                '--qos-reliability', 'reliable',
                '--qos-durability', 'transient_local']
            publisher_qos_profile = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE)

    process = subprocess.Popen(
        ['ros2', 'topic', 'echo'] + echo_extra_options + [topic],
        stdout=subprocess.PIPE,
        stdin=subprocess.DEVNULL)

    publisher = node.create_publisher(String, topic, publisher_qos_profile)
    assert publisher

    def publish_message():
        publisher.publish(String(data='hello'))

    publish_timer = node.create_timer(0.5, publish_message)
    # The future won't complete - we will hit the timeout
    executor.spin_until_future_complete(rclpy.task.Future(), timeout_sec=4)
    # Note it is important to send SIGINT - terminate will make the stdout unavailable
    process.send_signal(signal.SIGINT)
    try:
        out, _ = process.communicate(timeout=0.5)
    except subprocess.TimeoutExpired:
        process.kill()
        assert False, "CLI subprocess didn't shut down correctly"

    # Cleanup
    node.destroy_timer(publish_timer)
    node.destroy_publisher(publisher)

    # Check results
    if compatible_qos:
        assert out, 'Echo CLI printed no output'
        lines = out.decode('utf-8').split('\n')
        assert 'data: hello' in lines, 'Echo CLI did not print expected message'
    else:
        assert not out, 'Echo CLI should not have received anything with incompatible QoS'
