# Copyright 2020 Open Source Robotics Foundation, Inc.
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

import time

import pytest

import rclpy
import rclpy.action

from ros2cli.node.daemon import DaemonNode
from ros2cli.node.daemon import is_daemon_running
from ros2cli.node.daemon import shutdown_daemon
from ros2cli.node.daemon import spawn_daemon

import test_msgs.action
import test_msgs.msg
import test_msgs.srv


TEST_NODE_NAME = 'test_node'
TEST_NODE_NAMESPACE = '/test'
TEST_NODE_ENCLAVE = '/'
TEST_TOPIC_NAME = '/test/topic'
TEST_TOPIC_TYPE = 'test_msgs/msg/Empty'
TEST_TOPIC_PUBLISHER_QOS = rclpy.qos.QoSProfile(
    reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
    durability=rclpy.qos.DurabilityPolicy.VOLATILE,
    depth=1
)
TEST_TOPIC_SUBSCRIPTION_QOS = rclpy.qos.QoSProfile(
    reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
    durability=rclpy.qos.DurabilityPolicy.VOLATILE,
    depth=1
)
TEST_SERVICE_NAME = '/test/service'
TEST_SERVICE_TYPE = 'test_msgs/srv/Empty'
TEST_ACTION_NAME = '/test/action'
TEST_ACTION_TYPE = 'test_msgs/action/Fibonacci'


@pytest.fixture(autouse=True, scope='module')
def local_node():
    rclpy.init()
    node = rclpy.create_node(
        node_name=TEST_NODE_NAME, namespace=TEST_NODE_NAMESPACE
    )
    publisher = node.create_publisher(
        msg_type=test_msgs.msg.Empty,
        topic=TEST_TOPIC_NAME,
        qos_profile=TEST_TOPIC_PUBLISHER_QOS
    )
    subscription = node.create_subscription(
        msg_type=test_msgs.msg.Empty,
        topic=TEST_TOPIC_NAME,
        callback=(lambda msg: None),
        qos_profile=TEST_TOPIC_SUBSCRIPTION_QOS
    )
    service = node.create_service(
        srv_type=test_msgs.srv.Empty,
        srv_name=TEST_SERVICE_NAME,
        callback=(lambda req, res: res)
    )
    client = node.create_client(
        srv_type=test_msgs.srv.Empty,
        srv_name=TEST_SERVICE_NAME
    )

    def noop_execute_callback(goal_handle):
        goal_handle.succeed()
        return test_msgs.action.Fibonacci.Result()

    action_server = rclpy.action.ActionServer(
        node=node,
        action_type=test_msgs.action.Fibonacci,
        action_name=TEST_ACTION_NAME,
        execute_callback=noop_execute_callback
    )
    action_client = rclpy.action.ActionClient(
        node=node,
        action_type=test_msgs.action.Fibonacci,
        action_name=TEST_ACTION_NAME
    )
    try:
        yield node
    finally:
        action_client.destroy()
        action_server.destroy()
        node.destroy_client(client)
        node.destroy_service(service)
        node.destroy_subscription(subscription)
        node.destroy_publisher(publisher)
        node.destroy_node()
        rclpy.shutdown()


@pytest.fixture(scope='module')
def daemon_node():
    if is_daemon_running(args=[]):
        assert shutdown_daemon(args=[], timeout=5.0)
    assert spawn_daemon(args=[], timeout=5.0)
    with DaemonNode(args=[]) as node:
        if not node.connected:
            pytest.fail(
                f'failed to connect daemon {TEST_NODE_NAMESPACE}/{TEST_NODE_NAME}'
            )
        attempts = 3
        delay_between_attempts = 2  # seconds
        for _ in range(attempts):
            node_names_and_namespaces = node.get_node_names_and_namespaces()
            if [TEST_NODE_NAME, TEST_NODE_NAMESPACE] in node_names_and_namespaces:
                break
            time.sleep(delay_between_attempts)
        else:
            pytest.fail(
                f'daemon failed to discover {TEST_NODE_NAMESPACE}/{TEST_NODE_NAME}'
            )
        yield node
        node.system.shutdown()


def test_get_name(daemon_node):
    assert 'daemon' in daemon_node.get_name()


def test_get_namespace(daemon_node):
    assert '/' == daemon_node.get_namespace()


def test_get_node_names_and_namespaces(daemon_node):
    node_names_and_namespaces = daemon_node.get_node_names_and_namespaces()
    assert [TEST_NODE_NAME, TEST_NODE_NAMESPACE] in node_names_and_namespaces


def test_get_node_names_and_namespaces_with_enclaves(daemon_node):
    node_names_and_namespaces = daemon_node.get_node_names_and_namespaces_with_enclaves()
    assert [TEST_NODE_NAME, TEST_NODE_NAMESPACE, TEST_NODE_ENCLAVE] in node_names_and_namespaces


def test_get_topic_names_and_types(daemon_node):
    topic_names_and_types = daemon_node.get_topic_names_and_types()
    assert [TEST_TOPIC_NAME, [TEST_TOPIC_TYPE]] in topic_names_and_types


def test_get_service_names_and_types(daemon_node):
    service_names_and_types = daemon_node.get_service_names_and_types()
    assert [TEST_SERVICE_NAME, [TEST_SERVICE_TYPE]] in service_names_and_types


def test_get_action_names_and_types(daemon_node):
    action_names_and_types = daemon_node.get_action_names_and_types()
    assert [TEST_ACTION_NAME, [TEST_ACTION_TYPE]] in action_names_and_types


def test_get_publisher_names_and_types_by_node(daemon_node):
    topic_names_and_types = daemon_node.get_publisher_names_and_types_by_node(
        TEST_NODE_NAME, TEST_NODE_NAMESPACE
    )
    assert [TEST_TOPIC_NAME, [TEST_TOPIC_TYPE]] in topic_names_and_types


def test_get_subscriber_names_and_types_by_node(daemon_node):
    topic_names_and_types = daemon_node.get_subscriber_names_and_types_by_node(
        TEST_NODE_NAME, TEST_NODE_NAMESPACE
    )
    assert [TEST_TOPIC_NAME, [TEST_TOPIC_TYPE]] in topic_names_and_types


def test_get_service_names_and_types_by_node(daemon_node):
    service_names_and_types = daemon_node.get_service_names_and_types_by_node(
        TEST_NODE_NAME, TEST_NODE_NAMESPACE
    )
    assert [TEST_SERVICE_NAME, [TEST_SERVICE_TYPE]] in service_names_and_types


def test_get_client_names_and_types_by_node(daemon_node):
    service_names_and_types = daemon_node.get_client_names_and_types_by_node(
        TEST_NODE_NAME, TEST_NODE_NAMESPACE
    )
    assert [TEST_SERVICE_NAME, [TEST_SERVICE_TYPE]] in service_names_and_types


def test_get_action_server_names_and_types_by_node(daemon_node):
    action_names_and_types = daemon_node.get_action_server_names_and_types_by_node(
        TEST_NODE_NAME, TEST_NODE_NAMESPACE
    )
    assert [TEST_ACTION_NAME, [TEST_ACTION_TYPE]] in action_names_and_types


def test_get_action_client_names_and_types_by_node(daemon_node):
    action_names_and_types = daemon_node.get_action_client_names_and_types_by_node(
        TEST_NODE_NAME, TEST_NODE_NAMESPACE
    )
    assert [TEST_ACTION_NAME, [TEST_ACTION_TYPE]] in action_names_and_types


def test_get_publishers_info_by_topic(daemon_node):
    publishers_info = daemon_node.get_publishers_info_by_topic(TEST_TOPIC_NAME)
    assert len(publishers_info) == 1
    test_publisher_info = publishers_info[0]
    assert test_publisher_info.node_name == TEST_NODE_NAME
    assert test_publisher_info.node_namespace == TEST_NODE_NAMESPACE
    assert test_publisher_info.topic_type == TEST_TOPIC_TYPE
    assert test_publisher_info.qos_profile.durability == \
        TEST_TOPIC_PUBLISHER_QOS.durability
    assert test_publisher_info.qos_profile.reliability == \
        TEST_TOPIC_PUBLISHER_QOS.reliability


def test_get_subscriptions_info_by_topic(daemon_node):
    subscriptions_info = daemon_node.get_subscriptions_info_by_topic(TEST_TOPIC_NAME)
    assert len(subscriptions_info) == 1
    test_subscription_info = subscriptions_info[0]
    assert test_subscription_info.node_name == TEST_NODE_NAME
    assert test_subscription_info.node_namespace == TEST_NODE_NAMESPACE
    assert test_subscription_info.topic_type == TEST_TOPIC_TYPE
    assert test_subscription_info.qos_profile.durability == \
        TEST_TOPIC_SUBSCRIPTION_QOS.durability
    assert test_subscription_info.qos_profile.reliability == \
        TEST_TOPIC_SUBSCRIPTION_QOS.reliability


def test_count_publishers(daemon_node):
    assert 1 == daemon_node.count_publishers(TEST_TOPIC_NAME)


def test_count_subscribers(daemon_node):
    assert 1 == daemon_node.count_subscribers(TEST_TOPIC_NAME)


def test_count_clients(daemon_node):
    assert 1 == daemon_node.count_clients(TEST_SERVICE_NAME)


def test_count_services(daemon_node):
    assert 1 == daemon_node.count_services(TEST_SERVICE_NAME)