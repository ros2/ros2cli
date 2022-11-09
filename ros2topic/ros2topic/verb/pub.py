# Copyright 2016-2017 Open Source Robotics Foundation, Inc.
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

import array
from functools import partial
import time
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import TypeVar

import numpy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from ros2cli.node.direct import add_arguments as add_direct_node_arguments
from ros2cli.node.direct import DirectNode
from ros2topic.api import profile_configure_short_keys
from ros2topic.api import TopicMessagePrototypeCompleter
from ros2topic.api import TopicNameCompleter
from ros2topic.api import TopicTypeCompleter
from ros2topic.verb import VerbExtension
from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import NamespacedType
from rosidl_runtime_py.convert import get_message_slot_types
from rosidl_runtime_py.import_message import import_message_from_namespaced_type
from rosidl_runtime_py.utilities import get_message
import yaml

MsgType = TypeVar('MsgType')


def set_message_fields_expanded(
        msg: Any, values: Dict[str, str], expand_header_auto: bool = False,
        expand_time_now: bool = False) -> List[Any]:
    """
    Set the fields of a ROS message.

    :param msg: The ROS message to populate.
    :param values: The values to set in the ROS message. The keys of the dictionary represent
        fields of the message.
    :param expand_header_auto: If enabled and 'auto' is passed as a value to a
        'std_msgs.msg.Header' field, an empty Header will be instantiated and a setter function
        will be returned so that its 'stamp' field can be set to the current time.
    :param expand_time_now: If enabled and 'now' is passed as a value to a
        'builtin_interfaces.msg.Time' field, a setter function will be returned so that
        its value can be set to the current time.
    :returns: A list of setter functions that can be used to update 'builtin_interfaces.msg.Time'
        fields, useful for setting them to the current time. The list will be empty if the message
        does not have any 'builtin_interfaces.msg.Time' fields, or if expand_header_auto and
        expand_time_now are false.
    :raises AttributeError: If the message does not have a field provided in the input dictionary.
    :raises TypeError: If a message value does not match its field type.
    """
    timestamp_fields = []

    def set_message_fields_expanded_internal(
            msg: Any, values: Dict[str, str],
            timestamp_fields: List[Any]) -> List[Any]:
        try:
            items = values.items()
        except AttributeError:
            raise TypeError(
                "Value '%s' is expected to be a dictionary but is a %s" %
                (values, type(values).__name__))
        for field_name, field_value in items:
            field = getattr(msg, field_name)
            field_type = type(field)
            qualified_class_name = '{}.{}'.format(field_type.__module__, field_type.__name__)
            if field_type is array.array:
                value = field_type(field.typecode, field_value)
            elif field_type is numpy.ndarray:
                value = numpy.array(field_value, dtype=field.dtype)
            elif type(field_value) is field_type:
                value = field_value
            # We can't import these types directly, so we use the qualified class name to
            # distinguish them from other fields
            elif qualified_class_name == 'std_msgs.msg._header.Header' and \
                    field_value == 'auto' and expand_header_auto:
                timestamp_fields.append(partial(setattr, field, 'stamp'))
                continue
            elif qualified_class_name == 'builtin_interfaces.msg._time.Time' and \
                    field_value == 'now' and expand_time_now:
                timestamp_fields.append(partial(setattr, msg, field_name))
                continue
            else:
                try:
                    value = field_type(field_value)
                except TypeError:
                    value = field_type()
                    set_message_fields_expanded_internal(
                        value, field_value, timestamp_fields)
            rosidl_type = get_message_slot_types(msg)[field_name]
            # Check if field is an array of ROS messages
            if isinstance(rosidl_type, AbstractNestedType):
                if isinstance(rosidl_type.value_type, NamespacedType):
                    field_elem_type = import_message_from_namespaced_type(rosidl_type.value_type)
                    for n in range(len(value)):
                        submsg = field_elem_type()
                        set_message_fields_expanded_internal(
                            submsg, value[n], timestamp_fields)
                        value[n] = submsg
            setattr(msg, field_name, value)
    set_message_fields_expanded_internal(msg, values, timestamp_fields)
    return timestamp_fields


def nonnegative_int(inval):
    ret = int(inval)
    if ret < 0:
        # The error message here gets completely swallowed by argparse
        raise ValueError('Value must be positive or zero')
    return ret


def positive_float(inval):
    ret = float(inval)
    if ret <= 0.0:
        # The error message here gets completely swallowed by argparse
        raise ValueError('Value must be positive')
    return ret


def get_pub_qos_profile():
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        depth=1)


class PubVerb(VerbExtension):
    """Publish a message to a topic."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'topic_name',
            help="Name of the ROS topic to publish to (e.g. '/chatter')")
        arg.completer = TopicNameCompleter(
            include_hidden_topics_key='include_hidden_topics')
        arg = parser.add_argument(
            'message_type',
            help="Type of the ROS message (e.g. 'std_msgs/String')")
        arg.completer = TopicTypeCompleter(
            topic_name_key='topic_name')
        arg = parser.add_argument(
            'values', nargs='?', default='{}',
            help='Values to fill the message with in YAML format '
                 "(e.g. 'data: Hello World'), "
                 'otherwise the message will be published with default values')
        arg.completer = TopicMessagePrototypeCompleter(
            topic_type_key='message_type')
        parser.add_argument(
            '-r', '--rate', metavar='N', type=positive_float, default=1.0,
            help='Publishing rate in Hz (default: 1)')
        parser.add_argument(
            '-p', '--print', metavar='N', type=int, default=1,
            help='Only print every N-th published message (default: 1)')
        group = parser.add_mutually_exclusive_group()
        group.add_argument(
            '-1', '--once', action='store_true',
            help='Publish one message and exit')
        group.add_argument(
            '-t', '--times', type=nonnegative_int, default=0,
            help='Publish this number of times and then exit')
        parser.add_argument(
            '-w', '--wait-matching-subscriptions', type=nonnegative_int, default=None,
            help=(
                'Wait until finding the specified number of matching subscriptions. '
                'Defaults to 1 when using "-1"/"--once"/"--times", otherwise defaults to 0.'))
        parser.add_argument(
            '--keep-alive', metavar='N', type=positive_float, default=0.1,
            help='Keep publishing node alive for N seconds after the last msg '
                 '(default: 0.1)')
        parser.add_argument(
            '-n', '--node-name',
            help='Name of the created publishing node')
        parser.add_argument(
            '--qos-profile',
            choices=rclpy.qos.QoSPresetProfiles.short_keys(),
            help='Quality of service preset profile to publish)')
        default_profile = get_pub_qos_profile()
        parser.add_argument(
            '--qos-depth', metavar='N', type=int, default=-1,
            help='Queue size setting to publish with '
                 '(overrides depth value of --qos-profile option)')
        parser.add_argument(
            '--qos-history',
            choices=rclpy.qos.QoSHistoryPolicy.short_keys(),
            help='History of samples setting to publish with '
                 '(overrides history value of --qos-profile option, default: {})'
                 .format(default_profile.history.short_key))
        parser.add_argument(
            '--qos-reliability',
            choices=rclpy.qos.QoSReliabilityPolicy.short_keys(),
            help='Quality of service reliability setting to publish with '
                 '(overrides reliability value of --qos-profile option, default: {})'
                 .format(default_profile.reliability.short_key))
        parser.add_argument(
            '--qos-durability',
            choices=rclpy.qos.QoSDurabilityPolicy.short_keys(),
            help='Quality of service durability setting to publish with '
                 '(overrides durability value of --qos-profile option, default: {})'
                 .format(default_profile.durability.short_key))
        add_direct_node_arguments(parser)

    def main(self, *, args):
        return main(args)


def main(args):
    qos_profile = get_pub_qos_profile()

    qos_profile_name = args.qos_profile
    if qos_profile_name:
        qos_profile = rclpy.qos.QoSPresetProfiles.get_from_short_key(qos_profile_name)
    profile_configure_short_keys(
        qos_profile, args.qos_reliability, args.qos_durability,
        args.qos_depth, args.qos_history)

    times = args.times
    if args.once:
        times = 1

    with DirectNode(args, node_name=args.node_name) as node:
        return publisher(
            node.node,
            args.message_type,
            args.topic_name,
            args.values,
            1. / args.rate,
            args.print,
            times,
            args.wait_matching_subscriptions
            if args.wait_matching_subscriptions is not None else int(times != 0),
            qos_profile,
            args.keep_alive)


def publisher(
    node: Node,
    message_type: MsgType,
    topic_name: str,
    values: dict,
    period: float,
    print_nth: int,
    times: int,
    wait_matching_subscriptions: int,
    qos_profile: QoSProfile,
    keep_alive: float,
) -> Optional[str]:
    """Initialize a node with a single publisher and run its publish loop (maybe only once)."""
    try:
        msg_module = get_message(message_type)
    except (AttributeError, ModuleNotFoundError, ValueError):
        raise RuntimeError('The passed message type is invalid')
    values_dictionary = yaml.safe_load(values)
    if not isinstance(values_dictionary, dict):
        return 'The passed value needs to be a dictionary in YAML format'

    pub = node.create_publisher(msg_module, topic_name, qos_profile)

    times_since_last_log = 0
    while pub.get_subscription_count() < wait_matching_subscriptions:
        # Print a message reporting we're waiting each 1s, check condition each 100ms.
        if not times_since_last_log:
            print(
                f'Waiting for at least {wait_matching_subscriptions} matching subscription(s)...')
        times_since_last_log = (times_since_last_log + 1) % 10
        time.sleep(0.1)

    msg = msg_module()
    try:
        timestamp_fields = set_message_fields_expanded(
            msg, values_dictionary, expand_header_auto=True, expand_time_now=True)
    except Exception as e:
        return 'Failed to populate field: {0}'.format(e)
    print('publisher: beginning loop')
    count = 0

    def timer_callback():
        stamp_now = node.get_clock().now().to_msg()
        for field_setter in timestamp_fields:
            field_setter(stamp_now)
        nonlocal count
        count += 1
        if print_nth and count % print_nth == 0:
            print('publishing #%d: %r\n' % (count, msg))
        pub.publish(msg)

    timer_callback()
    if times != 1:
        timer = node.create_timer(period, timer_callback)
        while times == 0 or count < times:
            rclpy.spin_once(node)
        # give some time for the messages to reach the wire before exiting
        time.sleep(keep_alive)
        node.destroy_timer(timer)
    else:
        # give some time for the messages to reach the wire before exiting
        time.sleep(keep_alive)
