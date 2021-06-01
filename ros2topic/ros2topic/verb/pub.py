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

import time
from typing import Optional
from typing import TypeVar

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ros2cli.node.direct import DirectNode
from ros2topic.api import qos_profile_from_short_keys
from ros2topic.api import TopicMessagePrototypeCompleter
from ros2topic.api import TopicNameCompleter
from ros2topic.api import TopicTypeCompleter
from ros2topic.verb import VerbExtension
from rosidl_runtime_py import set_message_fields
from rosidl_runtime_py.utilities import get_message
import yaml

MsgType = TypeVar('MsgType')
default_profile_str = 'system_default'


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
            '--keep-alive', metavar='N', type=positive_float, default=0.1,
            help='Keep publishing node alive for N seconds after the last msg '
                 '(default: 0.1)')
        parser.add_argument(
            '-n', '--node-name',
            help='Name of the created publishing node')
        parser.add_argument(
            '--qos-profile',
            choices=rclpy.qos.QoSPresetProfiles.short_keys(),
            default=default_profile_str,
            help='Quality of service preset profile to {} with (default: {})'
                 .format('publish', default_profile_str))
        default_profile = rclpy.qos.QoSPresetProfiles.get_from_short_key(
            default_profile_str)
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

    def main(self, *, args):
        return main(args)


def main(args):

    qos_profile_name = args.qos_profile
    if not qos_profile_name:
        qos_profile_name = default_profile_str

    qos_profile = qos_profile_from_short_keys(
        qos_profile_name, reliability=args.qos_reliability, durability=args.qos_durability,
        depth=args.qos_depth, history=args.qos_history)

    times = args.times
    if args.once:
        times = 1
    with DirectNode(args, node_name=args.node_name) as node:
        qos_profile = choose_qos(node, args)
        return publisher(
            node.node,
            args.message_type,
            args.topic_name,
            args.values,
            1. / args.rate,
            args.print,
            times,
            qos_profile,
            args.keep_alive)


def choose_qos(node, args):

    if (args.qos_profile is not None or
        args.qos_reliability is not None or
        args.qos_durability is not None or
        args.qos_depth is not None or
        args.qos_history is not None):

        if args.qos_profile is None:
            args.qos_profile = default_profile_str
        return qos_profile_from_short_keys(args.qos_profile,
                                           reliability=args.qos_reliability,
                                           durability=args.qos_durability,
                                           depth=args.qos_depth,
                                           history=args.qos_history)

    qos_profile = QoSPresetProfiles.get_from_short_key(default_profile_str)
    reliability_reliable_endpoints_count = 0
    durability_transient_local_endpoints_count = 0

    subs_info = node.get_subscriptions_info_by_topic(args.topic_name)
    subscribers_count = len(pubs_info)
    if subscribers_count == 0:
        return qos_profile

    for info in subs_info:
        if (info.qos_profile.reliability == QoSReliabilityPolicy.RELIABLE):
            reliability_reliable_endpoints_count += 1
        if (info.qos_profile.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL):
            durability_transient_local_endpoints_count += 1

    # If all endpoints are reliable, ask for reliable
    if reliability_reliable_endpoints_count == publishers_count:
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
    else:
        if reliability_reliable_endpoints_count > 0:
            print(
                'Some, but not all, publishers are offering '
                'QoSReliabilityPolicy.RELIABLE. Falling back to '
                'QoSReliabilityPolicy.BEST_EFFORT as it will connect '
                'to all publishers'
            )
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

    # If all endpoints are transient_local, ask for transient_local
    if durability_transient_local_endpoints_count == publishers_count:
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    else:
        if durability_transient_local_endpoints_count > 0:
            print(
                'Some, but not all, publishers are offering '
                'QoSDurabilityPolicy.TRANSIENT_LOCAL. Falling back to '
                'QoSDurabilityPolicy.VOLATILE as it will connect '
                'to all publishers'
            )
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

    return qos_profile


def publisher(
    node: Node,
    message_type: MsgType,
    topic_name: str,
    values: dict,
    period: float,
    print_nth: int,
    times: int,
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

    msg = msg_module()
    try:
        set_message_fields(msg, values_dictionary)
    except Exception as e:
        return 'Failed to populate field: {0}'.format(e)

    print('publisher: beginning loop')
    count = 0

    def timer_callback():
        nonlocal count
        count += 1
        if print_nth and count % print_nth == 0:
            print('publishing #%d: %r\n' % (count, msg))
        pub.publish(msg)

    timer = node.create_timer(period, timer_callback)
    while times == 0 or count < times:
        rclpy.spin_once(node)

    # give some time for the messages to reach the wire before exiting
    time.sleep(keep_alive)

    node.destroy_timer(timer)
