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
from ros2cli.node.direct import add_arguments as add_direct_node_arguments
from ros2cli.node.direct import DirectNode
from ros2topic.api import add_qos_arguments
from ros2topic.api import positive_float
from ros2topic.api import profile_configure_short_keys
from ros2topic.api import TopicMessagePrototypeCompleter
from ros2topic.api import TopicNameCompleter
from ros2topic.api import TopicTypeCompleter
from ros2topic.verb import VerbExtension
from rosidl_runtime_py import set_message_fields
from rosidl_runtime_py.utilities import get_message
import yaml

MsgType = TypeVar('MsgType')
DEFAULT_WAIT_TIME = 0.1


def nonnegative_int(inval):
    ret = int(inval)
    if ret < 0:
        # The error message here gets completely swallowed by argparse
        raise ValueError('Value must be positive or zero')
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
            '-w', '--wait-matching-subscriptions', type=nonnegative_int, default=None,
            help=(
                'Wait until finding the specified number of matching subscriptions. '
                'Defaults to 1 when using "-1"/"--once"/"--times", otherwise defaults to 0.'))
        parser.add_argument(
            '--max-wait-time-secs', type=positive_float, default=None,
            help=(
                'This sets the maximum wait time in seconds if --wait-until-matching-subscriptions is set.'
                'By default, this flag is not set meaning the subscriber will wait endlessly.'))
        parser.add_argument(
            '--keep-alive', metavar='N', type=positive_float, default=0.1,
            help='Keep publishing node alive for N seconds after the last msg '
                 '(default: 0.1)')
        parser.add_argument(
            '-n', '--node-name',
            help='Name of the created publishing node')
        add_qos_arguments(parser, 'publish', 'default')
        add_direct_node_arguments(parser)

    def main(self, *, args):
        return main(args)


def main(args):
    qos_profile_name = args.qos_profile
    qos_profile = rclpy.qos.QoSPresetProfiles.get_from_short_key(qos_profile_name)
    profile_configure_short_keys(
        qos_profile, args.qos_reliability, args.qos_durability,
        args.qos_depth, args.qos_history, args.qos_liveliness,
        args.qos_liveliness_lease_duration_seconds)

    times = args.times
    if args.once:
        times = 1

    if args.wait_matching_subscriptions is None and args.max_wait_time_secs is not None:
        return '--max-wait-time-secs option is only effective with --wait-matching-subscriptions'

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
            args.max_wait_time_secs,
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
    max_wait_time: float | None,
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
    total_wait_time = 0
    while pub.get_subscription_count() < wait_matching_subscriptions:
        # Print a message reporting we're waiting each 1s, check condition each 100ms.
        if not times_since_last_log:
            print(
                f'Waiting for at least {wait_matching_subscriptions} matching subscription(s)...')
        if max_wait_time is not None and max_wait_time <= total_wait_time:
            return f'Timed out waiting for subscribers: Expected {wait_matching_subscriptions} subcribers but only got {pub.get_subscription_count()} subscribers'       
        times_since_last_log = (times_since_last_log + 1) % 10
        time.sleep(DEFAULT_WAIT_TIME)
        total_wait_time += DEFAULT_WAIT_TIME
        
    msg = msg_module()
    try:
        timestamp_fields = set_message_fields(
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
