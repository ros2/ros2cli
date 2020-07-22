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

from typing import Any
from typing import Callable
from typing import Optional
from typing import TypeVar

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos_event import SubscriptionEventCallbacks
from rclpy.qos_event import UnsupportedEventTypeError
from rclpy.utilities import get_rmw_implementation_identifier
from ros2cli.node.strategy import add_arguments as add_strategy_node_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2topic.api import add_qos_arguments_to_argument_parser
from ros2topic.api import get_msg_class
from ros2topic.api import qos_profile_from_short_keys
from ros2topic.api import TopicNameCompleter
from ros2topic.api import unsigned_int
from ros2topic.verb import VerbExtension
from rosidl_runtime_py import message_to_csv
from rosidl_runtime_py import message_to_yaml
from rosidl_runtime_py.utilities import get_message

DEFAULT_TRUNCATE_LENGTH = 128
MsgType = TypeVar('MsgType')


class EchoVerb(VerbExtension):
    """Output messages from a topic."""

    def add_arguments(self, parser, cli_name):
        add_strategy_node_arguments(parser)

        arg = parser.add_argument(
            'topic_name',
            help="Name of the ROS topic to listen to (e.g. '/chatter')")
        arg.completer = TopicNameCompleter(
            include_hidden_topics_key='include_hidden_topics')
        parser.add_argument(
            'message_type', nargs='?',
            help="Type of the ROS message (e.g. 'std_msgs/msg/String')")
        add_qos_arguments_to_argument_parser(
            parser, is_publisher=False, default_preset='sensor_data')
        parser.add_argument(
            '--csv', action='store_true',
            help='Output all recursive fields separated by commas (e.g. for '
                 'plotting)')
        parser.add_argument(
            '--full-length', '-f', action='store_true',
            help='Output all elements for arrays, bytes, and string with a '
                 "length > '--truncate-length', by default they are truncated "
                 "after '--truncate-length' elements with '...''")
        parser.add_argument(
            '--truncate-length', '-l', type=unsigned_int, default=DEFAULT_TRUNCATE_LENGTH,
            help='The length to truncate arrays, bytes, and string to '
                 '(default: %d)' % DEFAULT_TRUNCATE_LENGTH)
        parser.add_argument(
            '--no-arr', action='store_true', help="Don't print array fields of messages")
        parser.add_argument(
            '--no-str', action='store_true', help="Don't print string fields of messages")
        parser.add_argument(
            '--lost-messages', action='store_true', help='Report when a message is lost')
        parser.add_argument(
            '--raw', action='store_true', help='Echo the raw binary representation')

    def main(self, *, args):
        return main(args)


def main(args):
    if not args.csv:
        truncate_length = args.truncate_length if not args.full_length else None
        callback = subscriber_cb(truncate_length, args.no_arr, args.no_str)
    else:
        truncate_length = args.truncate_length if not args.full_length else None
        callback = subscriber_cb_csv(truncate_length, args.no_arr, args.no_str)
    qos_profile = qos_profile_from_short_keys(
        args.qos_profile, reliability=args.qos_reliability, durability=args.qos_durability,
        depth=args.qos_depth, history=args.qos_history)
    with NodeStrategy(args) as node:
        if args.message_type is None:
            message_type = get_msg_class(
                node, args.topic_name, include_hidden_topics=True)
        else:
            message_type = get_message(args.message_type)

        if message_type is None:
            raise RuntimeError(
                'Could not determine the type for the passed topic')

        subscriber(
            node, args.topic_name, message_type, callback, qos_profile, args.lost_messages, args.raw)


def subscriber(
    node: Node,
    topic_name: str,
    message_type: MsgType,
    callback: Callable[[MsgType], Any],
    qos_profile: QoSProfile,
    report_lost_messages: bool,
    raw: bool
) -> Optional[str]:
    """Initialize a node with a single subscription and spin."""
    event_callbacks = None
    if report_lost_messages:
        event_callbacks = SubscriptionEventCallbacks(
            message_lost=message_lost_event_callback)
    try:
        node.create_subscription(
            message_type, topic_name, callback, qos_profile, event_callbacks=event_callbacks, raw=raw)
    except UnsupportedEventTypeError:
        assert report_lost_messages
        print(
            f"The rmw implementation '{get_rmw_implementation_identifier()}'"
            ' does not support reporting lost messages'
        )
    rclpy.spin(node)


def subscriber_cb(truncate_length, noarr, nostr):
    def cb(msg):
        nonlocal truncate_length, noarr, nostr
        if isinstance(msg, bytes):
            print(msg, end='\n---\n')
        else:
            print(
                message_to_yaml(
                    msg, truncate_length=truncate_length, no_arr=noarr, no_str=nostr),
                end='---\n')
    return cb


def subscriber_cb_csv(truncate_length, noarr, nostr):
    def cb(msg):
        nonlocal truncate_length, noarr, nostr
        if isinstance(msg, bytes):
            print(msg)
        else:
            print(message_to_csv(msg, truncate_length=truncate_length, no_arr=noarr, no_str=nostr))
    return cb


def message_lost_event_callback(message_lost_status):
    print(
        'A message was lost!!!\n\ttotal count change:'
        f'{message_lost_status.total_count_change}'
        f'\n\ttotal count: {message_lost_status.total_count}',
        end='---\n'
    )
