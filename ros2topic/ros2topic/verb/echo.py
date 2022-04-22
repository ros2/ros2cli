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

import sys
from typing import Optional
from typing import TypeVar

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSPresetProfiles
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos_event import SubscriptionEventCallbacks
from rclpy.qos_event import UnsupportedEventTypeError
from rclpy.task import Future
from ros2cli.node.strategy import add_arguments as add_strategy_node_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2topic.api import get_msg_class
from ros2topic.api import qos_profile_from_short_keys
from ros2topic.api import TopicNameCompleter
from ros2topic.api import unsigned_int
from ros2topic.verb import VerbExtension
import rosidl_parser
from rosidl_runtime_py import message_to_csv
from rosidl_runtime_py import message_to_yaml
from rosidl_runtime_py.utilities import get_message

DEFAULT_TRUNCATE_LENGTH = 128
MsgType = TypeVar('MsgType')
default_profile_str = 'sensor_data'


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
        parser.add_argument(
            '--qos-profile',
            choices=rclpy.qos.QoSPresetProfiles.short_keys(),
            help='Quality of service preset profile to subscribe with (default: {})'
                 .format(default_profile_str))
        default_profile = rclpy.qos.QoSPresetProfiles.get_from_short_key(default_profile_str)
        parser.add_argument(
            '--qos-depth', metavar='N', type=int,
            help='Queue size setting to subscribe with '
                 '(overrides depth value of --qos-profile option)')
        parser.add_argument(
            '--qos-history',
            choices=rclpy.qos.QoSHistoryPolicy.short_keys(),
            help='History of samples setting to subscribe with '
                 '(overrides history value of --qos-profile option, default: {})'
                 .format(default_profile.history.short_key))
        parser.add_argument(
            '--qos-reliability',
            choices=rclpy.qos.QoSReliabilityPolicy.short_keys(),
            help='Quality of service reliability setting to subscribe with '
                 '(overrides reliability value of --qos-profile option, default: '
                 'Automatically match existing publishers )')
        parser.add_argument(
            '--qos-durability',
            choices=rclpy.qos.QoSDurabilityPolicy.short_keys(),
            help='Quality of service durability setting to subscribe with '
                 '(overrides durability value of --qos-profile option, default: '
                 'Automatically match existing publishers )')
        parser.add_argument(
            '--csv', action='store_true',
            help='Output all recursive fields separated by commas (e.g. for '
                 'plotting)')
        parser.add_argument(
            '--field', type=str, default=None,
            help='Echo a selected field of a message. '
                 "Use '.' to select sub-fields. "
                 'For example, to echo the position field of a nav_msgs/msg/Odometry message: '
                 "'ros2 topic echo /odom --field pose.pose.position'",
        )
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
            '--flow-style', action='store_true',
            help='Print collections in the block style (not available with csv format)')
        parser.add_argument(
            '--lost-messages', action='store_true', help='DEPRECATED: Does nothing')
        parser.add_argument(
            '--no-lost-messages', action='store_true', help="Don't report when a message is lost")
        parser.add_argument(
            '--raw', action='store_true', help='Echo the raw binary representation')
        parser.add_argument(
            '--filter', dest='filter_expr', help='Python expression to filter messages that '
                                                 'are printed. Expression can use Python builtins '
                                                 'as well as m (the message).')
        parser.add_argument(
            '--once', action='store_true', help='Print the first message received and then exit.')
        parser.add_argument(
            '--verbose', '-v', action='store_true',
            help='Shows the associated message info.')

    def choose_qos(self, node, args):

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

        pubs_info = node.get_publishers_info_by_topic(args.topic_name)
        publishers_count = len(pubs_info)
        if publishers_count == 0:
            return qos_profile

        for info in pubs_info:
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

    def main(self, *, args):

        if args.lost_messages:
            print(
                "WARNING: '--lost-messages' is deprecated; lost messages are reported by default",
                file=sys.stderr)

        # Select print function
        self.print_func = _print_yaml
        self.csv = args.csv
        if self.csv:
            self.print_func = _print_csv

        # Validate field selection
        self.field = args.field
        if self.field is not None:
            self.field = list(filter(None, self.field.split('.')))
            if not self.field:
                raise RuntimeError(f"Invalid field value '{args.field}'")

        self.truncate_length = args.truncate_length if not args.full_length else None
        self.no_arr = args.no_arr
        self.no_str = args.no_str
        self.flow_style = args.flow_style

        self.filter_fn = None
        if args.filter_expr:
            self.filter_fn = _expr_eval(args.filter_expr)

        self.future = None
        if args.once:
            self.future = Future()
        self.verbose = args.verbose

        with NodeStrategy(args) as node:

            qos_profile = self.choose_qos(node, args)

            if args.message_type is None:
                message_type = get_msg_class(
                    node, args.topic_name, include_hidden_topics=True)
            else:
                try:
                    message_type = get_message(args.message_type)
                except (AttributeError, ModuleNotFoundError, ValueError):
                    raise RuntimeError('The passed message type is invalid')

            if message_type is None:
                raise RuntimeError(
                    'Could not determine the type for the passed topic')

            self.subscribe_and_spin(
                node,
                args.topic_name,
                message_type,
                qos_profile,
                args.no_lost_messages,
                args.raw)

    def subscribe_and_spin(
        self,
        node: Node,
        topic_name: str,
        message_type: MsgType,
        qos_profile: QoSProfile,
        no_report_lost_messages: bool,
        raw: bool,
    ) -> Optional[str]:
        """Initialize a node with a single subscription and spin."""
        event_callbacks = None
        if not no_report_lost_messages:
            event_callbacks = SubscriptionEventCallbacks(
                message_lost=_message_lost_event_callback)
        try:
            node.create_subscription(
                message_type,
                topic_name,
                self._subscriber_callback,
                qos_profile,
                event_callbacks=event_callbacks,
                raw=raw)
        except UnsupportedEventTypeError:
            assert not no_report_lost_messages
            node.create_subscription(
                message_type,
                topic_name,
                self._subscriber_callback,
                qos_profile,
                event_callbacks=None,
                raw=raw)

        if self.future is not None:
            rclpy.spin_until_future_complete(node, self.future)
        else:
            rclpy.spin(node)

    def _subscriber_callback(self, msg, info):
        submsg = msg
        if self.field is not None:
            for field in self.field:
                try:
                    submsg = getattr(submsg, field)
                except AttributeError as ex:
                    raise RuntimeError(f"Invalid field '{'.'.join(self.field)}': {ex}")

        # Evaluate the current msg against the supplied expression
        if self.filter_fn is not None and not self.filter_fn(submsg):
            return

        if self.future is not None:
            self.future.set_result(True)

        if self.verbose:
            if hasattr(msg, '__slots__'):
                submsg = MessageWrapperWithInfo(submsg, info)
            else:
                print('---Got new message, message info:---')
                print(info)
                print('---Message data:---')
        if self.csv:
            self.print_func(submsg, self.truncate_length, self.no_arr, self.no_str)
        else:
            self.print_func(
                submsg, self.truncate_length, self.no_arr, self.no_str, self.flow_style)


def _expr_eval(expr):
    def eval_fn(m):
        return eval(expr)
    return eval_fn


class MessageInfo:

    __slots__ = [
        '_source_timestamp',
        '_received_timestamp',
        '_publication_sequence_number',
        '_reception_sequence_number',
    ]
    SLOT_TYPES = [
        int,
        int,
        int,
        int,
    ]

    def __init__(self, message_info_dict):
        for slot in self.__slots__:
            value = message_info_dict[slot[1:]]
            if value is None:
                self.__slots__.remove(slot)
            else:
                setattr(self, slot, value)


class MessageWrapperWithInfo:

    __slots__ = ['_message_info', '_message']
    SLOT_TYPES = [
        rosidl_parser.definition.NamespacedType(['doesntmatter', 'msg'], 'neitherthismatter'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['doesntmatter', 'msg'], 'neitherthismatter'),  # noqa: E501
    ]

    def __init__(self, msg, info):
        self._message = msg
        self._message_info = MessageInfo(info)


def _print_yaml(msg, truncate_length, noarr, nostr, flowstyle):
    if hasattr(msg, '__slots__'):
        print(
            message_to_yaml(
                msg, truncate_length=truncate_length,
                no_arr=noarr, no_str=nostr, flow_style=flowstyle),
            end='---\n')
    else:
        print(msg, end='\n---\n')


def _print_csv(msg, truncate_length, noarr, nostr):
    if hasattr(msg, '__slots__'):
        print(message_to_csv(msg, truncate_length=truncate_length, no_arr=noarr, no_str=nostr))
    else:
        print(msg)


def _message_lost_event_callback(message_lost_status):
    print(
        'A message was lost!!!\n\ttotal count change:'
        f'{message_lost_status.total_count_change}'
        f'\n\ttotal count: {message_lost_status.total_count}',
        end='---\n'
    )
