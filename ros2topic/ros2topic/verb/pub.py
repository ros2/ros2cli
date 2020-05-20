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
from ros2topic.api import add_qos_arguments_to_argument_parser
from ros2topic.api import qos_profile_from_short_keys
from ros2topic.api import TopicMessagePrototypeCompleter
from ros2topic.api import TopicNameCompleter
from ros2topic.api import TopicTypeCompleter
from ros2topic.verb import VerbExtension
from rosidl_runtime_py import set_message_fields
from rosidl_runtime_py.utilities import get_message
import yaml

MsgType = TypeVar('MsgType')


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
            '-n', '--node-name',
            help='Name of the created publishing node')
        add_qos_arguments_to_argument_parser(
            parser, is_publisher=True, default_preset='system_default')

    def main(self, *, args):
        return main(args)


def main(args):
    qos_profile = qos_profile_from_short_keys(
        args.qos_profile, reliability=args.qos_reliability, durability=args.qos_durability)
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
            qos_profile)


def publisher(
    node: Node,
    message_type: MsgType,
    topic_name: str,
    values: dict,
    period: float,
    print_nth: int,
    times: int,
    qos_profile: QoSProfile,
) -> Optional[str]:
    """Initialize a node with a single publisher and run its publish loop (maybe only once)."""
    msg_module = get_message(message_type)
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

    if times == 1:
        time.sleep(0.1)  # make sure the message reaches the wire before exiting

    node.destroy_timer(timer)
