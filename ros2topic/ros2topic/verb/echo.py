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

from argparse import ArgumentTypeError

import rclpy
from rclpy.expand_topic_name import expand_topic_name
from rclpy.qos import qos_profile_sensor_data
from rclpy.validate_full_topic_name import validate_full_topic_name
from ros2cli.node.direct import DirectNode
from ros2topic.api import get_topic_names_and_types
from ros2topic.api import import_message_type
from ros2topic.api import TopicNameCompleter
from ros2topic.verb import VerbExtension
from rosidl_runtime_py import message_to_csv
from rosidl_runtime_py import message_to_yaml

DEFAULT_TRUNCATE_LENGTH = 128


def unsigned_int(string):
    try:
        value = int(string)
    except ValueError:
        value = -1
    if value < 0:
        raise ArgumentTypeError('value must be non-negative integer')
    return value


class EchoVerb(VerbExtension):
    """Output messages from a topic."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'topic_name',
            help="Name of the ROS topic to listen to (e.g. '/chatter')")
        arg.completer = TopicNameCompleter(
            include_hidden_topics_key='include_hidden_topics')
        parser.add_argument(
            'message_type', nargs='?',
            help="Type of the ROS message (e.g. 'std_msgs/String')")
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

    def main(self, *, args):
        return main(args)


def main(args):
    if not args.csv:
        truncate_length = args.truncate_length if not args.full_length else None
        callback = subscriber_cb(truncate_length, args.no_arr, args.no_str)
    else:
        truncate_length = args.truncate_length if not args.full_length else None
        callback = subscriber_cb_csv(truncate_length, args.no_arr, args.no_str)
    with DirectNode(args) as node:
        subscriber(
            node.node, args.topic_name, args.message_type, callback)


def subscriber(node, topic_name, message_type, callback):
    if message_type is None:
        topic_names_and_types = get_topic_names_and_types(node=node, include_hidden_topics=True)
        try:
            expanded_name = expand_topic_name(topic_name, node.get_name(), node.get_namespace())
        except ValueError as e:
            raise RuntimeError(e)
        try:
            validate_full_topic_name(expanded_name)
        except rclpy.exceptions.InvalidTopicNameException as e:
            raise RuntimeError(e)
        for n, t in topic_names_and_types:
            if n == expanded_name:
                if len(t) > 1:
                    raise RuntimeError(
                        "Cannot echo topic '%s', as it contains more than one type: [%s]" %
                        (topic_name, ', '.join(t))
                    )
                message_type = t[0]
                break
        else:
            raise RuntimeError(
                'Could not determine the type for the passed topic')

    msg_module = import_message_type(topic_name, message_type)

    node.create_subscription(
        msg_module, topic_name, callback, qos_profile_sensor_data)

    while rclpy.ok():
        rclpy.spin_once(node)


def subscriber_cb(truncate_length, noarr, nostr):
    def cb(msg):
        nonlocal truncate_length, noarr, nostr
        print(message_to_yaml(msg, truncate_length, noarr, nostr), end='---\n')
    return cb


def subscriber_cb_csv(truncate_length, noarr, nostr):
    def cb(msg):
        nonlocal truncate_length, noarr, nostr
        print(message_to_csv(msg, truncate_length, noarr, nostr))
    return cb
